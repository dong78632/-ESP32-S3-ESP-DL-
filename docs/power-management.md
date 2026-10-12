# 自动 Light Sleep 与动态调频

本实现沿用传感器、视觉、报警三个任务和 Wi-Fi/MQTT。`app_main()` 开头
调用 `esp_pm_configure()`，设置最大 CPU 频率 240 MHz、最小频率 40 MHz，
并允许系统在满足空闲条件时自动 Light Sleep。Wi-Fi 使用
`WIFI_PS_MIN_MODEM`，驱动按路由器的 DTIM 节奏协调无线电睡眠和唤醒，
维持关联。应用没有为日常空闲调用 `esp_wifi_stop()` 或手动睡眠 API。

## 编译配置

`sdkconfig.defaults` 已加入：

```ini
CONFIG_PM_ENABLE=y
CONFIG_FREERTOS_USE_TICKLESS_IDLE=y
```

对于已有 `sdkconfig`，执行 `idf.py menuconfig`，按 `/` 搜索 `PM_ENABLE`
和 `FREERTOS_USE_TICKLESS_IDLE` 并启用，然后构建。Tickless Idle 是必要
条件；缺少这两个配置时，本工程会在编译期给出明确错误。

本工程固定 ESP-IDF 5.0.x，使用该版本的 `esp_pm_config_esp32s3_t`：

```cpp
esp_pm_config_esp32s3_t pm_config{};
pm_config.max_freq_mhz = 240;
pm_config.min_freq_mhz = 40;
pm_config.light_sleep_enable = true;
ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
```

`main/CMakeLists.txt` 已增加 `REQUIRES esp_pm`。

## 摄像头和电源锁

摄像头 v2.0.16 初始化后会启动驱动任务并启用 VSYNC 中断。其 ESP32-S3
底层直接使用 LCD_CAM/GDMA，没有自动为采集持有 PM 锁。仅在 `infer()`
两侧加 CPU 锁，会留下驱动运行期间进入睡眠的风险，也会让空闲受到摄像头
事件干扰。因此这里使用公开的 `esp_camera_init()`/`esp_camera_deinit()`
按需管理摄像头；模型仍在视觉任务循环外构造一次并重复使用。

每轮生命周期是：

1. 阻塞等待 PIR 信号，此时不持有视觉/摄像头电源锁。
2. `VisionPowerGuard` 获取 `ESP_PM_NO_LIGHT_SLEEP` 和
   `ESP_PM_CPU_FREQ_MAX`，使整个摄像头生命周期和推理运行在 240 MHz。
3. `CameraSessionGuard` 初始化摄像头，随后获取帧并推理。
4. 离开本轮作用域时，依次归还帧、反初始化摄像头、释放 CPU 锁和禁止
   睡眠锁，再通知报警任务。

这三个 RAII 对象覆盖正常退出、空帧、非法图像、提前 `continue` 和 C++
异常。摄像头初始化失败时不重复反初始化驱动，仍释放电源锁；摄像头关闭或
锁操作失败则由 `ESP_ERROR_CHECK` 停止当前执行，避免继续在不安全状态睡眠。
启动阶段也在同样的锁保护下检查摄像头并立即关闭。

## 延迟与功耗边界

FreeRTOS 定时唤醒仍支持现有 100 ms PIR 轮询，不需要额外接线。这里没有
改为 GPIO 唤醒中断，所以 PIR 首次采样延迟可能达到约 100 ms，之后还有
30 ms 二次确认，以及摄像头重新初始化、采集和推理时间。

芯片退出 Light Sleep 的时间不等于报警总响应时间。按需初始化摄像头会增加
响应开销，并产生重复驱动/帧缓冲分配；需要测量首帧质量、最大空闲内存块和
长时间运行的稳定性。`CAM_PIN_PWDN=-1` 表示摄像头没有独立 GPIO 断电，
传感器、板载指示灯和稳压器仍可能耗电。

实际睡眠时长由 Wi-Fi、MQTT、其他驱动锁、定时器和任务共同决定，配置成功
不表示设备持续睡眠。“几十 mA”应作为待测目标，实际功耗和温升取决于板卡、
PSRAM、外设、信号强度及路由器配置。

## 验证

已执行的主机测试：

```sh
python3 tests/test_power_lifecycle.py
git diff --check
```

测试直接提取生产代码中的三个 RAII 类，用 C++17 编译并模拟 API 调用，
验证清理顺序、异常/失败路径和 1000 次连续循环。主机模拟不能验证真实
ESP32 的睡眠、驱动并发或无线电行为；当前服务器没有 Xtensa/ESP-IDF 工具链。

上板后检查：

1. 启动日志显示 `Power management ready: CPU 40-240 MHz` 和
   `Camera ready for on-demand capture`。
2. 在不触发 PIR 时测量整板平均电流；确认 MQTT 连接可持续运行，没有因
   日常空闲频繁重连。必要时在 menuconfig 开启 PM profiling 并用
   `esp_pm_dump_locks(stdout)` 检查哪些锁仍被持有。
3. 连续触发 PIR，确认每次都能重新初始化摄像头、输出合法帧、完成推理并
   发布报警；测量从 PIR 到声光输出的总延迟和首帧成像质量。
4. 长时间重复触发，检查剩余内存及最大空闲块，验证没有锁计数或驱动资源
   累积；测试 Wi-Fi 网络短暂断开后的恢复。

官方依据：

- [ESP-IDF v5.0.6 ESP32-S3 电源管理](https://docs.espressif.com/projects/esp-idf/en/v5.0.6/esp32s3/api-reference/system/power_management.html)
- [ESP32-S3 PM 配置结构](https://github.com/espressif/esp-idf/blob/v5.0.6/components/esp_pm/include/esp32s3/pm.h)
- [摄像头 v2.0.16 初始化与关闭](https://github.com/espressif/esp32-camera/blob/v2.0.16/driver/esp_camera.c)
- [摄像头 v2.0.16 HAL 任务与 VSYNC](https://github.com/espressif/esp32-camera/blob/v2.0.16/driver/cam_hal.c)
