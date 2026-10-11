# ESP32-S3 与 ESP-DL 端侧视觉安防报警系统

毕业设计题目：**基于 ESP32-S3 与 ESP-DL 的端侧人体检测智能安防报警系统设计与实现**。

当前为第四步开发快照：FreeRTOS 框架、OV2640 初始化/采集、经典 ESP-DL
两阶段人脸检测、PIR 防抖触发及有源蜂鸣器/LED 报警已经接入。PIR 二次采样
确认间隔为 30 ms，触发冷却期为 2 秒；LED 每 250 ms 切换一次，有源蜂鸣器
持续鸣叫，单轮报警约 4 秒，结束后关闭两路输出。
**当前模型检测人脸，不能将这一阶段的效果表述为全身人体或行人检测。**
最终若要求识别背影、遮脸或画面内仅出现身体，需要接入真正的行人/人体模型，
并随模型选择调整 SDK、预处理与阈值，不能只更换类名。

## 固定版本

| 项目 | 当前选择 |
| --- | --- |
| 目标芯片 | ESP32-S3，实际硬件须带 PSRAM |
| ESP-IDF | **5.0.x**，建议使用已核对版本信息的 v5.0.6 |
| ESP-DL | **v2.0**，提交 `6f809f6513c71de204aa49eb41d2acced51eb1d7` |
| 摄像头驱动 | `espressif/esp32-camera` 2.0.16 |
| 模型 | `HumanFaceDetectMSR01` + `HumanFaceDetectMNP01` |
| 输入 | RGB565，QVGA 320×240，单帧缓冲区在 PSRAM |
| 报警判定 | 合法检测框，且最终 `score > 0.7` |

ESP-DL v2.0 自身的 `idf_component.yml` 指定 `idf: "5.0.*"`。
因此先前宽泛的“ESP-IDF 5.0 以上”已在本阶段收紧，不能直接使用 ESP-DL
最新版 v3.x 编译这些经典头文件/API。预编译模型库与工具链也应保持这组版本。

## 工程结构

```text
.
├── CMakeLists.txt
├── README.md
├── sdkconfig.defaults
├── partitions.csv
├── components/
│   └── README.md
├── docs/
│   ├── step3.md
│   └── step4.md
└── main/
    ├── CMakeLists.txt
    ├── idf_component.yml
    ├── Kconfig.projbuild
    └── main.cpp
```

`main.c` 已切换为 `main.cpp`，入口为 `extern "C" void app_main(void)`。
初始化结构体使用零初始化后逐字段赋值，兼容 C++17。
第三步的完整核心代码片段与本地组件链接说明见 [docs/step3.md](docs/step3.md)。
第四步的传感器/报警任务实现、运行时序及上板检查步骤见
[docs/step4.md](docs/step4.md)。

## 依赖和编译

在正确的 ESP-IDF **5.0.x** 开发终端中，切换到工程根目录：

```sh
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
```

默认 `main/idf_component.yml` 由 Component Manager 获取摄像头组件，以及
精确提交的 ESP-DL 组件；第一次构建需要网络和 Git。这里没有复制第三方
预编译库到项目，版本和下载来源均明确记录在 manifest 中。

若已经有完整的 `components/esp-dl/`，请确认是上表的 v2.0 版本，删除
manifest 中的 `esp-dl` Git 依赖，保留 `REQUIRES esp-dl`。详见
[components/README.md](components/README.md)。不要同时保留两套不同版本的库。

`REQUIRES esp-dl` 通过组件本身的 CMake 导出头文件目录并链接 ESP32-S3
对应的模型静态库，无需在 main 中重复手动写 `libhuman_face_detect.a` 的路径。
只复制两个 `.hpp` 文件无法完成链接。

## 必须核对的硬件设置

1. **PSRAM**：`Component config → ESP PSRAM`。按实际模组选择 Quad/Octal
   模式及工作频率。8 MB 是容量，不足以确定总线模式；常见 R8 模组通常为
   Octal，最终以型号和原理图为准。`sdkconfig.defaults` 启用了 PSRAM，
   不会替你确定实际总线模式。
2. **Flash**：按实际模组选择 Flash 大小。提供的分区表占用 4 MB 地址空间，
   适用于实际 Flash 至少 4 MB 的设备；该分区表没有 OTA 分区。
3. **C++ 异常**：启用 `CONFIG_COMPILER_CXX_EXCEPTIONS`。代码用
   `try/catch` 处理可恢复的 C++ 异常，并配置 4096 字节异常应急池；这会
   增加少量代码/内存开销。若已有 `sdkconfig`，默认文件不会覆盖其中的值，
   必须到 menuconfig 确认。
4. **接线**：摄像头引脚不是 ESP32-S3-WROOM 的统一标准，须对照实际板卡。

| 信号 | 暂定 GPIO |
| --- | --- |
| PIR / 蜂鸣器 / LED | 1 / 2 / 21 |
| PWDN / RESET | -1 / -1，无 GPIO 控制 |
| XCLK / SIOD / SIOC | 10 / 40 / 39 |
| Y9 / Y8 / Y7 / Y6 | 48 / 11 / 12 / 14 |
| Y5 / Y4 / Y3 / Y2 | 16 / 18 / 17 / 15 |
| VSYNC / HREF / PCLK | 38 / 47 / 13 |

摄像头 XCLK 占用 `LEDC_TIMER_0` 和 `LEDC_CHANNEL_0`，后续 PWM 蜂鸣器
需要使用其他资源。本阶段使用有源蜂鸣器，不需要 PWM；蜂鸣器和 LED
按高电平有效配置。有源蜂鸣器的控制输入需兼容 3.3 V；裸蜂鸣器按额定
电流选择三极管/MOSFET 驱动，LED 串联限流电阻，所有模块共地。

## 第四步验证

默认由真实 PIR 高电平触发视觉任务。HC-SR501 上电后先等待约 30–60 秒
稳定，再测试移动触发。PIR 持续为高电平时，冷却后会再次确认并请求检测。

也可在 menuconfig 的 `Security application` 中启用
`Capture and infer once after startup (development test)`，启动后会向视觉
队列发送一次测试触发；此路径确认人脸后同样会启动真实声光报警。

烧录、查看日志（端口按实际设备修改）：

```sh
idf.py -p COM5 flash monitor
```

观察摄像头初始化成功、`Face detectors ready`、320×240/153600 字节采集信息，
以及推理耗时、候选框数量、最终框数量、通过阈值的数量。框坐标为
`[左上 x, 左上 y, 右下 x, 右下 y]`，对应当前图像坐标系。

PIR 确认后应看到 `PIR motion confirmed; vision requested`。画面出现合格
人脸时，应看到 `Alarm started`，LED 亮/灭各 250 ms，蜂鸣器持续鸣叫，
约 4 秒后看到 `Alarm finished; LED and buzzer are off`。随后两路输出
恢复低电平；报警期间新增的确认信号最多保留一个，可能立即启动下一轮。

背对摄像头不应作为这一人脸模型必须检出的测试用例。阈值 0.7 是初始值，
不是已标定的准确率，也不代表“70% 识别准确率”。详细检查见第四步文档。

## 帧和任务资源约定

- 视觉任务独占模型对象和帧消费过程；检测器在任务循环外构造一次。
- 每个非空帧由 `CameraFrameGuard` 归还一次。正常路径、检查失败的
  `continue`、C++ 异常展开都执行 `esp_camera_fb_return()`。
- `fb` 与 `fb->buf` 属于摄像头驱动，不调用 `free()`。归还帧后不再访问它们。
- 两个推理结果列表为检测器内部所有，使用引用解析，不拷贝、不手动释放。
  在下一次同一检测器的 `infer()` 前完成读取。
- 完成推理并归还帧后，最多发送一个报警信号；两个信号队列长度均为 1。
- PIR/报警任务使用 `vTaskDelay()` 让出 CPU，不忙等，也不申请图像缓冲区。
- 报警正常结束或 GPIO 激活/闪烁失败均执行关闭流程；关闭任一路输出失败
  时，每秒重试两路关闭，不开始新的报警轮次。
- 启动事件位只在三个任务全部创建成功后设置。启动失败时任务尚未初始化
  模型，可以先删任务、再释放队列/事件组并反初始化摄像头。
- 不可在运行期直接调用这段“启动失败清理”；FreeRTOS 外部删除 C++ 任务
  不会自动展开任务栈上的对象。运行期停机需要另行实现协作式退出。

模型旧版接口没有 `esp_err_t` 式推理错误返回值；可以捕获的 C++ 异常会
被记录。预编译库内部的断言、`abort()` 或硬件异常不属于可捕获异常，
仍可能导致系统复位，不将 `try/catch` 表述为覆盖所有内存故障。

## 当前验证状态和下一步

已对照官方 v2.0 头文件、示例、组件 CMake 和 manifest 检查构造参数、
RGB565 输入类型、结果结构及链接关系，并审阅第四步的触发、防抖、冷却、
报警时序和 GPIO 错误关闭路径。本机未安装 ESP-IDF/Xtensa 工具链，也没有
实物，尚未完成固件交叉编译、链接、上板推理、实际电平或准确率测量。

后续模块：可选 Wi-Fi/MQTT；真正的全身人体模型；连续触发稳定性与
内存测量；根据实际硬件与场景调整 PIR 及报警策略。

官方依据：

- [ESP-DL v2.0 README](https://github.com/espressif/esp-dl/blob/v2.0/README_cn.md)
- [MSR01 接口](https://github.com/espressif/esp-dl/blob/v2.0/include/model_zoo/human_face_detect_msr01.hpp)
- [MNP01 接口](https://github.com/espressif/esp-dl/blob/v2.0/include/model_zoo/human_face_detect_mnp01.hpp)
- [人脸检测示例](https://github.com/espressif/esp-dl/blob/v2.0/examples/human_face_detect/main/app_main.cpp)
