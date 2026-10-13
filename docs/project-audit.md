# 完整项目审查与验证记录

审查日期：2026-10-12。基线提交：`c4e680b`。
范围：应用源码、配置、组件依赖、分区、资源生命周期、网络异常路径、
硬件适配入口、文档与主机测试。没有实物，不包含实际准确率、电流或无线电测量。

## 已修复/完善

| 问题 | 影响 | 当前处理 |
| --- | --- | --- |
| C++ MQTT 事件注册传入裸 `ESP_EVENT_ANY_ID` | ESP-IDF 5.0.6 实际编译失败 | 显式转换为 `esp_mqtt_event_id_t`，交叉编译验证 |
| Wi-Fi 失败 5 次后永久停止连接 | 热点恢复后仍离线 | 独立监督任务，5 次一组，持续重试；连接成功清零 |
| 启动网络失败就反复重启 | 无网络时没有本地防护 | 最多等 IP 30 秒，后台重连，本地继续启动 |
| 同步 MQTT 发布先于蜂鸣器启动 | socket 超时拖延本地报警 | 先开启蜂鸣器，零等待交给独立工作任务，使用 SDK enqueue |
| 离线 QoS 消息缺少内存限制 | 持续报警可能耗尽内存 | 8 条应用队列、1 条工作项、4096 字节编码 outbox 上限和过期日志 |
| tickless sleep 默认隔离 GPIO | 蜂鸣中断或有效低电平模块浮空 | 报警两路 `gpio_sleep_sel_dis()`；初始化前预装关闭电平 |
| 省电不能单独关闭 | 首次联调难以建立基线 | menuconfig 睡眠开关；支持 PM 完全关闭构建 |
| GPIO/板型硬编码、无冲突检查 | 接错相机或使用存储保留引脚 | Custom/Freenove 映射、可配置外设、电平和启动校验 |
| 密码需写入公开 main.cpp | 容易随源码泄露 | Git 忽略的本地 secrets 头文件和无秘密示例 |
| PIR 上电不稳定、摄像头首帧曝光 | 首次误触发或错过检测 | 默认 PIR 预热 60 秒、丢弃相机前 2 帧，均可配置 |
| 接受退化或越界检测框 | 不合法输出可能触发报警 | 非零宽高、坐标有序及图像边界检查 |
| 模型或本地任务初始化失败后静默退出 | 系统表面在线但无检测 | 明确错误日志、清理启动资源并重启恢复 |
| 默认 Flash 大小与 4 MB 分区表不一致 | 分区大小构建检查失败 | 默认 Flash 改为 4 MB，仍需匹配实物 |
| 只有总堆余量、缺少故障回归 | 难定位 SRAM 压力和失败清理 | 分别记录 SRAM/PSRAM 空闲量与最大块，增加策略/JSON 失败测试 |

网络 SDK 初始化本身的不可恢复错误仍可能由 `ESP_ERROR_CHECK` 停止/复位；
离线保障针对连接失败/断线，不是对所有 SDK 或硬件故障的容错承诺。
相机初始化/模型初始化等本地严重故障会明确复位；若接线或内存配置持续错误，
复位不会自动修好，必须根据日志处理。

## 已执行验证

工具链：官方 ESP-IDF **v5.0.6**，Xtensa ESP32-S3 GCC **11.2.0**。
ESP-DL v2.0 固定提交、esp32-camera 2.0.16、传递依赖 esp_jpeg 1.3.1
成功解析、编译和链接；生成并保留 `dependencies.lock`。

验证配置：

- Custom + Quad PSRAM + Wi-Fi/MQTT + 自动睡眠。
- Custom + Quad PSRAM + 网络关闭 + PM/tickless 关闭。
- Freenove + Octal PSRAM + Wi-Fi/MQTT + 动态调频开启、自动睡眠关闭。
- 占位 SSID 配置，验证自动降级离线的构建。

联网构建使用临时虚构测试 SSID/密码，以确保网络路径不仅通过语法检查，
也实际链接进镜像；测试配置没有提交到仓库。所有镜像通过 4 MB 分区容量检查。
这些编译变体不代表相应 PSRAM 或引脚已在实物上验证。

最终验证镜像大小（单 factory 应用分区为 `0x3f0000` 字节）：

| 构建变体 | 应用镜像大小 | 分区剩余 |
| --- | --- | --- |
| 网络凭据已配置 + 自动睡眠 | `0x19ba20` | 59% |
| 离线 + PM 关闭 | `0x0fdf70` | 75% |
| Freenove + Octal + 睡眠关闭 | `0x19c890` | 59% |
| 占位 SSID 自动离线 | `0x104c80` | 74% |

镜像含测试配置/未最终选择的硬件参数，仅用于构建验证，不作为可直接烧录的交付件。

主机测试：

```sh
python3 tests/test_power_lifecycle.py
python3 tests/test_runtime_policy.py
python3 tests/test_mqtt_cleanup.py
git diff --check
```

结果均通过：

- PM 开/关，正常/提前退出/异常/空帧/相机初始化失败，1000 次清理顺序回归。
- 1000 组重连退避循环、成功后重置、检测框边界、消息到期及 outbox 上限。
- JSON 创建、逐字段添加和打印的模拟分配失败，空 MQTT 句柄、提交失败和
  outbox 满，1000 轮后对象/字符串无残留，QoS 1 和复制入队参数正确。

主机 API mock 验证的是应用清理和策略，不能证明 SDK 内部无泄漏、摄像头
并发安全、Wi-Fi 稳定或实际睡眠成功。没有将测试循环数表述为实机连续报警数。

## 复现构建

在 ESP-IDF v5.0.6 终端中运行。首次下载组件需要网络和 Git；具体命令中的
`SDKCONFIG` 使用独立、Git 忽略文件，避免互相覆盖配置：

```sh
# 常规默认配置
idf.py -B build-default -DIDF_TARGET=esp32s3 -DSDKCONFIG=sdkconfig.default.local build

# 离线且关闭 PM
idf.py -B build-offline -DIDF_TARGET=esp32s3 -DSDKCONFIG=sdkconfig.offline.local "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;config/offline.defaults" build

# Freenove 模板：先核对板卡版本、Octal PSRAM 和所有引脚
idf.py -B build-freenove -DIDF_TARGET=esp32s3 -DSDKCONFIG=sdkconfig.freenove.local "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;config/freenove.defaults" build
```

配置默认文件只用于初始化新 sdkconfig。已有 sdkconfig 不会被 defaults
强行覆盖；改用 menuconfig 修改，或给新验证变体指定一个新的配置文件。
实际 Wi-Fi 使用本地 `main/security_secrets.h`，正式部署不要用公开 Broker。
`dependencies.lock` 是 Component Manager 2.x 生成的锁文件；旧管理器遇到
锁格式不兼容时，在 ESP-IDF 的 Python 环境升级管理器后再构建，避免手改哈希。

## 必须保留的项目边界

1. **当前不是全身人体检测模型**。只检测人脸，背影、遮脸、无脸身体可能漏检；
   论文若要求全身人体/行人检测，需要单独选择和集成真正的人体模型，并重测性能。
2. **公共明文 MQTT 不适用于真实安防**。生产部署还需私有 Broker、账号认证、
   TLS/证书验证、唯一设备主题，以及接收端去重。仅改 URI 不足以完成安全接入。
3. **不是持久化离线存储**。队列满、过期、断电或复位会丢失报警，QoS 1 也可能重复。
4. **软件不能保证复位时蜂鸣器关闭**。上电到初始化前应由硬件上拉/下拉/驱动保证
   安全电平；GPIO 不能直接给大电流蜂鸣器供电。
5. **没有 OTA、断电续传或防拆功能**。当前分区是单 factory 应用，用 USB 更新。
6. **版本有意固定在旧版模型兼容组合**。没有把 ESP-IDF 5.0.6/ESP-DL v2.0
   视为生产安全维护基线；后续升级 SDK 必须同步迁移模型/API 并重做验证。

## 到货后的验收顺序

按 [硬件联调指南](hardware-bringup.md) 先离线、关闭自动睡眠建立本地闭环，
再启用网络和省电。至少记录：

- 实物模组、Flash/PSRAM 配置与引脚；首次上电安全电平及 PIR 预热。
- 相机画面、首帧曝光、光照/距离条件、正脸/背影/遮脸/无人的误报漏报。
- PIR → 推理 → 声光总延迟，联网及 Broker 断线时报警不能等待 socket。
- 热点离线启动、断线超过 5 次重试、恢复热点、DNS/Broker 不可用与恢复。
- 消息满队列/过期、长时间反复触发、SRAM/PSRAM 最大块及任务栈余量。
- 省电前后整板电流、温升和 MQTT 在线时长，而非仅用日志宣布功耗达标。

GitHub PAT、真实 Wi-Fi 密码不得写入仓库。曾在聊天或其他位置公开过的 Token
应在 GitHub 撤销，重新授权时优先使用本机安全登录，不在聊天粘贴新凭据。
