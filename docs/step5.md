# 第五步：Wi-Fi 与 MQTT 报警推送

启动顺序：电源/GPIO/NVS → Wi-Fi 初始化和等待 IP（最多 30 秒）→ 摄像头检查
→ 创建本地应用任务。MQTT 客户端句柄在网络监督任务创建前初始化一次；仅在
首次获取 IP 后启动 MQTT。SSID 仍为占位符时不启动网络；错误密码、热点离线
或 DHCP 超时不会阻止本地任务启动。
当前还接入了自动 Light Sleep：开头配置电源管理，摄像头检查及每轮采集/推理
均在电源锁保护下运行；检查或推理结束后关闭摄像头。Wi-Fi 使用
`WIFI_PS_MIN_MODEM` 协调省电，详情见 [电源管理说明](power-management.md)。

## 使用前配置

把 `main/security_secrets.example.h` 复制成 Git 忽略的 `main/security_secrets.h`，
填写 SSID、密码、Broker URI、主题。不要在公开源码中保存真实密码或 Token。
默认配置：

```cpp
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASS       "YOUR_WIFI_PASSWORD"
#define MQTT_BROKER_URI "mqtt://broker.emqx.io"
#define MQTT_TOPIC      "esp32s3/security/alert"
```

当前 Broker 使用未加密、无项目级鉴权的公共 MQTT 服务，只适合联调。主题也
是公共的，可能与其他用户冲突。正式部署需要改为 `mqtts://`、配置服务端证书
和账号认证，并把 MAC、芯片 ID 或项目编号加入主题。

## 报警载荷与内存

每次 `SIGNAL_HUMAN_CONFIRMED` 到达时，报警任务先开启蜂鸣器，把报警开始时的
时间戳用零等待方式交给独立 MQTT 工作任务，并执行 4 秒声光报警。工作任务生成：

```json
{"device":"ESP32-S3","alert":"Human Detected","timestamp":123456789}
```

`timestamp` 是 `esp_timer_get_time()` 返回的自启动以来微秒数，不是 Unix
时间。cJSON 对象由 `cJSON_CreateObject()` 创建，字符串由
`cJSON_PrintUnformatted()` 分配；本实现的公共退出路径始终执行
`free(payload)` 和 `cJSON_Delete(root)`。即使 MQTT 未初始化、发布失败或
JSON 序列化失败，也不会把本轮对象或字符串留到下一轮。
`Human Detected` 保留约定载荷，但当前确认依据是人脸，不是全身人体模型。

工作任务在 MQTT 连接后调用 `esp_mqtt_client_enqueue(..., 1, 0, true)`；SDK
复制消息并由内部任务发送，应用任务不直接调用同步 socket 发布。

限制和丢弃策略：

- 应用队列最多 8 条时间戳，加上工作任务当前 1 条；队列满时丢弃新网络报警并记录日志。
- 等待发送超过 5 分钟的报警丢弃；重启后不会保留这些内存消息。
- outbox 中编码消息总大小最多 4096 字节，不含每条链表节点/分配器开销；
  此上限依赖本工作任务是唯一生产者，新增发布者必须共享限流机制。
- SDK 默认文件设置未确认 outbox 消息 5 分钟过期，并报告删除事件。
  已有 sdkconfig 需在 menuconfig 核对上述 MQTT 配置。
- QoS 1 可能重复；它不消除队列满、过期、断电造成的消息丢失。

## 持续重连

事件回调只记录状态，不在回调中阻塞或执行重试循环。网络监督任务串行发起连接，
每次最多等 10 秒获取 IP；每 5 次失败后冷却 30 秒，其余失败等待 2 秒，
始终继续尝试。成功后重置重试计数。MQTT 启动失败也会后台重试；正常 MQTT
断线由 ESP-MQTT 自动恢复。当前最低认证要求为 WPA2，不支持开放热点或企业认证配置。

## 组件依赖

`main/CMakeLists.txt` 显式声明 `esp_wifi`、`esp_event`、`esp_netif`、
`mqtt` 和 `json`，并保留摄像头、ESP-DL、NVS、GPIO 与定时器依赖。

## 联调检查

1. 使用 ESP-IDF 5.0.x 构建并烧录，确认 MQTT 在 Wi-Fi IP 后启动；
   断网/错误密码启动最多等待 30 秒，然后仍能本地检测报警。
2. 使用 MQTTX、MQTT Explorer 或命令行客户端连接 `broker.emqx.io:1883`，
   订阅 `esp32s3/security/alert`。
3. 触发 PIR 并让画面出现通过阈值的人脸，确认本地声光报警和 JSON 消息均
   出现，且消息 QoS 为 1。
4. 断开热点超过 5 次重试，确认固件继续重试；恢复热点后确认重新获得 IP，
   MQTT 客户端随后自动恢复连接。
5. 连续触发并观察剩余堆内存，确认没有随报警次数持续下降。还应分别验证
   Broker 离线、错误 Wi-Fi 密码和 DNS 不可用等故障场景。

主机测试只覆盖 API 模拟和内存清理，编译结果见 [审查报告](project-audit.md)。
Wi-Fi/MQTT 实机联调、Broker 离线时的本地报警延迟和长期内存测试仍需上板完成。
