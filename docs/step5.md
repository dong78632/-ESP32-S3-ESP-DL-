# 第五步：Wi-Fi 与 MQTT 报警推送

本阶段在第四步声光闭环上增加 Wi-Fi STA 和 MQTT。`app_main()` 初始化 NVS
后阻塞等待 Wi-Fi 获取 IPv4 地址，随后启动 MQTT 客户端，再检查摄像头并
创建传感器、视觉和报警任务。启动连接连续失败 5 次时，设备延时 3 秒重启。
当前还接入了自动 Light Sleep：开头配置电源管理，摄像头检查及每轮采集/推理
均在电源锁保护下运行；检查或推理结束后关闭摄像头。Wi-Fi 使用
`WIFI_PS_MIN_MODEM` 协调省电，详情见 [电源管理说明](power-management.md)。

## 使用前配置

在 `main/main.cpp` 中替换以下占位符：

```cpp
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASS       "YOUR_WIFI_PASSWORD"
#define MQTT_BROKER_URI "mqtt://broker.emqx.io"
#define MQTT_TOPIC      "esp32s3/security/alert"
#define MAX_RETRY       5
```

当前 Broker 使用未加密、无项目级鉴权的公共 MQTT 服务，只适合联调。主题也
是公共的，可能与其他用户冲突。正式部署需要改为 `mqtts://`、配置服务端证书
和账号认证，并把 MAC、芯片 ID 或项目编号加入主题。

## 报警载荷与内存

每次 `SIGNAL_HUMAN_CONFIRMED` 到达报警任务时，系统先构造并提交 QoS 1
消息，然后执行原有声光报警。载荷格式如下：

```json
{"device":"ESP32-S3","alert":"Human Detected","timestamp":123456789}
```

`timestamp` 是 `esp_timer_get_time()` 返回的自启动以来微秒数，不是 Unix
时间。cJSON 对象由 `cJSON_CreateObject()` 创建，字符串由
`cJSON_PrintUnformatted()` 分配；本实现的公共退出路径始终执行
`free(payload)` 和 `cJSON_Delete(root)`。即使 MQTT 未初始化、发布失败或
JSON 序列化失败，也不会把本轮对象或字符串留到下一轮。

## 组件依赖

`main/CMakeLists.txt` 显式声明 `esp_wifi`、`esp_event`、`esp_netif`、
`mqtt` 和 `json`，并保留摄像头、ESP-DL、NVS、GPIO 与定时器依赖。

## 联调检查

1. 使用 ESP-IDF 5.0.x 构建并烧录，确认日志先出现 Wi-Fi IP，再出现 MQTT
   连接和检测框架就绪信息。
2. 使用 MQTTX、MQTT Explorer 或命令行客户端连接 `broker.emqx.io:1883`，
   订阅 `esp32s3/security/alert`。
3. 触发 PIR 并让画面出现通过阈值的人脸，确认本地声光报警和 JSON 消息均
   出现，且消息 QoS 为 1。
4. 暂时断开热点，确认固件记录断线并自动重连；恢复热点后确认重新获得 IP，
   MQTT 客户端随后自动恢复连接。
5. 连续触发并观察剩余堆内存，确认没有随报警次数持续下降。还应分别验证
   Broker 离线、错误 Wi-Fi 密码和 DNS 不可用等故障场景。

当前服务器未安装 ESP-IDF/Xtensa 工具链，本阶段只完成源码、依赖和静态
检查；固件编译、Wi-Fi/MQTT 实机联调和长期内存测试仍需在开发环境完成。
