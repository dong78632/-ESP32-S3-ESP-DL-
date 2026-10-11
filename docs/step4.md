# 第四步：PIR 传感器与声光报警

当前完整实现位于 `main/main.cpp`。本阶段沿用长度为 1 的视觉/报警队列、
启动事件组及第三步的 ESP-DL 推理和摄像头帧管理。

## 运行行为

| 参数 | 默认值 |
| --- | --- |
| PIR 引脚/有效电平 | GPIO1 / 高电平 |
| 空闲轮询周期 | 100 ms |
| 高电平二次确认间隔 | 30 ms |
| 确认后的触发冷却期 | 2000 ms |
| LED 引脚/有效电平 | GPIO21 / 高电平 |
| 有源蜂鸣器引脚/有效电平 | GPIO2 / 高电平 |
| LED 亮/灭时长 | 各 250 ms |
| 报警周期数/正常总时长 | 8 个周期 / 约 4000 ms |
| GPIO 关闭失败重试间隔 | 1000 ms |

PIR 首次采到高电平后，延时 30 ms，再采到高电平才向视觉队列发送
`SIGNAL_PIR_TRIGGERED`。这是一种简单的二次采样防抖；不声称监测了
两次采样之间的全部电平变化。低电平或二次确认失败时按 100 ms 周期轮询。

确认高电平后，即使队列发送失败也进入 2 秒冷却，避免连续快速重试。
持续高电平会在冷却结束后再次接受二次采样确认；正常情况下连续请求间隔
约 2030 ms，加上调度和日志开销。HC-SR501 输出通常维持数秒，该行为允许
对持续活动继续检测。本阶段没有改为必须等到低电平后才能重新触发的策略。

报警任务阻塞等待 `SIGNAL_HUMAN_CONFIRMED`，收到后开启有源蜂鸣器，
LED 执行 8 次“亮 250 ms、灭 250 ms”，正常单轮约 4 秒。所有延时使用
`vTaskDelay()` 让出 CPU，实际时长受 FreeRTOS tick 和调度影响。

每轮结束或 GPIO 激活/闪烁失败，都进入关闭两路输出的公共流程。两次
关闭写入都会尝试，不会因第一路失败而跳过第二路。关闭失败时记录错误，
每秒重试；只有两路关闭 API 均返回成功才等待/处理下一轮。API 返回成功
表示驱动接受了写入；实际接线、电平和蜂鸣器状态仍需上板确认。

报警进行期间不清空队列：长度为 1 的覆盖队列最多保留一个待处理确认，
因此报警结束并关闭输出后，若已有信号等待，可能立即开始下一轮。

## 核心代码

以下 GPIO 初始化、队列创建和 `wait_for_startup()` 均沿用现有工程。

```cpp
#define BUZZER_ACTIVE_LEVEL         1
#define BUZZER_INACTIVE_LEVEL       0
#define LED_ACTIVE_LEVEL            1
#define LED_INACTIVE_LEVEL          0
#define SENSOR_POLL_PERIOD_MS      100
#define PIR_ACTIVE_LEVEL           1
#define PIR_DEBOUNCE_MS             30
#define PIR_COOLDOWN_MS             2000
#define ALARM_HALF_PERIOD_MS        250
#define ALARM_CYCLE_COUNT           8
#define ALARM_GPIO_RETRY_MS         1000
```

### PIR 任务

```cpp
static void sensor_task(void *arg)
{
    (void)arg;
    wait_for_startup();
    ESP_LOGI(TAG, "PIR task started: debounce=%d ms, cooldown=%d ms",
             PIR_DEBOUNCE_MS, PIR_COOLDOWN_MS);

    for (;;) {
        if (gpio_get_level(PIR_GPIO) == PIR_ACTIVE_LEVEL) {
            /* Confirm a second high sample after a short debounce delay. */
            vTaskDelay(pdMS_TO_TICKS(PIR_DEBOUNCE_MS));
            if (gpio_get_level(PIR_GPIO) == PIR_ACTIVE_LEVEL) {
                const app_signal_t signal = SIGNAL_PIR_TRIGGERED;
                if (xQueueOverwrite(s_vision_queue, &signal) == pdPASS) {
                    ESP_LOGI(TAG, "PIR motion confirmed; vision requested");
                } else {
                    ESP_LOGE(TAG, "Failed to enqueue PIR trigger");
                }

                /* Rate-limit confirmed levels, including a failed enqueue.
                 * A sustained high level can trigger again after cooldown.
                 */
                vTaskDelay(pdMS_TO_TICKS(PIR_COOLDOWN_MS));
                continue;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_PERIOD_MS));
    }
}
```

### GPIO 写入、关闭流程及报警任务

```cpp
static bool set_alarm_gpio(gpio_num_t pin, int level)
{
    const esp_err_t err = gpio_set_level(pin, level);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GPIO %d to %d: %s",
                 static_cast<int>(pin), level, esp_err_to_name(err));
        return false;
    }
    return true;
}

static void ensure_alarm_outputs_off(void)
{
    for (;;) {
        /* Always attempt both writes; do not short-circuit on the first error. */
        const bool led_off = set_alarm_gpio(LED_GPIO, LED_INACTIVE_LEVEL);
        const bool buzzer_off =
            set_alarm_gpio(BUZZER_GPIO, BUZZER_INACTIVE_LEVEL);
        if (led_off && buzzer_off) {
            return;
        }
        ESP_LOGE(TAG, "Alarm outputs could not be switched off; retrying");
        vTaskDelay(pdMS_TO_TICKS(ALARM_GPIO_RETRY_MS));
    }
}

static void alarm_task(void *arg)
{
    (void)arg;
    wait_for_startup();
    ensure_alarm_outputs_off();
    ESP_LOGI(TAG, "Alarm task started");

    app_signal_t signal;
    for (;;) {
        if (xQueueReceive(s_alarm_queue, &signal, portMAX_DELAY) != pdTRUE ||
            signal != SIGNAL_HUMAN_CONFIRMED) {
            continue;
        }

        ESP_LOGI(TAG, "Alarm started: duration approximately %d ms",
                 ALARM_CYCLE_COUNT * 2 * ALARM_HALF_PERIOD_MS);
        /* Active buzzer sounds continuously; LED flashes at 250 ms per phase. */
        bool alarm_ok = set_alarm_gpio(BUZZER_GPIO, BUZZER_ACTIVE_LEVEL);
        if (alarm_ok) {
            for (int cycle = 0; cycle < ALARM_CYCLE_COUNT; ++cycle) {
                if (!set_alarm_gpio(LED_GPIO, LED_ACTIVE_LEVEL)) {
                    alarm_ok = false;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(ALARM_HALF_PERIOD_MS));

                if (!set_alarm_gpio(LED_GPIO, LED_INACTIVE_LEVEL)) {
                    alarm_ok = false;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(ALARM_HALF_PERIOD_MS));
            }
        }

        /* Common cleanup for normal completion and every GPIO-write error. */
        ensure_alarm_outputs_off();
        if (alarm_ok) {
            ESP_LOGI(TAG, "Alarm finished; LED and buzzer are off");
        } else {
            ESP_LOGE(TAG, "Alarm aborted after GPIO error; outputs are off");
        }
        /* A single coalesced confirmation may already be waiting in the queue. */
    }
}
```

## 接线和上板检查

这些 GPIO 是暂定值，须对照板卡原理图，保持与摄像头引脚不冲突。
HC-SR501 OUT 接 PIR GPIO，模块与 ESP32 共地；供电按具体模块规格选择，
输入到 ESP32 的信号必须符合 3.3 V GPIO 电平要求。

当前驱动适用于高电平有效的有源蜂鸣器模块，不需要 PWM。裸蜂鸣器按
额定电流选用三极管/MOSFET 驱动；LED 串联适当的限流电阻。低电平触发
模块需要同时修改 ACTIVE/INACTIVE 宏，并核对上电默认状态。

1. 使用 ESP-IDF 5.0.x 构建并烧录，确认 GPIO 初始化无错误，两路输出默认关闭。
2. HC-SR501 上电后等待约 30–60 秒稳定，在触发覆盖区内移动，确认出现
   `PIR motion confirmed; vision requested` 和随后图像采集/推理日志。
3. 测试一次被第二次采样判为低电平的脉冲，确认不触发视觉队列；测试持续
   高电平时，确认请求间隔约 2 秒以上。受轮询采样位置影响，不能声称所有
   短脉冲都会被过滤。
4. 画面出现通过阈值的人脸时，确认 LED 亮/灭各 250 ms、蜂鸣器连续鸣叫，
   单轮约 4 秒；结束后测量两路 GPIO 回到低电平。
5. 无合格人脸时，确认仅采集/推理，不启动声光报警。当前模型仍是人脸模型。
6. 在报警期间连续触发，确认待处理报警最多保留一个，且每轮结束都执行关闭。
7. GPIO 故障路径需要通过受控故障注入验证：激活/闪烁失败应进入关闭流程，
   关闭失败应每秒重试两路，期间不启动下一轮。尚未完成这项运行时测试。

`Capture and infer once after startup` 开发选项仍可使用；这一测试路径
确认人脸后会启动真实声光输出，正式运行时按需要关闭。

## 当前验证状态

已审阅传感器采样、防抖、队列通知、冷却、正常报警时序和 GPIO 错误处理。
PIR/报警任务没有新增动态内存分配，视觉任务的 RAII 帧管理保持原样。
当前环境没有 ESP-IDF/Xtensa 编译工具链和实际硬件，尚未完成固件编译、
实际报警时长/电平测量或硬件故障注入测试。上述步骤是上板验证流程。

