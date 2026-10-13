#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <inttypes.h>
#include <list>
#include <new>
#include <vector>

#include "cJSON.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"
#include "esp_pm.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "app_config.hpp"
#include "runtime_policy.hpp"

/* Classic ESP-DL v2.0 APIs, used with ESP-IDF 5.0.x. */
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"

#if !CONFIG_COMPILER_CXX_EXCEPTIONS
#error "Enable CONFIG_COMPILER_CXX_EXCEPTIONS for model/STL allocation handling"
#endif

#if CONFIG_SECURITY_AUTO_LIGHT_SLEEP && (!CONFIG_PM_ENABLE || !CONFIG_FREERTOS_USE_TICKLESS_IDLE)
#error "Enable CONFIG_PM_ENABLE and CONFIG_FREERTOS_USE_TICKLESS_IDLE"
#endif

#define CAMERA_XCLK_FREQ_HZ         20000000
#define CAMERA_RESTART_DELAY_MS     1000

#define MAX_RETRY                   5
#define WIFI_ATTEMPT_TIMEOUT_MS     10000
#define WIFI_STARTUP_TIMEOUT_MS     30000
#define PM_MAX_CPU_FREQ_MHZ         240
#define PM_MIN_CPU_FREQ_MHZ         40

/* ESP-IDF xTaskCreate() stack sizes are specified in bytes. */
#define SENSOR_TASK_STACK_SIZE     2048
#define VISION_TASK_STACK_SIZE     8192
#define ALARM_TASK_STACK_SIZE      4096
#define MQTT_TASK_STACK_SIZE       4096
#define NETWORK_TASK_STACK_SIZE    4096

#define SENSOR_TASK_PRIORITY       5
#define VISION_TASK_PRIORITY       4
#define ALARM_TASK_PRIORITY        6

/* Length 1 permits xQueueOverwrite(): repeated pending signals are coalesced. */
#define SIGNAL_QUEUE_LENGTH        1
#define SENSOR_POLL_PERIOD_MS      100
#define PIR_ACTIVE_LEVEL           1
#define PIR_DEBOUNCE_MS             30
#define PIR_COOLDOWN_MS             2000
#define ALARM_HALF_PERIOD_MS        250
#define ALARM_CYCLE_COUNT           8
#define ALARM_GPIO_RETRY_MS         1000
#define APP_START_BIT              BIT0
#define VISION_READY_BIT           BIT1
#define WIFI_CONNECTED_BIT         BIT0
#define WIFI_DISCONNECTED_BIT      BIT1
#define WIFI_STARTED_BIT           BIT2
#define MQTT_CONNECTED_BIT         BIT3
#define MQTT_ALERT_QUEUE_LENGTH     8

static constexpr float FACE_ALARM_SCORE_THRESHOLD = 0.7F;
static constexpr float MSR_SCORE_THRESHOLD = 0.1F;
static constexpr float MSR_NMS_THRESHOLD = 0.5F;
static constexpr int MSR_TOP_K = 10;
static constexpr float MSR_RESIZE_SCALE = 0.2F;
static constexpr float MNP_SCORE_THRESHOLD = 0.5F;
static constexpr float MNP_NMS_THRESHOLD = 0.3F;
static constexpr int MNP_TOP_K = 5;

static const char *TAG = "security_main";

typedef enum {
    SIGNAL_PIR_TRIGGERED = 1,
    SIGNAL_HUMAN_CONFIRMED,
} app_signal_t;

static QueueHandle_t s_vision_queue;
static QueueHandle_t s_alarm_queue;
static QueueHandle_t s_mqtt_queue;
static EventGroupHandle_t s_start_event;
static EventGroupHandle_t s_wifi_event_group;
static TaskHandle_t s_task_handles[4];
static TaskHandle_t s_network_task;
static esp_event_handler_instance_t s_wifi_any_id_instance;
static esp_event_handler_instance_t s_wifi_got_ip_instance;
static esp_mqtt_client_handle_t mqtt_client = nullptr;
static esp_pm_lock_handle_t s_vision_cpu_lock;
static esp_pm_lock_handle_t s_camera_awake_lock;

static esp_err_t init_camera(void);
static bool mqtt_app_init(void);

struct mqtt_alert_t {
    int64_t timestamp_us;
};

/* These locks are only used by app_main before startup, then by vision_task. */
class VisionPowerGuard {
public:
    VisionPowerGuard() noexcept
    {
#if CONFIG_PM_ENABLE
        ESP_ERROR_CHECK(esp_pm_lock_acquire(s_camera_awake_lock));
        ESP_ERROR_CHECK(esp_pm_lock_acquire(s_vision_cpu_lock));
#endif
    }
    ~VisionPowerGuard()
    {
#if CONFIG_PM_ENABLE
        ESP_ERROR_CHECK(esp_pm_lock_release(s_vision_cpu_lock));
        ESP_ERROR_CHECK(esp_pm_lock_release(s_camera_awake_lock));
#endif
    }
    VisionPowerGuard(const VisionPowerGuard &) = delete;
    VisionPowerGuard &operator=(const VisionPowerGuard &) = delete;
};

/* Declare after VisionPowerGuard, but before CameraFrameGuard, in each scope. */
class CameraSessionGuard {
public:
    CameraSessionGuard() noexcept = default;
    esp_err_t start()
    {
        const esp_err_t err = init_camera();
        initialized_ = (err == ESP_OK);
        return err;
    }
    ~CameraSessionGuard()
    {
        if (initialized_) {
            /* Stop DMA/VSYNC before permitting sleep; the frame is returned first. */
            ESP_ERROR_CHECK(esp_camera_deinit());
        }
    }
    CameraSessionGuard(const CameraSessionGuard &) = delete;
    CameraSessionGuard &operator=(const CameraSessionGuard &) = delete;

private:
    bool initialized_ = false;
};

/* Driver owns both the frame descriptor and its pixel buffer. */
class CameraFrameGuard {
public:
    explicit CameraFrameGuard(camera_fb_t *frame) noexcept : frame_(frame) {}
    ~CameraFrameGuard()
    {
        if (frame_ != nullptr) {
            esp_camera_fb_return(frame_);
        }
    }
    camera_fb_t *get() const noexcept { return frame_; }
    CameraFrameGuard(const CameraFrameGuard &) = delete;
    CameraFrameGuard &operator=(const CameraFrameGuard &) = delete;

private:
    camera_fb_t *frame_;
};

static void wait_for_startup(void)
{
    /* No task may allocate a model or acquire a frame before startup succeeds. */
    xEventGroupWaitBits(s_start_event, APP_START_BIT, pdFALSE, pdTRUE,
                       portMAX_DELAY);
}

static void sensor_task(void *arg)
{
    (void)arg;
    wait_for_startup();
    xEventGroupWaitBits(s_start_event, VISION_READY_BIT, pdFALSE, pdTRUE,
                       portMAX_DELAY);
    ESP_LOGI(TAG, "Waiting %d ms for PIR stabilization", PIR_WARMUP_MS);
    vTaskDelay(pdMS_TO_TICKS(PIR_WARMUP_MS));
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

static void vision_task(void *arg)
{
    (void)arg;
    wait_for_startup();
    ESP_LOGI(TAG, "Vision task initializing classic ESP-DL face detectors");

    try {
        /* Construct once, reuse for every trigger, and keep in this task only. */
        HumanFaceDetectMSR01 stage1(MSR_SCORE_THRESHOLD, MSR_NMS_THRESHOLD,
                                   MSR_TOP_K, MSR_RESIZE_SCALE);
        HumanFaceDetectMNP01 stage2(MNP_SCORE_THRESHOLD, MNP_NMS_THRESHOLD,
                                   MNP_TOP_K);
        ESP_LOGI(TAG, "Face detectors ready; alarm score threshold > %.2f",
                 static_cast<double>(FACE_ALARM_SCORE_THRESHOLD));
        xEventGroupSetBits(s_start_event, VISION_READY_BIT);

        app_signal_t signal;
        for (;;) {
            if (xQueueReceive(s_vision_queue, &signal, portMAX_DELAY) != pdTRUE ||
                signal != SIGNAL_PIR_TRIGGERED) {
                continue;
            }

            size_t confirmed_faces = 0;
            try {
                /* Lock across initialization, capture, inference and deinit. */
                VisionPowerGuard power;
                CameraSessionGuard camera;
                const esp_err_t camera_err = camera.start();
                if (camera_err != ESP_OK) {
                    ESP_LOGE(TAG, "Camera initialization failed: %s",
                             esp_err_to_name(camera_err));
                    continue;
                }
                /* Avoid inferring the first unsettled auto-exposure frames. */
                bool warmup_ok = true;
                for (int i = 0; i < CAMERA_WARMUP_FRAMES; ++i) {
                    CameraFrameGuard discarded(esp_camera_fb_get());
                    if (discarded.get() == nullptr) {
                        warmup_ok = false;
                        break;
                    }
                }
                if (!warmup_ok) {
                    ESP_LOGE(TAG, "Camera warmup capture failed");
                    continue;
                }
                CameraFrameGuard frame(esp_camera_fb_get());
                camera_fb_t *fb = frame.get();
                if (fb == nullptr) {
                    ESP_LOGE(TAG, "Camera capture failed: no frame buffer");
                    continue;
                }
                if (fb->buf == nullptr || fb->format != PIXFORMAT_RGB565 ||
                    fb->width != 320 || fb->height != 240 ||
                    fb->len != fb->width * fb->height * sizeof(uint16_t) ||
                    reinterpret_cast<uintptr_t>(fb->buf) % alignof(uint16_t) != 0) {
                    ESP_LOGE(TAG, "Invalid QVGA RGB565 frame: len=%zu, "
                                  "width=%zu, height=%zu, format=%d",
                             fb->len, fb->width, fb->height,
                             static_cast<int>(fb->format));
                    continue;  /* The guard returns any acquired frame. */
                }
                ESP_LOGI(TAG, "Frame captured: len=%zu bytes, width=%zu, "
                              "height=%zu, format=RGB565",
                         fb->len, fb->width, fb->height);

                auto *pixels = reinterpret_cast<uint16_t *>(fb->buf);
                const int64_t start_us = esp_timer_get_time();
                /* API shape is {H, W, 3}, even for packed uint16_t RGB565. */
                const std::vector<int> shape{
                    static_cast<int>(fb->height), static_cast<int>(fb->width), 3};
                std::list<dl::detect::result_t> &candidates =
                    stage1.infer(pixels, shape);
                std::list<dl::detect::result_t> &results =
                    candidates.empty() ? candidates
                                       : stage2.infer(pixels, shape, candidates);
                const int64_t elapsed_us = esp_timer_get_time() - start_us;

                /* Result lists are owned by the detectors; read before next infer. */
                for (const dl::detect::result_t &result : results) {
                    if (!std::isfinite(result.score) ||
                        result.score <= FACE_ALARM_SCORE_THRESHOLD) {
                        continue;
                    }
                    if (result.box.size() < 4) {
                        ESP_LOGW(TAG, "Ignoring detection with incomplete box");
                        continue;
                    }
                    const int x1 = result.box[0];
                    const int y1 = result.box[1];
                    const int x2 = result.box[2];
                    const int y2 = result.box[3];
                    if (!security::valid_detection_box(
                            x1, y1, x2, y2, static_cast<int>(fb->width),
                            static_cast<int>(fb->height))) {
                        ESP_LOGW(TAG, "Ignoring out-of-range box [%d,%d,%d,%d]",
                                 x1, y1, x2, y2);
                        continue;
                    }
                    ++confirmed_faces;
                    ESP_LOGI(TAG, "Face %zu: score=%.3f, box=[%d,%d,%d,%d]",
                             confirmed_faces, static_cast<double>(result.score),
                             x1, y1, x2, y2);
                }

                ESP_LOGI(TAG, "Face inference: %" PRId64 " us (%.2f ms), "
                              "candidates=%zu, results=%zu, confirmed=%zu",
                         elapsed_us, static_cast<double>(elapsed_us) / 1000.0,
                         candidates.size(), results.size(), confirmed_faces);
                ESP_LOGI(TAG, "SRAM free=%u largest=%u; PSRAM free=%u largest=%u; vision stack=%u",
                         static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
                         static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
                         static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
                         static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM)),
                         static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
            } catch (const std::bad_alloc &) {
                /* Frame, camera session and PM locks have already unwound. */
                ESP_LOGE(TAG, "Insufficient memory during face inference");
                continue;
            } catch (const std::exception &error) {
                ESP_LOGE(TAG, "Face inference failed: %s", error.what());
                continue;
            } catch (...) {
                ESP_LOGE(TAG, "Face inference failed with an unknown exception");
                continue;
            }

            /* Frame returned, camera deinitialized and PM locks released. */
            if (confirmed_faces > 0) {
                const app_signal_t alarm_signal = SIGNAL_HUMAN_CONFIRMED;
                if (xQueueOverwrite(s_alarm_queue, &alarm_signal) != pdPASS) {
                    ESP_LOGE(TAG, "Failed to enqueue face-confirmed alarm");
                }
            }
        }
    } catch (const std::bad_alloc &) {
        ESP_LOGE(TAG, "Insufficient memory while constructing face detectors");
    } catch (const std::exception &error) {
        ESP_LOGE(TAG, "Face detector initialization failed: %s", error.what());
    } catch (...) {
        ESP_LOGE(TAG, "Face detector initialization failed with unknown exception");
    }

    /* C++ model objects have left scope before recovery restarts the device. */
    ESP_LOGE(TAG, "Vision initialization failed; restarting to recover");
    vTaskDelay(pdMS_TO_TICKS(CAMERA_RESTART_DELAY_MS));
    esp_restart();
    vTaskDelete(nullptr);
}

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

static bool publish_mqtt_alert(int64_t timestamp_us)
{
    cJSON *root = cJSON_CreateObject();
    if (root == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate MQTT alert JSON object");
        return false;
    }

    char *payload = nullptr;
    bool queued = false;
    const bool json_ok =
        cJSON_AddStringToObject(root, "device", "ESP32-S3") != nullptr &&
        cJSON_AddStringToObject(root, "alert", "Human Detected") != nullptr &&
        cJSON_AddNumberToObject(root, "timestamp",
                                static_cast<double>(timestamp_us)) != nullptr;
    if (json_ok) {
        payload = cJSON_PrintUnformatted(root);
    }

    if (payload == nullptr) {
        ESP_LOGE(TAG, "Failed to build MQTT alert JSON payload");
    } else if (mqtt_client == nullptr) {
        ESP_LOGW(TAG, "MQTT client is unavailable; alert was not published");
    } else if (!security::outbox_has_room(
                   esp_mqtt_client_get_outbox_size(mqtt_client),
                   static_cast<int>(std::strlen(payload) + std::strlen(MQTT_TOPIC) + 16))) {
        ESP_LOGW(TAG, "MQTT outbox full; retaining alert in bounded worker queue");
    } else {
        /* Copies payload; never performs socket I/O in this application task. */
        const int message_id = esp_mqtt_client_enqueue(
            mqtt_client, MQTT_TOPIC, payload, 0, 1, 0, true);
        if (message_id < 0) {
            ESP_LOGE(TAG, "Failed to enqueue MQTT alert");
        } else {
            queued = true;
            ESP_LOGI(TAG, "MQTT alert queued: message_id=%d, payload=%s",
                     message_id, payload);
        }
    }

    /* cJSON_PrintUnformatted() uses malloc with the default cJSON hooks. */
    free(payload);
    cJSON_Delete(root);
    return queued;
}

static void mqtt_alert_task(void *arg)
{
    (void)arg;
    wait_for_startup();
    mqtt_alert_t alert;
    for (;;) {
        if (xQueueReceive(s_mqtt_queue, &alert, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        for (;;) {
            if (security::alert_expired(alert.timestamp_us, esp_timer_get_time())) {
                ESP_LOGW(TAG, "Dropping expired MQTT alert (older than 5 min)");
                break;
            }
            const EventBits_t bits = xEventGroupWaitBits(
                s_wifi_event_group, MQTT_CONNECTED_BIT, pdFALSE, pdTRUE,
                pdMS_TO_TICKS(1000));
            if ((bits & MQTT_CONNECTED_BIT) != 0) {
                if (publish_mqtt_alert(alert.timestamp_us)) {
                    break;
                }
                /* Rate limit retries on allocation/outbox errors. */
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
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
        const mqtt_alert_t alert{esp_timer_get_time()};
        if (s_mqtt_queue != nullptr &&
            xQueueSend(s_mqtt_queue, &alert, 0) != pdPASS) {
            ESP_LOGW(TAG, "MQTT application queue full; dropping network alert");
        }
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

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_STARTED_BIT);
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT);
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT);
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const auto *event = static_cast<ip_event_got_ip_t *>(event_data);
        ESP_LOGI(TAG, "WiFi connected, IP=" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupClearBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void network_task(void *arg)
{
    (void)arg;
    xEventGroupWaitBits(s_wifi_event_group, WIFI_STARTED_BIT, pdFALSE, pdTRUE,
                       portMAX_DELAY);
    security::RetryBackoff backoff(MAX_RETRY);
    bool mqtt_started = false;
    for (;;) {
        if ((xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) != 0) {
            backoff.reset();
            if (!mqtt_started && mqtt_client != nullptr) {
                const esp_err_t err = esp_mqtt_client_start(mqtt_client);
                mqtt_started = (err == ESP_OK);
                if (!mqtt_started) {
                    ESP_LOGE(TAG, "MQTT start failed: %s; will retry",
                             esp_err_to_name(err));
                }
            }
            /* Timeout permits another MQTT-start attempt without busy polling. */
            xEventGroupWaitBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT,
                                pdTRUE, pdTRUE, pdMS_TO_TICKS(1000));
            continue;
        }

        xEventGroupClearBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT);
        const esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "WiFi connect request failed: %s", esp_err_to_name(err));
        }
        const EventBits_t bits = xEventGroupWaitBits(
            s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_DISCONNECTED_BIT,
            pdFALSE, pdFALSE, pdMS_TO_TICKS(WIFI_ATTEMPT_TIMEOUT_MS));
        if ((bits & WIFI_CONNECTED_BIT) != 0) {
            continue;
        }
        /* Cancel an association/DHCP attempt that otherwise may hang forever. */
        esp_wifi_disconnect();
        const uint32_t delay_ms = backoff.next_delay_ms();
        ESP_LOGW(TAG, "WiFi offline; retry in %u ms (local alarm remains active)",
                 static_cast<unsigned>(delay_ms));
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

[[maybe_unused]] static bool wifi_init_sta(void)
{
    static_assert(sizeof(WIFI_SSID) - 1 <= 32, "Wi-Fi SSID must be at most 32 bytes");
    static_assert(sizeof(WIFI_PASS) - 1 <= 64, "Wi-Fi password must be at most 64 bytes");
    static_assert(sizeof(MQTT_TOPIC) < 256, "MQTT topic is too long for bounded outbox");
    if (std::strcmp(WIFI_SSID, "YOUR_WIFI_SSID") == 0 || std::strlen(WIFI_SSID) == 0) {
        ESP_LOGW(TAG, "WiFi credentials not configured; continuing offline");
        return false;
    }
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == nullptr) {
        ESP_LOGE(TAG, "Failed to create WiFi event group");
        return false;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    if (esp_netif_create_default_wifi_sta() == nullptr) {
        ESP_LOGE(TAG, "Failed to create default WiFi station interface");
        return false;
    }

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr,
        &s_wifi_any_id_instance));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr,
        &s_wifi_got_ip_instance));

    wifi_config_t wifi_config{};
    std::memcpy(wifi_config.sta.ssid, WIFI_SSID, sizeof(WIFI_SSID) - 1);
    std::memcpy(wifi_config.sta.password, WIFI_PASS, sizeof(WIFI_PASS) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    /* Publish the handle once, before any application/network task can use it. */
    mqtt_app_init();
    if (xTaskCreate(network_task, "network_task", NETWORK_TASK_STACK_SIZE,
                    nullptr, 3, &s_network_task) != pdPASS) {
        ESP_LOGE(TAG, "Cannot create network supervisor; continuing offline");
        return false;
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    /* WiFi manages its own sleep/wakeup schedule while staying associated. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));

    ESP_LOGI(TAG, "Waiting for WiFi SSID '%s'", WIFI_SSID);
    const EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE,
        pdTRUE, pdMS_TO_TICKS(WIFI_STARTUP_TIMEOUT_MS));
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;
    const auto *event = static_cast<esp_mqtt_event_handle_t>(event_data);

    switch (static_cast<esp_mqtt_event_id_t>(event_id)) {
    case MQTT_EVENT_CONNECTED:
        xEventGroupSetBits(s_wifi_event_group, MQTT_CONNECTED_BIT);
        ESP_LOGI(TAG, "MQTT connected to %s", MQTT_BROKER_URI);
        break;
    case MQTT_EVENT_DISCONNECTED:
        xEventGroupClearBits(s_wifi_event_group, MQTT_CONNECTED_BIT);
        ESP_LOGW(TAG, "MQTT disconnected; client will reconnect automatically");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT QoS message acknowledged: message_id=%d",
                 event->msg_id);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT transport error");
        break;
    case MQTT_EVENT_DELETED:
        ESP_LOGW(TAG, "MQTT unacknowledged message expired: message_id=%d",
                 event->msg_id);
        break;
    default:
        break;
    }
}

static bool mqtt_app_init(void)
{
    esp_mqtt_client_config_t mqtt_config{};
    mqtt_config.broker.address.uri = MQTT_BROKER_URI;

    mqtt_client = esp_mqtt_client_init(&mqtt_config);
    if (mqtt_client == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return false;
    }

    const esp_err_t err = esp_mqtt_client_register_event(
        mqtt_client, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
        &mqtt_event_handler, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT event registration failed: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = nullptr;
        return false;
    }
    return true;
}

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS requires erase before initialization");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void init_power_management(void)
{
#if CONFIG_PM_ENABLE
    /* ESP-IDF 5.0.x uses the chip-specific PM configuration type. */
    esp_pm_config_esp32s3_t pm_config{};
    pm_config.max_freq_mhz = PM_MAX_CPU_FREQ_MHZ;
    pm_config.min_freq_mhz = PM_MIN_CPU_FREQ_MHZ;
#if CONFIG_SECURITY_AUTO_LIGHT_SLEEP
    pm_config.light_sleep_enable = true;
#endif
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    ESP_ERROR_CHECK(esp_pm_lock_create(
        ESP_PM_CPU_FREQ_MAX, 0, "vision_cpu", &s_vision_cpu_lock));
    ESP_ERROR_CHECK(esp_pm_lock_create(
        ESP_PM_NO_LIGHT_SLEEP, 0, "camera_awake", &s_camera_awake_lock));
    ESP_LOGI(TAG, "Power management ready: CPU %d-%d MHz, automatic light sleep=%d",
             PM_MIN_CPU_FREQ_MHZ, PM_MAX_CPU_FREQ_MHZ,
             static_cast<int>(pm_config.light_sleep_enable));
#else
    ESP_LOGI(TAG, "Power management disabled; fixed CPU frequency");
#endif
}

static void validate_hardware_config(void)
{
    struct Pin { const char *name; int number; bool output; };
    const Pin pins[] = {
        {"PIR", PIR_GPIO, false}, {"buzzer", BUZZER_GPIO, true},
        {"LED", LED_GPIO, true}, {"XCLK", CAM_PIN_XCLK, true},
        {"SDA", CAM_PIN_SIOD, true}, {"SCL", CAM_PIN_SIOC, true},
        {"D7", CAM_PIN_Y9, false}, {"D6", CAM_PIN_Y8, false},
        {"D5", CAM_PIN_Y7, false}, {"D4", CAM_PIN_Y6, false},
        {"D3", CAM_PIN_Y5, false}, {"D2", CAM_PIN_Y4, false},
        {"D1", CAM_PIN_Y3, false}, {"D0", CAM_PIN_Y2, false},
        {"VSYNC", CAM_PIN_VSYNC, false}, {"HREF", CAM_PIN_HREF, false},
        {"PCLK", CAM_PIN_PCLK, false}, {"PWDN", CAM_PIN_PWDN, true},
        {"RESET", CAM_PIN_RESET, true},
    };
    uint64_t used = 0;
    for (const auto &pin : pins) {
        if (pin.number == -1) { continue; }
        bool valid = GPIO_IS_VALID_GPIO(pin.number) &&
                     (!pin.output || GPIO_IS_VALID_OUTPUT_GPIO(pin.number));
        /* SPI0/1 pins are reserved by module Flash/PSRAM wiring. */
        valid = valid && !(pin.number >= 26 && pin.number <= 32);
#if CONFIG_SPIRAM_MODE_OCT || CONFIG_ESPTOOLPY_FLASHMODE_OPI
        valid = valid && !(pin.number >= 33 && pin.number <= 37);
#endif
        if (!valid || (used & (UINT64_C(1) << pin.number)) != 0) {
            ESP_LOGE(TAG, "Invalid/reserved/duplicate GPIO: %s=%d; check menuconfig",
                     pin.name, pin.number);
            ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
        }
        used |= UINT64_C(1) << pin.number;
        if (pin.number == 0 || pin.number == 3 || pin.number == 45 || pin.number == 46 ||
            pin.number == 19 || pin.number == 20) {
            ESP_LOGW(TAG, "%s uses strapping/USB GPIO%d; verify board wiring", pin.name, pin.number);
        }
    }
    if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) == 0) {
        ESP_LOGE(TAG, "PSRAM unavailable; verify module type and PSRAM menuconfig");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
}

static void init_gpio(void)
{
    gpio_config_t pir_config{};
    pir_config.pin_bit_mask = UINT64_C(1) << PIR_GPIO;
    pir_config.mode = GPIO_MODE_INPUT;
    pir_config.pull_up_en = GPIO_PULLUP_DISABLE;
    pir_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    pir_config.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&pir_config));

    gpio_config_t alarm_config{};
    alarm_config.pin_bit_mask = (UINT64_C(1) << BUZZER_GPIO) |
                               (UINT64_C(1) << LED_GPIO);
    alarm_config.mode = GPIO_MODE_OUTPUT;
    alarm_config.pull_up_en = GPIO_PULLUP_DISABLE;
    alarm_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    alarm_config.intr_type = GPIO_INTR_DISABLE;
    /* Preload inactive latches before enabling outputs (active-low modules too). */
    ESP_ERROR_CHECK(gpio_set_level(BUZZER_GPIO, BUZZER_INACTIVE_LEVEL));
    ESP_ERROR_CHECK(gpio_set_level(LED_GPIO, LED_INACTIVE_LEVEL));
    ESP_ERROR_CHECK(gpio_config(&alarm_config));
    ESP_ERROR_CHECK(gpio_set_level(BUZZER_GPIO, BUZZER_INACTIVE_LEVEL));
    ESP_ERROR_CHECK(gpio_set_level(LED_GPIO, LED_INACTIVE_LEVEL));
    /* IDF may otherwise tri-state outputs during each tickless sleep period. */
    ESP_ERROR_CHECK(gpio_sleep_sel_dis(BUZZER_GPIO));
    ESP_ERROR_CHECK(gpio_sleep_sel_dis(LED_GPIO));
}

static esp_err_t init_camera(void)
{
    /* Zero initialization + assignments also work in standard C++17. */
    camera_config_t camera_config{};
    camera_config.pin_pwdn = CAM_PIN_PWDN;
    camera_config.pin_reset = CAM_PIN_RESET;
    camera_config.pin_xclk = CAM_PIN_XCLK;
    camera_config.pin_sccb_sda = CAM_PIN_SIOD;
    camera_config.pin_sccb_scl = CAM_PIN_SIOC;

    /* OV2640 Y2..Y9 correspond to the driver's D0..D7. */
    camera_config.pin_d7 = CAM_PIN_Y9;
    camera_config.pin_d6 = CAM_PIN_Y8;
    camera_config.pin_d5 = CAM_PIN_Y7;
    camera_config.pin_d4 = CAM_PIN_Y6;
    camera_config.pin_d3 = CAM_PIN_Y5;
    camera_config.pin_d2 = CAM_PIN_Y4;
    camera_config.pin_d1 = CAM_PIN_Y3;
    camera_config.pin_d0 = CAM_PIN_Y2;
    camera_config.pin_vsync = CAM_PIN_VSYNC;
    camera_config.pin_href = CAM_PIN_HREF;
    camera_config.pin_pclk = CAM_PIN_PCLK;

    camera_config.xclk_freq_hz = CAMERA_XCLK_FREQ_HZ;
    /* Compatibility fields; the ESP32-S3 driver generates XCLK via LCD_CAM. */
    camera_config.ledc_timer = LEDC_TIMER_0;
    camera_config.ledc_channel = LEDC_CHANNEL_0;
    camera_config.pixel_format = PIXFORMAT_RGB565;
    camera_config.frame_size = FRAMESIZE_QVGA;
    /* JPEG quality does not affect RGB565; retained as requested. */
    camera_config.jpeg_quality = 12;
    camera_config.fb_count = 1;
    /* Enable the actual board's PSRAM configuration in menuconfig. */
    camera_config.fb_location = CAMERA_FB_IN_PSRAM;
    camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&camera_config);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Camera initialized: RGB565, QVGA 320x240, "
                      "1 frame buffer in PSRAM");
    }
    return err;
}

/* Called only before APP_START_BIT is set; task-local models do not exist yet. */
static void cleanup_startup_resources(void)
{
    for (size_t i = 0; i < sizeof(s_task_handles) / sizeof(s_task_handles[0]); ++i) {
        if (s_task_handles[i] != NULL) {
            vTaskDelete(s_task_handles[i]);
            s_task_handles[i] = NULL;
        }
    }

    if (s_vision_queue != NULL) {
        vQueueDelete(s_vision_queue);
        s_vision_queue = NULL;
    }
    if (s_alarm_queue != NULL) {
        vQueueDelete(s_alarm_queue);
        s_alarm_queue = NULL;
    }
    if (s_mqtt_queue != nullptr) {
        vQueueDelete(s_mqtt_queue);
        s_mqtt_queue = nullptr;
    }

    if (s_start_event != nullptr) {
        vEventGroupDelete(s_start_event);
        s_start_event = nullptr;
    }

    /* Camera startup validation is already closed; tasks cannot hold locks yet. */
    if (s_vision_cpu_lock != nullptr) {
        ESP_ERROR_CHECK(esp_pm_lock_delete(s_vision_cpu_lock));
        s_vision_cpu_lock = nullptr;
    }
    if (s_camera_awake_lock != nullptr) {
        ESP_ERROR_CHECK(esp_pm_lock_delete(s_camera_awake_lock));
        s_camera_awake_lock = nullptr;
    }
}

extern "C" void app_main(void)
{
    validate_hardware_config();
    init_power_management();
    init_nvs();
    init_gpio();

    /* Wait for IP first, but a prolonged network outage must not disable alarms. */
#if CONFIG_SECURITY_NETWORK_ENABLED
    if (!wifi_init_sta()) {
        ESP_LOGW(TAG, "No startup IP; enabling local protection offline");
    }
#else
    ESP_LOGI(TAG, "Offline hardware mode: WiFi/MQTT disabled");
#endif

    /* Validate the camera under locks, then leave it deinitialized during idle. */
    esp_err_t err;
    {
        VisionPowerGuard power;
        CameraSessionGuard camera;
        err = camera.start();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s (0x%x); "
                      "restarting in %d ms",
                 esp_err_to_name(err), (unsigned int)err,
                 CAMERA_RESTART_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(CAMERA_RESTART_DELAY_MS));
        esp_restart();
        return;
    }
    ESP_LOGI(TAG, "Camera ready for on-demand capture; idle sleep is permitted");

    s_vision_queue = xQueueCreate(SIGNAL_QUEUE_LENGTH, sizeof(app_signal_t));
    s_alarm_queue = xQueueCreate(SIGNAL_QUEUE_LENGTH, sizeof(app_signal_t));
    if (mqtt_client != nullptr && s_network_task != nullptr) {
        s_mqtt_queue = xQueueCreate(MQTT_ALERT_QUEUE_LENGTH, sizeof(mqtt_alert_t));
        if (s_mqtt_queue == nullptr) {
            ESP_LOGW(TAG, "Cannot allocate MQTT queue; local protection only");
        }
    }
    s_start_event = xEventGroupCreate();
    if (s_vision_queue == NULL || s_alarm_queue == NULL || s_start_event == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate queues/startup event group");
        goto startup_failed;
    }

    if (xTaskCreate(sensor_task, "sensor_task", SENSOR_TASK_STACK_SIZE,
                    NULL, SENSOR_TASK_PRIORITY, &s_task_handles[0]) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        goto startup_failed;
    }
    if (xTaskCreate(vision_task, "vision_task", VISION_TASK_STACK_SIZE,
                    NULL, VISION_TASK_PRIORITY, &s_task_handles[1]) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create vision task");
        goto startup_failed;
    }
    if (xTaskCreate(alarm_task, "alarm_task", ALARM_TASK_STACK_SIZE,
                    NULL, ALARM_TASK_PRIORITY, &s_task_handles[2]) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create alarm task");
        goto startup_failed;
    }
    if (s_mqtt_queue != nullptr &&
        xTaskCreate(mqtt_alert_task, "mqtt_alert", MQTT_TASK_STACK_SIZE,
                    nullptr, 2, &s_task_handles[3]) != pdPASS) {
        ESP_LOGW(TAG, "Cannot create MQTT worker; local protection only");
        vQueueDelete(s_mqtt_queue);
        s_mqtt_queue = nullptr;
    }

    /* Past this point cleanup_startup_resources() must never be called. */
    xEventGroupSetBits(s_start_event, APP_START_BIT);
    ESP_LOGI(TAG, "Framework ready: sensor -> face vision -> alarm + MQTT");
#if CONFIG_SECURITY_CAPTURE_ON_BOOT
    /* Optional development trigger using the same path as a real PIR event. */
    {
        const app_signal_t test_signal = SIGNAL_PIR_TRIGGERED;
        if (xQueueOverwrite(s_vision_queue, &test_signal) != pdPASS) {
            ESP_LOGE(TAG, "Failed to enqueue boot-time test trigger");
        }
    }
#endif
    return;

startup_failed:
    cleanup_startup_resources();
    ESP_LOGE(TAG, "Local task startup failed; restarting to recover");
    vTaskDelay(pdMS_TO_TICKS(CAMERA_RESTART_DELAY_MS));
    esp_restart();
}
