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

/* Classic ESP-DL v2.0 APIs, used with ESP-IDF 5.0.x. */
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"

#if !CONFIG_COMPILER_CXX_EXCEPTIONS
#error "Enable CONFIG_COMPILER_CXX_EXCEPTIONS for model/STL allocation handling"
#endif

#if !CONFIG_PM_ENABLE || !CONFIG_FREERTOS_USE_TICKLESS_IDLE
#error "Enable CONFIG_PM_ENABLE and CONFIG_FREERTOS_USE_TICKLESS_IDLE"
#endif

/* Example pins: confirm board availability and OV2640 pin assignments first. */
#define PIR_GPIO                    GPIO_NUM_1
#define BUZZER_GPIO                 GPIO_NUM_2
#define LED_GPIO                    GPIO_NUM_21

/*
 * Provisional OV2640 wiring supplied for this project.
 * ESP32-S3-WROOM does not define a universal camera pin map.
 * Verify every pin against the actual board schematic and camera wiring.
 * -1 means PWDN/RESET is not controlled by a GPIO.
 */
#define CAM_PIN_PWDN                (-1)
#define CAM_PIN_RESET               (-1)
#define CAM_PIN_XCLK                10
#define CAM_PIN_SIOD                40
#define CAM_PIN_SIOC                39
#define CAM_PIN_Y9                  48
#define CAM_PIN_Y8                  11
#define CAM_PIN_Y7                  12
#define CAM_PIN_Y6                  14
#define CAM_PIN_Y5                  16
#define CAM_PIN_Y4                  18
#define CAM_PIN_Y3                  17
#define CAM_PIN_Y2                  15
#define CAM_PIN_VSYNC               38
#define CAM_PIN_HREF                47
#define CAM_PIN_PCLK                13

#define CAMERA_XCLK_FREQ_HZ         20000000
#define CAMERA_RESTART_DELAY_MS     1000

/* Replace these placeholders before flashing the board. */
#define WIFI_SSID                   "YOUR_WIFI_SSID"
#define WIFI_PASS                   "YOUR_WIFI_PASSWORD"
#define MQTT_BROKER_URI             "mqtt://broker.emqx.io"
#define MQTT_TOPIC                  "esp32s3/security/alert"
#define MAX_RETRY                   5
#define NETWORK_RESTART_DELAY_MS    3000
#define PM_MAX_CPU_FREQ_MHZ         240
#define PM_MIN_CPU_FREQ_MHZ         40

/* Assumes active-high alarm outputs; adjust for the selected modules. */
#define BUZZER_ACTIVE_LEVEL         1
#define BUZZER_INACTIVE_LEVEL       0
#define LED_ACTIVE_LEVEL            1
#define LED_INACTIVE_LEVEL          0

/* ESP-IDF xTaskCreate() stack sizes are specified in bytes. */
#define SENSOR_TASK_STACK_SIZE     2048
#define VISION_TASK_STACK_SIZE     8192
#define ALARM_TASK_STACK_SIZE      4096

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
#define WIFI_CONNECTED_BIT         BIT0
#define WIFI_FAIL_BIT              BIT1

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
static EventGroupHandle_t s_start_event;
static EventGroupHandle_t s_wifi_event_group;
static TaskHandle_t s_task_handles[3];
static int s_wifi_retry_count;
static esp_event_handler_instance_t s_wifi_any_id_instance;
static esp_event_handler_instance_t s_wifi_got_ip_instance;
static esp_mqtt_client_handle_t mqtt_client = nullptr;
static esp_pm_lock_handle_t s_vision_cpu_lock;
static esp_pm_lock_handle_t s_camera_awake_lock;

static esp_err_t init_camera(void);

/* These locks are only used by app_main before startup, then by vision_task. */
class VisionPowerGuard {
public:
    VisionPowerGuard() noexcept
    {
        ESP_ERROR_CHECK(esp_pm_lock_acquire(s_camera_awake_lock));
        ESP_ERROR_CHECK(esp_pm_lock_acquire(s_vision_cpu_lock));
    }
    ~VisionPowerGuard()
    {
        ESP_ERROR_CHECK(esp_pm_lock_release(s_vision_cpu_lock));
        ESP_ERROR_CHECK(esp_pm_lock_release(s_camera_awake_lock));
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
                    if (x1 < 0 || y1 < 0 || x2 < x1 || y2 < y1 ||
                        x2 > static_cast<int>(fb->width) ||
                        y2 > static_cast<int>(fb->height)) {
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

    /* C++ objects have left scope before this FreeRTOS task deletes itself. */
    ESP_LOGE(TAG, "Vision task stopped; face detection is unavailable");
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

static void publish_mqtt_alert(void)
{
    cJSON *root = cJSON_CreateObject();
    if (root == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate MQTT alert JSON object");
        return;
    }

    char *payload = nullptr;
    const int64_t timestamp_us = esp_timer_get_time();
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
    } else {
        const int message_id = esp_mqtt_client_publish(
            mqtt_client, MQTT_TOPIC, payload, 0, 1, 0);
        if (message_id < 0) {
            ESP_LOGE(TAG, "Failed to enqueue MQTT alert");
        } else {
            ESP_LOGI(TAG, "MQTT alert queued: message_id=%d, payload=%s",
                     message_id, payload);
        }
    }

    /* cJSON_PrintUnformatted() uses malloc with the default cJSON hooks. */
    free(payload);
    cJSON_Delete(root);
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
        publish_mqtt_alert();

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

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        const esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Initial WiFi connect request failed: %s",
                     esp_err_to_name(err));
        }
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        if (s_wifi_retry_count < MAX_RETRY) {
            ++s_wifi_retry_count;
            ESP_LOGW(TAG, "WiFi disconnected; reconnecting (%d/%d)",
                     s_wifi_retry_count, MAX_RETRY);
            const esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "WiFi reconnect request failed: %s",
                         esp_err_to_name(err));
            }
        } else {
            ESP_LOGE(TAG, "WiFi connection failed after %d retries", MAX_RETRY);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const auto *event = static_cast<ip_event_got_ip_t *>(event_data);
        ESP_LOGI(TAG, "WiFi connected, IP=" IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retry_count = 0;
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_init_sta(void)
{
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
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr,
        &s_wifi_got_ip_instance));

    wifi_config_t wifi_config{};
    std::strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), WIFI_SSID,
                 sizeof(wifi_config.sta.ssid) - 1);
    std::strncpy(reinterpret_cast<char *>(wifi_config.sta.password), WIFI_PASS,
                 sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* WiFi manages its own sleep/wakeup schedule while staying associated. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));

    ESP_LOGI(TAG, "Waiting for WiFi SSID '%s'", WIFI_SSID);
    const EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE,
        pdFALSE, portMAX_DELAY);
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
        ESP_LOGI(TAG, "MQTT connected to %s", MQTT_BROKER_URI);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected; client will reconnect automatically");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT QoS message acknowledged: message_id=%d",
                 event->msg_id);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT transport error");
        break;
    default:
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_config{};
    mqtt_config.broker.address.uri = MQTT_BROKER_URI;

    mqtt_client = esp_mqtt_client_init(&mqtt_config);
    if (mqtt_client == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        mqtt_client, ESP_EVENT_ANY_ID, &mqtt_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
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
    /* ESP-IDF 5.0.x uses the chip-specific PM configuration type. */
    esp_pm_config_esp32s3_t pm_config{};
    pm_config.max_freq_mhz = PM_MAX_CPU_FREQ_MHZ;
    pm_config.min_freq_mhz = PM_MIN_CPU_FREQ_MHZ;
    pm_config.light_sleep_enable = true;
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    ESP_ERROR_CHECK(esp_pm_lock_create(
        ESP_PM_CPU_FREQ_MAX, 0, "vision_cpu", &s_vision_cpu_lock));
    ESP_ERROR_CHECK(esp_pm_lock_create(
        ESP_PM_NO_LIGHT_SLEEP, 0, "camera_awake", &s_camera_awake_lock));
    ESP_LOGI(TAG, "Power management ready: CPU %d-%d MHz, automatic light sleep",
             PM_MIN_CPU_FREQ_MHZ, PM_MAX_CPU_FREQ_MHZ);
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
    ESP_ERROR_CHECK(gpio_config(&alarm_config));
    ESP_ERROR_CHECK(gpio_set_level(BUZZER_GPIO, BUZZER_INACTIVE_LEVEL));
    ESP_ERROR_CHECK(gpio_set_level(LED_GPIO, LED_INACTIVE_LEVEL));
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
    init_power_management();
    init_nvs();
    init_gpio();

    /* Do not start sensing until the station has obtained an IPv4 address. */
    if (!wifi_init_sta()) {
        ESP_LOGE(TAG, "WiFi startup failed; restarting in %d ms",
                 NETWORK_RESTART_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(NETWORK_RESTART_DELAY_MS));
        esp_restart();
        return;
    }
    mqtt_app_start();

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
    s_start_event = xEventGroupCreate();
    if (s_vision_queue == NULL || s_alarm_queue == NULL || s_start_event == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate queues/startup event group");
        cleanup_startup_resources();
        return;
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
}
