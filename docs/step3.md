# 第三步：经典 ESP-DL 两阶段人脸检测

本文代码是第三步教学快照；最新电源守卫、曝光预热、检测框校验和网络功能
请以 `main/main.cpp` 及 [审查报告](project-audit.md) 为准，不要用旧片段覆盖它。

当前实现完整代码在 `main/main.cpp`。其中 GPIO/NVS/摄像头初始化承接
第二步，以下说明聚焦 C++、模型及队列通信。
当前工程还已接入第四步的真实 PIR 触发和声光报警，详见
[step4.md](step4.md)。

## 环境切换

将 `main.c` 改为 `main.cpp`，同步更新 main/CMakeLists.txt 中的 SRCS。
入口保留 `extern "C" void app_main(void)`，避免 C++ 名称修饰影响底层调用。

选用 `human_face_detect_msr01.hpp`、`human_face_detect_mnp01.hpp` 时使用
ESP-DL v2.0 和 ESP-IDF 5.0.x。最新版库和经典接口不可混用。

## 模型实例化与数据

```cpp
HumanFaceDetectMSR01 stage1(0.1F, 0.5F, 10, 0.2F);
HumanFaceDetectMNP01 stage2(0.5F, 0.3F, 5);

auto *pixels = reinterpret_cast<uint16_t *>(fb->buf);
const std::vector<int> shape{
    static_cast<int>(fb->height), static_cast<int>(fb->width), 3};

std::list<dl::detect::result_t> &candidates = stage1.infer(pixels, shape);
std::list<dl::detect::result_t> &results = candidates.empty()
    ? candidates : stage2.infer(pixels, shape, candidates);
```

stage1 的四个参数为候选置信度阈值、NMS 阈值、top_k、缩放比例。
stage2 的三个参数为置信度阈值、NMS 阈值、top_k。最终报警阈值独立为
`score > 0.7`。模型内部处理 RGB565，不必先申请 RGB888 整帧缓冲区。
形状中的通道数仍为 3；每像素占 2 字节由 `uint16_t *` 输入类型确定。

## 结果与报警

使用结果列表的引用，遍历 `score` 和 `box`。
箱体坐标合法且得分通过阈值的数量大于 0，才发送
`SIGNAL_HUMAN_CONFIRMED`。信号名保持上一阶段兼容，当前实际依据是人脸。
无人脸、低分、帧获取失败或可捕获的推理异常都不发送报警。

`esp_timer_get_time()` 测量模型调用的耗时，日志打印候选数、最终结果数、
通过阈值的结果数及坐标。日志耗时不包括相机采集及报警执行。

## 内存与失败处理

`CameraFrameGuard` 的析构函数归还帧，其复制操作被禁止，避免重复归还。
空指针没有待归还资源。非空帧无论检查失败、正常处理还是 C++ 异常展开
都会归还，之后才发送报警通知。

本实现启用 C++ 异常并捕获 STL `std::bad_alloc` 及其他异常。模型初始化
失败时先离开对象作用域，再结束视觉任务；不会把检测器当作裸指针遗忘。
旧版静态库没有通用错误码接口，内部断言/abort 不可通过 catch 恢复。

如果移植到关闭异常的工程，必须同时改写 catch 代码，并另行设计内存
不足的处理方式，不能只删除 sdkconfig 配置。

## 链接

```cmake
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_system esp_timer esp32-camera esp-dl
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
```

根目录 `components/esp-dl/` 必须包含完整官方 v2.0 库，由其 CMake 提供
头文件与静态库；默认 manifest 也能获取固定提交作为 managed component。
详见根目录 README 中的构建和硬件配置步骤。

## 完整核心代码片段

以下片段来自 `main/main.cpp`，展示包含文件、检测阈值、队列、帧归还守卫及完整
`vision_task`。它接入原有的三任务架构，不重复 GPIO/NVS/摄像头初始化。
`app_main` 必须使用 `extern "C" void app_main(void)`；创建两个长度为 1 的
信号队列和启动事件组后，成功创建全部任务，再设置 `APP_START_BIT`。

```cpp
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <inttypes.h>
#include <list>
#include <new>
#include <vector>

#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

/* Classic ESP-DL v2.0 APIs, used with ESP-IDF 5.0.x. */
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"

#if !CONFIG_COMPILER_CXX_EXCEPTIONS
#error "Enable CONFIG_COMPILER_CXX_EXCEPTIONS for model/STL allocation handling"
#endif

#define APP_START_BIT BIT0

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
                /* Frame guard has already returned the frame during unwinding. */
                ESP_LOGE(TAG, "Insufficient memory during face inference");
                continue;
            } catch (const std::exception &error) {
                ESP_LOGE(TAG, "Face inference failed: %s", error.what());
                continue;
            } catch (...) {
                ESP_LOGE(TAG, "Face inference failed with an unknown exception");
                continue;
            }

            /* Frame has been returned before notifying the alarm task. */
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
```

`CameraFrameGuard` 在离开当前作用域时归还非空帧。正常推理、格式校验失败
以及可捕获异常路径都遵守这一规则；帧归还后才通知报警任务。不要再对同一
帧额外调用 `esp_camera_fb_return()`，也不要对 `fb->buf` 调用 `free()`。

若使用本地完整的 `components/esp-dl/` v2.0 组件，`main/idf_component.yml`
可改为下面的形式，并继续保留 main/CMakeLists.txt 中的 `REQUIRES esp-dl`：

```yaml
dependencies:
  idf: "5.0.*"
  espressif/esp32-camera:
    version: "2.0.16"
```

根目录的 CMakeLists.txt 保持为：

```cmake
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32s3_edge_security)

if(NOT IDF_TARGET STREQUAL "esp32s3")
    message(FATAL_ERROR "This project requires idf.py set-target esp32s3")
endif()
```

根目录 `components/esp-dl/` 会被 ESP-IDF 自动发现。完整组件包含模型头文件、
实现和芯片对应的静态库，仅添加头文件路径不能完成链接。
