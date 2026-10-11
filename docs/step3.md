# 第三步：经典 ESP-DL 两阶段人脸检测

当前实现完整代码在 `main/main.cpp`。其中 GPIO/NVS/摄像头初始化承接
第二步，以下说明聚焦 C++、模型及队列通信。

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
