#!/usr/bin/env python3
"""Compile the production RAII guards against mocks and check cleanup order.

Run with: python3 tests/test_power_lifecycle.py
This host test does not simulate ESP32 sleep, Wi-Fi, or camera hardware.
"""

from pathlib import Path
import subprocess
import tempfile


project = Path(__file__).resolve().parents[1]
source = (project / "main/main.cpp").read_text(encoding="utf-8")


def production_class(name):
    start = source.index(f"class {name} {{")
    end = source.index("\n};", start) + len("\n};")
    return source[start:end]


preamble = r"""
#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
#define ESP_ERROR_CHECK(expr) do { assert((expr) == ESP_OK); } while (0)

struct Lock { int count = 0; const char *name; };
using esp_pm_lock_handle_t = Lock *;
Lock cpu_lock{0, "cpu"}, awake_lock{0, "awake"};
esp_pm_lock_handle_t s_vision_cpu_lock = &cpu_lock;
esp_pm_lock_handle_t s_camera_awake_lock = &awake_lock;
std::vector<std::string> events;
bool camera_active = false;
bool frame_held = false;
bool fail_camera_init = false;
struct camera_fb_t {};
camera_fb_t frame_buffer;

esp_err_t esp_pm_lock_acquire(esp_pm_lock_handle_t lock) {
    ++lock->count;
    events.push_back(std::string("acquire_") + lock->name);
    return ESP_OK;
}
esp_err_t esp_pm_lock_release(esp_pm_lock_handle_t lock) {
    assert(lock->count > 0);
    assert(!camera_active && !frame_held);
    --lock->count;
    events.push_back(std::string("release_") + lock->name);
    return ESP_OK;
}
esp_err_t init_camera() {
    assert(cpu_lock.count == 1 && awake_lock.count == 1);
    assert(!camera_active);
    events.push_back("camera_init");
    if (fail_camera_init) return -1;
    camera_active = true;
    return ESP_OK;
}
esp_err_t esp_camera_deinit() {
    assert(cpu_lock.count == 1 && awake_lock.count == 1);
    assert(camera_active && !frame_held);
    camera_active = false;
    events.push_back("camera_deinit");
    return ESP_OK;
}
camera_fb_t *esp_camera_fb_get() {
    assert(camera_active && !frame_held);
    frame_held = true;
    events.push_back("frame_get");
    return &frame_buffer;
}
void esp_camera_fb_return(camera_fb_t *frame) {
    assert(frame == &frame_buffer && camera_active && frame_held);
    assert(cpu_lock.count == 1 && awake_lock.count == 1);
    frame_held = false;
    events.push_back("frame_return");
}
"""

test_body = r"""
void assert_idle() {
    assert(cpu_lock.count == 0 && awake_lock.count == 0);
    assert(!camera_active && !frame_held);
}
void assert_events(std::vector<std::string> expected) {
    assert(events == expected);
    assert_idle();
    events.clear();
}
const std::vector<std::string> capture_events{
    "acquire_awake", "acquire_cpu", "camera_init", "frame_get",
    "frame_return", "camera_deinit", "release_cpu", "release_awake"
};

void early_return() {
    VisionPowerGuard power;
    CameraSessionGuard camera;
    assert(camera.start() == ESP_OK);
    CameraFrameGuard frame(esp_camera_fb_get());
    return;
}

int main() {
    // Normal capture, early return, and repeated cycles must not accumulate locks.
    for (int cycle = 0; cycle < 1000; ++cycle) {
        early_return();
        assert_events(capture_events);
    }

    // continue (e.g. invalid frame) follows the same cleanup order.
    for (int iteration = 0; iteration < 1; ++iteration) {
        VisionPowerGuard power;
        CameraSessionGuard camera;
        assert(camera.start() == ESP_OK);
        CameraFrameGuard frame(esp_camera_fb_get());
        continue;
    }
    assert_events(capture_events);

    // Inference throws; the frame returns before camera shutdown and lock release.
    try {
        VisionPowerGuard power;
        CameraSessionGuard camera;
        assert(camera.start() == ESP_OK);
        CameraFrameGuard frame(esp_camera_fb_get());
        throw std::runtime_error("inference failed");
    } catch (const std::runtime_error &) {}
    assert_events(capture_events);

    // Capture failure returns no frame, but the initialized camera must close.
    {
        VisionPowerGuard power;
        CameraSessionGuard camera;
        assert(camera.start() == ESP_OK);
        CameraFrameGuard frame(nullptr);
    }
    assert_events({"acquire_awake", "acquire_cpu", "camera_init",
                   "camera_deinit", "release_cpu", "release_awake"});

    // Init failure must not deinit an uninitialized camera or retain locks.
    fail_camera_init = true;
    {
        VisionPowerGuard power;
        CameraSessionGuard camera;
        assert(camera.start() != ESP_OK);
    }
    assert_events({"acquire_awake", "acquire_cpu", "camera_init",
                   "release_cpu", "release_awake"});
    fail_camera_init = false;

    // Boot validation closes the camera before the application starts waiting.
    {
        VisionPowerGuard power;
        CameraSessionGuard camera;
        assert(camera.start() == ESP_OK);
    }
    assert_events({"acquire_awake", "acquire_cpu", "camera_init",
                   "camera_deinit", "release_cpu", "release_awake"});
}
"""

guards = "\n".join(production_class(name) for name in (
    "VisionPowerGuard", "CameraSessionGuard", "CameraFrameGuard"
))
with tempfile.TemporaryDirectory(prefix="esp32s3-pm-test-") as temp_dir:
    binary = str(Path(temp_dir) / "test_power_lifecycle")
    subprocess.run(
        ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
         "-x", "c++", "-", "-o", binary],
        input=preamble + guards + test_body, text=True, check=True,
    )
    subprocess.run([binary], check=True)
print("PASS: frame -> camera -> PM lock cleanup, failures and 1000 cycles")
