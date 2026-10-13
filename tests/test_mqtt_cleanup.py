#!/usr/bin/env python3
"""Compile production MQTT payload logic against allocation/API failure mocks."""
from pathlib import Path
import subprocess
import tempfile

project = Path(__file__).resolve().parents[1]
main = (project / "main/main.cpp").read_text(encoding="utf-8")
start = main.index("static bool publish_mqtt_alert(")
end = main.index("\nstatic void mqtt_alert_task", start)
function = main[start:end]
mock = r'''
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include "runtime_policy.hpp"
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
#define MQTT_TOPIC "esp32s3/security/alert"
struct cJSON {};
void *mqtt_client = reinterpret_cast<void *>(1);
int allocation_step, fail_at, live_objects, live_strings, enqueue_result, outbox_size;
cJSON *cJSON_CreateObject() {
    if (++allocation_step == fail_at) return nullptr;
    ++live_objects; return new cJSON;
}
void cJSON_Delete(cJSON *root) {
    if (root) { --live_objects; delete root; }
}
cJSON *add_field(cJSON *root) {
    return ++allocation_step == fail_at ? nullptr : root;
}
cJSON *cJSON_AddStringToObject(cJSON *root, const char *, const char *) { return add_field(root); }
cJSON *cJSON_AddNumberToObject(cJSON *root, const char *, double value) {
    assert(value == 123456789); return add_field(root);
}
char *cJSON_PrintUnformatted(cJSON *) {
    if (++allocation_step == fail_at) return nullptr;
    ++live_strings;
    char *payload = static_cast<char *>(std::malloc(3));
    std::strcpy(payload, "{}"); return payload;
}
void tracked_free(void *payload) {
    if (payload) { --live_strings; std::free(payload); }
}
int esp_mqtt_client_get_outbox_size(void *) { return outbox_size; }
int esp_mqtt_client_enqueue(void *, const char *topic, const char *payload,
                            int len, int qos, int retain, bool store) {
    assert(std::strcmp(topic, MQTT_TOPIC) == 0 && std::strcmp(payload, "{}") == 0);
    assert(len == 0 && qos == 1 && retain == 0 && store);
    return enqueue_result;
}
#define free tracked_free
'''
checks = r'''
int main() {
    for (int cycle = 0; cycle < 1000; ++cycle) {
        for (int failure = 0; failure <= 5; ++failure) {
            fail_at = failure; allocation_step = 0;
            assert(publish_mqtt_alert(123456789) == (failure == 0));
            assert(live_objects == 0 && live_strings == 0);
        }
    }
    fail_at = 0;
    mqtt_client = nullptr; assert(!publish_mqtt_alert(123456789));
    mqtt_client = reinterpret_cast<void *>(1);
    enqueue_result = -1; assert(!publish_mqtt_alert(123456789));
    enqueue_result = 1; outbox_size = 4096; assert(!publish_mqtt_alert(123456789));
    assert(live_objects == 0 && live_strings == 0);
}
'''
with tempfile.TemporaryDirectory(prefix="esp32s3-mqtt-test-") as directory:
    binary = str(Path(directory) / "mqtt_test")
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-I", str(project / "main"), "-x", "c++", "-", "-o", binary],
                   input=mock + function + checks, text=True, check=True)
    subprocess.run([binary], check=True)
print("PASS: MQTT/JSON failure cleanup, QoS1 enqueue and 1000 allocation cycles")
