#!/usr/bin/env python3
"""Exercise production pure policies using the host C++17 compiler."""
from pathlib import Path
import subprocess
import tempfile

project = Path(__file__).resolve().parents[1]
source = r'''
#include "runtime_policy.hpp"
#include <cassert>
#include <climits>
int main() {
    security::RetryBackoff retry;
    for (int burst = 0; burst < 1000; ++burst) {
        for (int i = 0; i < 4; ++i) assert(retry.next_delay_ms() == 2000);
        assert(retry.next_delay_ms() == 30000);
    }
    retry.next_delay_ms(); retry.reset();
    assert(retry.next_delay_ms() == 2000);
    security::RetryBackoff single(1);
    assert(single.next_delay_ms() == 30000);
    assert(security::valid_detection_box(0, 0, 319, 239, 320, 240));
    assert(!security::valid_detection_box(-1, 0, 20, 20, 320, 240));
    assert(!security::valid_detection_box(0, -1, 20, 20, 320, 240));
    assert(!security::valid_detection_box(10, 10, 10, 20, 320, 240));
    assert(!security::valid_detection_box(10, 20, 20, 20, 320, 240));
    assert(!security::valid_detection_box(20, 10, 10, 20, 320, 240));
    assert(!security::valid_detection_box(0, 0, 320, 239, 320, 240));
    assert(!security::valid_detection_box(0, 0, 319, 240, 320, 240));
    assert(!security::valid_detection_box(0, 0, 1, 1, 0, 0));
    assert(!security::alert_expired(100, 100));
    assert(!security::alert_expired(100, 300000100));
    assert(security::alert_expired(100, 300000101));
    assert(security::alert_expired(100, 99));
    assert(security::outbox_has_room(0, 4096));
    assert(security::outbox_has_room(4096, 0));
    assert(!security::outbox_has_room(4096, 1));
    assert(!security::outbox_has_room(4097, 0));
    assert(!security::outbox_has_room(-1, 100));
    assert(!security::outbox_has_room(0, -1));
    assert(!security::outbox_has_room(INT_MAX, INT_MAX));
}
'''
with tempfile.TemporaryDirectory(prefix="esp32s3-policy-test-") as directory:
    binary = str(Path(directory) / "policy_test")
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-I", str(project / "main"), "-x", "c++", "-", "-o", binary],
                   input=source, text=True, check=True)
    subprocess.run([binary], check=True)
print("PASS: indefinite reconnect bursts, box bounds, alert expiry and outbox limits")
