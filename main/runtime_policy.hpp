#pragma once
#include <cstdint>

namespace security {

/* After a burst of retries, keep trying at a slower rate indefinitely. */
class RetryBackoff {
public:
    explicit RetryBackoff(unsigned max_attempts = 5) : max_attempts_(max_attempts) {}
    uint32_t next_delay_ms()
    {
        if (++attempts_ < max_attempts_) {
            return 2000;
        }
        attempts_ = 0;
        return 30000;
    }
    void reset() { attempts_ = 0; }
private:
    unsigned attempts_ = 0;
    const unsigned max_attempts_;
};

inline bool valid_detection_box(int x1, int y1, int x2, int y2,
                                int width, int height)
{
    return x1 >= 0 && y1 >= 0 && x2 > x1 && y2 > y1 &&
           x2 < width && y2 < height;
}

inline bool alert_expired(int64_t created_us, int64_t now_us)
{
    return now_us < created_us || now_us - created_us > INT64_C(300000000);
}

inline bool outbox_has_room(int current_bytes, int message_bytes)
{
    /* Reserve space for the encoded topic/header in addition to JSON. */
    return current_bytes >= 0 && message_bytes >= 0 &&
           current_bytes <= 4096 && message_bytes <= 4096 - current_bytes;
}

} // namespace security
