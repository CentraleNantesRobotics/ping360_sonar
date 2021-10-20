#include "../ping-time.h"

#include <chrono>
#include <thread>

int PingTime::timeMs()
{
    auto now = std::chrono::steady_clock::now();

    auto duration = now.time_since_epoch();
    auto durationMs = std::chrono::duration_cast<std::chrono::milliseconds>(duration);

    return durationMs.count();
}

void PingTime::microsecondDelay(unsigned int microseconds)
{
    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

void PingTime::yeild() { std::this_thread::yield(); }
