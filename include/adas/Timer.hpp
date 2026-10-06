#pragma once

#include <chrono>

namespace adas {


class Timer {
public:
    using Clock = std::chrono::steady_clock;

    Timer() noexcept : start_(Clock::now()) {}

    void reset() noexcept { start_ = Clock::now(); }

    [[nodiscard]] double elapsedMs() const noexcept {
        return std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
    }

private:
    Clock::time_point start_;
};

}  