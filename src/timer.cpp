#include "timer.hpp"

namespace PoseGraph{
Timer::Timer() : m_start(std::chrono::high_resolution_clock::now()),
                 m_last_check(std::chrono::high_resolution_clock::now()) {}

void Timer::reset() {
    m_start = std::chrono::high_resolution_clock::now();
}

double Timer::elapsedSinceStart() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - m_start).count() / 1000.0;
}

double Timer::elapsedSinceCheck() {
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = end - m_last_check;
    m_last_check = end;
    return std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() / 1000.0;
}
} // namespace PoseGraph