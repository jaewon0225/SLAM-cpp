#pragma once
#include <chrono>

namespace Timer{
class Timer {
public:
    Timer();
    void reset();
    double elapsedSinceStart() const;
    double elapsedSinceCheck();

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_start;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_last_check;
    
};
} // namespace Timer