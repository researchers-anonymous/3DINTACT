#include "i3dtimer.h"
#include <chrono>

const double MILLISECOND = 1000;

i3dtimer::i3dtimer()
{
    m_startTime = std::chrono::high_resolution_clock::now();
}
std::string i3dtimer::getDuration()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto startInstant
        = std::chrono::time_point_cast<std::chrono::microseconds>(m_startTime)
              .time_since_epoch()
              .count();
    auto stopInstant
        = std::chrono::time_point_cast<std::chrono::microseconds>(now)
              .time_since_epoch()
              .count();

    auto duration = stopInstant - startInstant;
    double ms = duration / MILLISECOND;
    return std::to_string(ms);
}
