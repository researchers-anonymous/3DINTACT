#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <string>

class i3dtimer {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;

public:
    i3dtimer();
    ~i3dtimer() = default;
    std::string getDuration();
};

#endif /* TIMER_H */
