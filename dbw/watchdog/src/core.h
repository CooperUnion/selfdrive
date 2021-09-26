#ifndef CORE_H
#define CORE_H

#include <atomic>
#include <array>
#include <cstring>

#define NUM_THREADS 4

namespace core 
{
    void estop();

    enum modes 
    {
        FULL_DRIVE,
        ONLY_STEERING,
        MANUAL,
        ESTOP
    };

    extern std::atomic<int> current_mode;
    //extern std::atomic<int> status;
    //extern std::atomic<std::array<int, NUM_THREADS>> status;
    extern std::array<std::atomic<std::int64_t>, NUM_THREADS> status;
}

#define COLOR_NORMAL "\e[m"
#define COLOR_BLACK "\e[30m"
#define COLOR_RED "\e[31m"
#define COLOR_GREEN "\e[32m"
#define COLOR_YELLOW "\e[33m"
#define COLOR_BLUE "\e[34m"
#define COLOR_MAGENTA "\e[35m"
#define COLOR_CYAN "\e[36m"
#define COLOR_WHITE "\e[37m"

#define LOGMSG(TASK, angry, msg, ...) do {\
    int _LOGMSG_len = (int)strlen(#TASK);\
    int _LOGMSG_adj = (16-_LOGMSG_len)%2 ? 0 : 1;\
    fprintf(stderr, "%s[ %*c%s%*c ]\t" COLOR_NORMAL msg "\n", \
        (angry ? COLOR_RED : COLOR_GREEN), \
        (16-_LOGMSG_len)/2, ' ', #TASK, \
        (16-_LOGMSG_len)/2 - _LOGMSG_adj, ' '\
        __VA_OPT__(,) __VA_ARGS__);\
} while(0)

#endif
