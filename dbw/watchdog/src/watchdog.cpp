#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include "watchdog.h"
#include "core.h"
#include "test_estop.h"
#include "mod_canifstate.h"
#include "mod_canhealth.h"
#include <signal.h>
#include <atomic>

#define NAME_OF( v ) #v

std::array<void(*)(const int), 2> modules
{
    mod_canifstate,
    mod_canhealth
};

void ctrlcsignal(int);
void exit_now();
volatile int sigcaught = 0;

int main()
{
    LOGMSG(watchdog, 0, "Cooper Union IGVC DBW Watchdog v0.0.0a");
    LOGMSG(watchdog, 0, "-----------------INIT-----------------");
    // set up handler for ctrl+c
    struct sigaction estop_sig;
    estop_sig.sa_handler = ctrlcsignal;
    sigemptyset(&estop_sig.sa_mask);
    if (sigaction(SIGINT, &estop_sig, NULL) < 0)
    {
        LOGMSG(watchdog, 1, "Error with sigaction");
    }
    LOGMSG(watchdog, 0, "Number of modules: %lu", modules.size());
    LOGMSG(watchdog, 0, "launching modules...");
    std::vector<std::thread> threads;
    core::current_mode.store(core::modes::FULL_DRIVE);
    //printf("the current driving mode is %d\n", core::current_mode.load());
    //std::thread mod_test_estop (test_estop);
    //std::thread mod_sample_task (sample_task);
    for (int t = 0; t<modules.size(); t++) {
        threads.push_back(std::thread(modules.at(t), t));
    }
    //printf("the current driving mode is %d\n", core::current_mode.load());
    int buffer[modules.size()] = {0};
    int leave = 0;

    for (;;)
    {
        if (sigcaught) { // use of many functions (like fprintf) is limited inside signal handlers, so do it here
            putchar('\n');
            LOGMSG(watchdog, 1, "!!! Caught SIGINT, will exit on next occurence!");
            sigcaught = 0;
        }

        for(int i=0; i < modules.size(); ++i) {
            buffer[i] = core::status[i].load();
            //printf("buffer: index %d, Value: %d\n", i, buffer[i]);
        }

        usleep(100 * 1000);

        for(int i=0; i < modules.size(); ++i) {
            // printf("buffer: thread %d, Buffer Value: %d, Status Value: %ld\n", i, buffer[i], core::status[i].load());
            if (buffer[i] == core::status[i].load()) {
                LOGMSG(watchdog, 1, "Stopped due to dead thread: #%d", i);
                core::estop();
            }
        }
    }
}

void ctrlcsignal(int c) {
    static int count = 0;
    if (!count) {
        sigcaught++;
        count++;
    }
    else {
        exit(20);
    }
}

void exit_now() {
    LOGMSG(watchdog, 1, "**** EXITING ****");
    // when the program exits, ALL the threads die too
    exit(0);
}
