#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include "core.h"

using namespace core;

void stopsignal(int signum)
{
    core::estop();
}

void test_estop()
{
    printf("test_estop thread is: %d\n", getpid());

    struct sigaction estop_sig;
    estop_sig.sa_handler = stopsignal;
    sigemptyset(&estop_sig.sa_mask);
    if (sigaction(SIGUSR1, &estop_sig, NULL) < 0)
    {
        fprintf(stderr, "error with sigaction\n");
    }

    for (;;)
        sleep(1);
}
