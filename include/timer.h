#ifndef _TIMER_H_
#define _TIMER_H_
#include <iostream>
#include <chrono>

namespace timer{
    void tic();
    double toc(bool flag_verbose = false);
    const std::string currentDateTime();
};
#endif