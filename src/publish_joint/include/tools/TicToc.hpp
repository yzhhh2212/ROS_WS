#ifndef _TIC_TOC_HPP_
#define _TIC_TOC_HPP_

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc {

public:
    TicToc(void)
    {
        tic();
    }

    void tic(void)
    {
        start = std::chrono::system_clock::now();
    }

    double toc(void)
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        start = end;
        return elapsed_seconds.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
};


#endif