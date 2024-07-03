#pragma once

#include <array>

template<typename time_t, unsigned len>
struct ODESolver {
    virtual void solve(const std::array<time_t, len> &x_start, 
                       std::array<time_t, len> &x_end,
                       time_t t0, 
                       time_t t1, 
                       std::array<time_t, len> &x_dot
                       ) = 0;
};

template<typename time_t, unsigned len>
struct ExplicitSolver : public ODESolver<time_t, len> {
    virtual void solve(const std::array<time_t, len> &x_start, 
                       std::array<time_t, len> &x_end,
                       time_t t0, 
                       time_t t1, 
                       std::array<time_t, len> &x_dot
                       ) override;
};

template<typename time_t, unsigned len>
void ExplicitSolver<time_t, len>::solve(const std::array<time_t, len> &x_start, 
                       std::array<time_t, len> &x_end,
                       time_t t0, 
                       time_t t1, 
                       std::array<time_t, len> &x_dot
                       )
{
    float delta_t = t1 - t0;
    for (int i = 0; i < len; ++i) {
        x_end[i] = x_start[i] + x_dot[i] * delta_t;
    }
}