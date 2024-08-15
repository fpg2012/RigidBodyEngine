#pragma once

#include <cassert>
struct Interval {
    float l, r;

    Interval(float l, float r) : l(l), r(r) {
        assert(l < r);
    }

    // return true if `r < oth.r`
    bool operator<(const Interval &oth) {
        return r < oth.r;
    }
};

struct BoundingBox3D {
    Interval ix, iy, iz;
};