#pragma once

#include <arips_tf/math_2d.h>

class Door {
public:
    Point2d pivot;
    Point2d extent;
    double openingAngle; // around pivot, positive or negative
};


