#pragma once
#ifndef COLOUR_H
#define COLOUR_H

#include "vector.h"
#include "interval.h"
#include "global.h"

using Colour = Vector3;

inline void WriteColour(const Colour& pic_colour, std::ostream& os) {
    auto r = std::pow(pic_colour.x, GAMMA);
    auto g = std::pow(pic_colour.y, GAMMA);
    auto b = std::pow(pic_colour.z, GAMMA);
    // Translate the [0,1] component values to the byte range [0,255].
    static const Interval intensity(0, 1-EPS_DEUX);
    int rbyte = int(256 * intensity.Clamp(r));
    int gbyte = int(256 * intensity.Clamp(g));
    int bbyte = int(256 * intensity.Clamp(b));
    // Write the bytes to the output stream.
    os << rbyte << " " << gbyte << " " << bbyte << '\n';
}
inline Colour RandomColour() { return RandomVec3(); }
inline Colour RandomColour(double min, double max) { return RandomVec3(min, max); }

#endif // COLOUR_H