#pragma once
#ifndef INTERVAL_H
#define INTERVAL_H

#include "global.h"

class Interval {
public:
    // Constructors
    Interval() : _min(POS_INF), _max(NEG_INF), size(0) {}
    Interval(double min, double max) : _min(min), _max(max), size(max-min) {}

    // Methods
    bool Contains(double value) const { return value >= _min && value <= _max; }
    bool Surrounds(double value) const { return value > _min && value < _max; }
    double Clamp (double value) const { return Max(_min, Min(_max, value)); }

    // Members
    double _min, _max;
    double size;

    // Conditions
    static const Interval EMPTY, UNIVERSE;
};

const Interval Interval::EMPTY    = Interval(POS_INF, NEG_INF);
const Interval Interval::UNIVERSE = Interval(NEG_INF, POS_INF);


#endif // INTERVAL_H