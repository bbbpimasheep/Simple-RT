#pragma once
#ifndef INTERVAL_H
#define INTERVAL_H

#include "global.h"

class Interval {
public:
    // Constructors
    Interval() : _min(POS_INF), _max(NEG_INF), size(0) {}
    Interval(double min, double max) : _min(min), _max(max), size(max-min) {}
    Interval(const Interval& a, const Interval& b) {
        _min = Min(a._min, b._min); _max = Max(a._max, b._max);
        size = _max - _min;
    }

    // Methods
    bool Contains(double value) const { return value >= _min && value <= _max; }
    bool Surrounds(double value) const { return value > _min && value < _max; }
    double Clamp(double value) const { return Max(_min, Min(_max, value)); }
    double Centroid() const { return (_min + _max) * 0.5; }
    Interval Expand(double amount) const { return Interval(_min - amount/2, _max + amount/2); }

    // Members
    double _min, _max;
    double size;

    // Conditions
    static const Interval Empty, Universe;
};


#endif // INTERVAL_H