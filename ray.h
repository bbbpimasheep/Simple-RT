#pragma once
#ifndef RAY_H
#define RAY_H

#include "maths.h"
#include "global.h"

class Ray {
public:
    // Constructor
    Ray() = default; 
    Ray(const Point3& o_, const Vector3& d_, double t_) : org(o_), dir(d_), time(t_) {}
    Ray(const Point3& o_, const Vector3& d_) : org(o_), dir(d_), time(0.0) {}

    // Methods
    Vector3 operator()(float t) const { return org + dir * t; }

    // Members
    Point3 dir;
    Vector3 org;
    double time;
};

// Debugging
inline std::string Str(const Ray& r)
{ return "[Origin: " + Str(r.org) + ", Direction: " + Str(r.dir) + "]"; }


#endif // RAY_H