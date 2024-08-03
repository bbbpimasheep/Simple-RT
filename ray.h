#pragma once
#ifndef RAY_H
#define RAY_H

#include "vector.h"
#include "global.h"

class Ray {
public:
    // Constructor
    Ray() = default; 
    Ray(const Vector3& o_, const Point3& d_) : org(o_), dir(d_) {}

    // Methods
    Vector3 operator()(float t) const { return org + dir * t; }

    // Members
    Point3 dir;
    Vector3 org;
};

// Debugging
inline std::string Str(const Ray& r)
{ return "[Origin: " + Str(r.org) + ", Direction: " + Str(r.dir) + "]"; }


#endif // RAY_H