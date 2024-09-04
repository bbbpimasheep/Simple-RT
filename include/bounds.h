#pragma once
#ifndef BOUNDS_H
#define BOUNDS_H

#include "global.h"
#include "ray.h"
#include "mathematics.h"

class Bounds3 {
public:
    // Constructors
    Bounds3() = default;
    Bounds3(const Interval& _x, const Interval& _y, const Interval& _z) : x(_x), y(_y), z(_z) {
        // Adjust the Bounds so that no side is narrower than some delta, padding if necessary.
        if (x.size < EPS_UNIT) x = x.Expand(EPS_UNIT);
        if (y.size < EPS_UNIT) y = y.Expand(EPS_UNIT);
        if (z.size < EPS_UNIT) z = z.Expand(EPS_UNIT);
    }
    Bounds3(const Point3& a, const Point3& b) {
        x = a.x <= b.x? Interval(a.x, b.x) : Interval(b.x, a.x);
        y = a.y <= b.y? Interval(a.y, b.y) : Interval(b.y, a.y);
        z = a.z <= b.z? Interval(a.z, b.z) : Interval(b.z, a.z);
        if (x.size < EPS_UNIT) x = x.Expand(EPS_UNIT);
        if (y.size < EPS_UNIT) y = y.Expand(EPS_UNIT);
        if (z.size < EPS_UNIT) z = z.Expand(EPS_UNIT);
    }

    // Methods
    Interval operator[](int i) const { return (i == 0) ? x : (i == 1) ? y : z; }
    Interval &operator[](int i) { return (i == 0) ? x : (i == 1) ? y : z; }
    bool Intersect(const Ray& ray, Interval ray_t) const {
        const Point3& ray_o = ray.org;
        const Vector3& ray_d = ray.dir;
        for (int axis = 0; axis < 3; axis += 1) {
            const Interval& intrv = (*this)[axis];
            const double t_inv = 1.0 / ray_d[axis];
            auto t0 = (intrv._min - ray_o[axis]) * t_inv;
            auto t1 = (intrv._max - ray_o[axis]) * t_inv;

            if (t0 > t1) std::swap(t0, t1);
            if (t0 > ray_t._min) ray_t._min = t0;
            if (t1 < ray_t._max) ray_t._max = t1;
            if (ray_t._min >= ray_t._max) 
                return false;
        }
        return true;
    }
    int MaxAxis() const {
        if (x.size > y.size) return (x.size > z.size) ? 0 : 2;
        else return (y.size > z.size) ? 1 : 2;
    }
    double SurfaceArea() const {
        double area = 2 * (x.size * y.size + x.size * z.size + y.size * z.size);
        return area;
    }

    // Members
    Interval x, y, z;

    // Conditions
    static const Bounds3 Empty, Universe;
};

// Inline Functions
inline Bounds3 Union(const Bounds3& box1, const Bounds3& box2) {
    auto bbox_x = Interval(box1.x, box2.x);
    auto bbox_y = Interval(box1.y, box2.y);
    auto bbox_z = Interval(box1.z, box2.z);
    return Bounds3(bbox_x, bbox_y, bbox_z);
}


#endif // BOUNDS_H