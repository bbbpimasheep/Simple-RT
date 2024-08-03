#pragma once
#ifndef VECTOR_H
#define VECTOR_H

#include "global.h"

class Sphere;

class Vector3 {
public:
    // Constructor
    Vector3() = default;
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vector3(double v_) : x(v_), y(v_), z(v_) {}
    Vector3(const Vector3& v) : x(v.x), y(v.y), z(v.z) {}

    // Methods
    double operator[](int i) const { return (i == 0) ? x : (i == 1) ? y : z; }
    double &operator[](int i) { return (i == 0) ? x : (i == 1) ? y : z; }
    Vector3 operator+(const Vector3 v) const { return Vector3(x+v.x, y+v.y, z+v.z); }
    Vector3 operator-(const Vector3 v) const { return Vector3(x-v.x, y-v.y, z-v.z); }
    Vector3 operator-() const { return Vector3(-x, -y, -z); }
    Vector3 operator*(double s) const { return Vector3(x*s, y*s, z*s); }
    Vector3 operator/(double s) const { return Vector3(x/s, y/s, z/s); }
    Vector3& operator+=(const Vector3 v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vector3& operator-=(const Vector3 v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vector3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
    Vector3& operator/=(double s) { x /= s; y /= s; z /= s; return *this; }
    bool operator==(const Vector3 v) const { return x == v.x && y == v.y && z == v.z; }
    bool operator!=(const Vector3 v) const { return x!= v.x || y!= v.y || z!= v.z; }

    // Members
    double x=0, y=0, z=0;
};

using Point3 = Vector3;

// Inline Functions
inline bool IsZero(const Vector3& v) { return v.x < EPS_DEUX && v.y < EPS_DEUX && v.z < EPS_DEUX; }
inline double Length2(const Vector3 v) { return Sqr(v.x) + Sqr(v.y) + Sqr(v.z); }
inline double Length(const Vector3 v) { return Sqrt(Length2(v)); }
inline double Dot(const Vector3 v1, const Vector3 v2) { return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z; }
inline Vector3 operator*(double s, const Vector3 v) { return v*s; }  // Scalar Front Multiplication
inline Vector3 operator*(const Vector3& v, const Vector3& u) { return Vector3(v.x*u.x, v.y*u.y, v.z*u.z); }
inline Vector3 Normalize(const Vector3 v) // Prevent Division by Zero 
{ Vector3 u = v; if (IsZero(u)) u /= EPS_DEUX; return u / Length(u); }
inline void    Unitize(Vector3& v) { v = Normalize(v); }
inline Vector3 Lerp(const Vector3& v1, const Vector3& v2, double t) { return (1-t)*v1 + t*v2; }
inline Vector3 RandomVec3() { return Vector3(RandomFloat(), RandomFloat(), RandomFloat()); }
inline Vector3 RandomVec3(double min, double max) { 
    return Vector3(RandomFloat(min, max), 
                   RandomFloat(min, max), 
                   RandomFloat(min, max));
}
inline Vector3 RandomVec3Unit() {
    auto θ = M_PI   * RandomFloat();
    auto φ = 2*M_PI * RandomFloat();
    return Vector3(Sin(φ)*Cos(θ), Sin(φ)*Sin(θ), Cos(φ));
}
inline Vector3 RandomVec3Disk() {
    auto u = Vector3(RandomFloat(-1,1), RandomFloat(-1,1), 0);
    if (u.x == 0 && u.y == 0) return {0, 0, 0};
    double radius, theta;
    if (Abs(u.x) > Abs(u.y)) {
        radius = u.x;
        theta = M_PI_4 * u.y/u.x;
    } else {
        radius = u.y;
        theta = M_PI_2 - M_PI_4 * u.x/u.y;
    }
    return radius * Vector3(Cos(theta), Sin(theta), 0);
}
inline Vector3 Cross(const Vector3 v1, const Vector3 v2) { 
    return Vector3(v1.y*v2.z - v1.z*v2.y, 
                   v1.z*v2.x - v1.x*v2.z, 
                   v1.x*v2.y - v1.y*v2.x); 
}
inline Vector3 Reflect(const Vector3& wi, const Vector3& n) // v outward n
{ return 2 * Dot(wi,n) * n - wi; }
inline bool    Refract(const Vector3& wi, Vector3& wt, const Vector3& n, double eta) { 
    // wi outward n // etat / etai
    double cos_i = Min(Dot(wi,n), 1.0);
    double sin2i = 1 - Sqr(cos_i);
    double sin2r = sin2i / Sqr(eta);
    if (sin2r >= 1) return false;  // Total Internal Reflection
    double cos_r = Sqrt(Max(EPS_QUAT, 1-sin2r));
    wt = -wi/eta + (cos_i/eta - cos_r) * n;
    return true;
}

// Debugging
inline std::string Str(const Vector3 v) 
{ return "(" + Str(v.x) + ", " + Str(v.y) + ", " + Str(v.z) + ")"; }


#endif // VECTOR_H