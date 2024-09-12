#pragma once
#include "interval.h"
#include "vector.h"
#include "colour.h"
#include "ray.h"
#include "transformation.h"

// Inline Functions
inline bool TriangleIsect(const Vector3& edge1, const Vector3& edge2, 
                          const Point3& p0, const Ray& ray, 
                          double& t, double& u, double& v) {                      
    auto p_vec = Cross(ray.dir, edge2);
    auto det = Dot(edge1, p_vec);
    auto inv_det = 1.0 / det;
    if (Abs(det) < EPS_QUAT) 
        return false;
    auto t_vec = ray.org - p0;
    u = Dot(t_vec, p_vec) * inv_det;
    if (u < 0 || u > 1) 
        return false;
    auto q_vec = Cross(t_vec, edge1);
    v = Dot(ray.dir, q_vec) * inv_det;
    if (v < 0 || u + v > 1) 
        return false;
    t = Dot(edge2, q_vec) * inv_det;

    return true;
}
inline Vector3 SampleU_Hemisphere(const Vector3& normal) {
    auto u1 = RandomFloat(), u2 = RandomFloat();
    auto z = u1, r = Sqrt(Max(0.0, 1.0 - z*z));
    auto phi = u2 * 2*M_PI;
    auto local_sample = Vector3(r * Cos(phi), r * Sin(phi), z);

    Vector3 vec_b, vec_c;
    if (Abs(normal.x) > Abs(normal.y)) {
        auto length_inv = 1.0 / Sqrt(normal.x*normal.x + normal.z*normal.z);
        vec_c = Vector3(normal.z * length_inv, 0.0, -normal.x * length_inv);
    } else {
        auto length_inv = 1.0 / Sqrt(normal.y*normal.y + normal.z*normal.z);
        vec_c = Vector3(0.0, normal.z * length_inv, -normal.y * length_inv);
    }
    vec_b = Cross(vec_c, normal);
    return local_sample.x*vec_b + local_sample.y*vec_c + local_sample.z*normal;
}

// Direction
inline double CosΘ (Vector3& _v) { return _v.z / Length(_v); }
inline double Cos2Θ(Vector3& _v) { return Sqr(_v.z) / Length2(_v); }
inline double SinΘ (Vector3& _v) { return Sqrt(Max(0.0, 1 - Sqr(_v.z))); }
inline double Sin2Θ(Vector3& _v) { return Max(0.0, 1 - Sqr(_v.z)); }
inline double TanΘ (Vector3& _v) { return SinΘ(_v) / CosΘ(_v); }
inline double Tan2Θ(Vector3& _v) { return Sin2Θ(_v) / Cos2Θ(_v); }
inline double Cosφ (Vector3& _v) { 
    auto sin_theta = SinΘ(_v);
    return sin_theta == 0 ? 1 : Min(1.0, Max(-1.0, _v.x / sin_theta));
}
inline double Sinφ (Vector3& _v) {
    auto sin_theta = SinΘ(_v);
    return sin_theta == 0 ? 0 : Min(1.0, Max(-1.0, _v.y / sin_theta));
}
inline Vector3 Reflect(const Vector3& wi, const Vector3& n) // v outward n
{ return 2 * Dot(wi,n) * n - wi; }
inline bool Refract(const Vector3& wi, Vector3& wt, Vector3 n, double eta, double& etap) { 
    // wi outward n // etat / etai
    double cos_i = Min(Dot(wi,n), 1.0);
    if (cos_i < 0) {    // Potentially flip interface orientation for Snell’s law
        cos_i = -cos_i; eta = 1 / eta; n = -n;
    }
    double sin2_i = Max(0.0, 1 - Sqr(cos_i));
    double sin2_t = sin2_i / Sqr(eta);
    if (sin2_t >= 1) 
        return false;  // Total Internal Reflection
    double cos_t = Sqrt(Max(EPS_QUAT, 1-sin2_t));
    wt = Normalize(-wi/eta + (cos_i/eta - cos_t) * n);
    if (etap) etap = eta;
    return true;
}

// Coordinate system
inline Vector3 ToLocal(const Vector3& n, const Vector3& v) {
    auto z = Normalize(n);      // z-axis upward
    auto temp = Vector3(1, 0, 0);
    if (Abs(Dot(z, temp)) > 0.99) temp = Vector3(0, 1, 0);
    Vector3 x = Normalize(Cross(temp, z)); 
    Vector3 y = Normalize(Cross(z, x));         
    return Vector3(Dot(v,x), Dot(v,y), Dot(v,z));
}
inline Vector3 ToWorld(const Vector3& n, const Vector3& v) {
    auto z = Normalize(n);      // z-axis upward
    auto temp = Vector3(1, 0, 0);
    if (Abs(Dot(z, temp)) > 0.99) temp = Vector3(0, 1, 0);
    Vector3 x = Normalize(Cross(temp, z)); 
    Vector3 y = Normalize(Cross(z, x));         
    return x*v.x + y*v.y + z*v.z;
}