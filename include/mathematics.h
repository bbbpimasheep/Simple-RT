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
    auto e = EPS_DEUX;                        
    auto p_vec = Cross(ray.dir, edge2);
    auto det = Dot(edge1, p_vec);
    auto inv_det = 1.0 / det;
    if (Abs(det) < EPS_QUAT) 
        return false;
    auto t_vec = ray.org - p0;
    u = Dot(t_vec, p_vec) * inv_det;
    if (u < 0-e || u > 1+e) 
        return false;
    auto q_vec = Cross(t_vec, edge1);
    v = Dot(ray.dir, q_vec) * inv_det;
    if (v < 0-e || u + v > 1+e) 
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