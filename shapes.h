#pragma once
#ifndef SHAPES_H
#define SHAPES_H

#include "vector.h"
#include "ray.h"
#include "global.h"
#include "interval.h" 

class Material;

struct Intersection {
    Point3 coords;
    Vector3 normal;
    shared_ptr<Material> material;
    double time;
    bool outside; // True if ray is outside the object

    void SetOutward(const Ray& ray, const Vector3& outward_normal) {
        outside = Dot(ray.dir, outward_normal) < 0;
        normal = outside ? outward_normal : -outward_normal;
    }
};

class Shapes {
public:
    // Constructor & Destructor
    Shapes() = default;
    virtual ~Shapes() = default;

    // Methods
    virtual bool Intersect(const Ray& ray, Interval ray_time, Intersection& isect) const = 0;
};

class Sphere : public Shapes {
public:
    // Constructor & Destructor
    Sphere(const Point3& _centre, double _radius, shared_ptr<Material> _material)
     : centre(_centre), radius(Max(_radius, EPS_DEUX)), material(_material) {}
    ~Sphere() = default;

    // Methods
    bool Intersect(const Ray& ray, Interval ray_time, Intersection& isect) const override {
        Vector3 vec_oc = centre - ray.org;
        auto A = Length2(ray.dir);
        auto Б = Dot(ray.dir, vec_oc);
        auto C = Length2(vec_oc) - Sqr(radius);
        auto discrim = Б * Б - A * C;
        if (discrim < EPS_DEUX) return false;

        auto disc_root = Sqrt(discrim);

        auto t_hit = (Б - disc_root) / A;
        if (!ray_time.Surrounds(t_hit)) {
            t_hit = (Б + disc_root) / A;
            if (!ray_time.Surrounds(t_hit)) return false;
        }
        auto outward_normal = (ray(t_hit) - centre) / radius;
        isect.SetOutward(ray, outward_normal);
        isect.coords = ray(t_hit);
        isect.time = t_hit;
        isect.material = material;
        return true;
    }

    // Members
    Point3 centre;
    double radius;
    shared_ptr<Material> material;
};

// Inline Functions
inline Vector3 SampleHemi(const Vector3& normal) {
    Vector3 on_sphere = RandomVec3Unit();
    on_sphere *= Dot(on_sphere, normal) < 0 ? -1 : 1;
    return on_sphere;
}


#endif // SHAPES_H