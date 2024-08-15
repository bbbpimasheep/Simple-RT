#pragma once
#ifndef SHAPES_H
#define SHAPES_H

#include "global.h"
#include "bounds.h"
#include "ray.h"
#include "maths.h"

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
    virtual Bounds3 BBox() const = 0;
};

class Sphere : public Shapes {
public:
    // Constructor & Destructor
    // Staionary Sphere
    Sphere(const Point3& _centre, double _radius, shared_ptr<Material> _material)
     : centre0(_centre), radius(Max(_radius, EPS_DEUX)), material(_material), moving(false) {
        auto r_vec = Vector3(radius, radius, radius);
        bbox = Bounds3(centre0-r_vec, centre0+r_vec);
    }
    // Moving Sphere
    Sphere(const Point3& _centre1, const Point3& _centre2, double _radius, shared_ptr<Material> _material)
     : centre0(_centre1), radius(Max(_radius, EPS_DEUX)), material(_material), moving(true) { 
        auto r_vec = Vector3(radius, radius, radius);
        auto bbox1 = Bounds3(_centre1-r_vec, _centre1+r_vec);
        auto bbox2 = Bounds3(_centre2-r_vec, _centre2+r_vec);
        bbox = Union(bbox1, bbox2);
        shift = _centre2 - _centre1; 
    }
    ~Sphere() = default;

    // Methods
    bool Intersect(const Ray& ray, Interval ray_time, Intersection& isect) const override {
        Point3 centre = moving ? GetCentre(ray.time) : centre0;
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
    Bounds3 BBox() const override {
        return bbox;
    }

private:
    // Members
    Point3 centre0;
    double radius;
    bool   moving;
    Vector3 shift;
    Bounds3 bbox;
    shared_ptr<Material> material;

    // Methods
    Point3 GetCentre(double time) const { return centre0 + time * shift; }
};

// Inline Functions
inline Vector3 SampleHemi(const Vector3& normal) {
    Vector3 on_sphere = RandomVec3Unit();
    on_sphere *= Dot(on_sphere, normal) < 0 ? -1 : 1;
    return on_sphere;
}


#endif // SHAPES_H