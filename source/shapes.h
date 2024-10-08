#pragma once
#ifndef SHAPES_H
#define SHAPES_H

#include "global.h"
#include "bounds.h"
#include "ray.h"
#include "mathematics.h"

class Material;

struct Intersection {
    Point3 coords;
    Vector3 normal;
    shared_ptr<Material> material;
    double time;
    double u, v;
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
    Bounds3 BBox() const override { return bbox; }
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
        CountUV(outward_normal, isect.u, isect.v);
        
        return true;
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
    static void CountUV(const Point3& p, double& u, double& v) {
        auto theta = Acos(-p.y);
        auto phi   = Atan2(-p.z, p.x) + M_PI;
        u = phi / (2 * M_PI);
        v = theta / M_PI;
    }
};


class Quad: public Shapes {
public:
    // Constructors
    Quad(const Point3& _pin, const Vector3& _u, const Vector3& _v, shared_ptr<Material> _material) 
         : pin(_pin), vec_u(_u), vec_v(_v), material(_material), transform(Transform()) { 
        // Compute the normal and constant of the plane equation
        auto n = Cross(vec_u, vec_v);
        normal = Normalize(n); 
        constant = Dot(pin, normal);
        vec_w = n / Dot(n, n);
        CountBBox(); 
    }
    Quad(const Point3& _pin, const Vector3& _u, const Vector3& _v, shared_ptr<Material> _material,
         const Transform& _transform) 
         : transform(_transform), 
           pin(_transform.Apply(Homogeneous(_pin, 1.0))), 
           vec_u(_transform.Apply(Homogeneous(_u, 0.0))), 
           vec_v(_transform.Apply(Homogeneous(_v, 0.0))), 
           material(_material) { 
        // Compute the normal and constant of the plane equation
        auto n = Cross(vec_u, vec_v);
        normal = Normalize(n); 
        constant = Dot(pin, normal);
        vec_w = n / Dot(n, n);
        CountBBox(); 
    }

    // Methods
    virtual void CountBBox() {
        // Compute the bounding box of all four vertices.
        auto bbox_diagonal1 = Bounds3(pin, pin + vec_u + vec_v);
        auto bbox_diagonal2 = Bounds3(pin + vec_u, pin + vec_v);
        bbox = Union(bbox_diagonal1, bbox_diagonal2);
    }
    Bounds3 BBox() const override { return bbox; }
    bool Intersect(const Ray& ray, Interval ray_time, Intersection& isect) const override {
        auto denominator = Dot(ray.dir, normal);
        if (Abs(denominator) < EPS_DEUX) 
            return false;
        auto t = (constant - Dot(ray.org, normal)) / denominator;
        if (!ray_time.Contains(t)) 
            return false;
        auto isect_to_pin = ray(t) - pin;
        auto alpha = Dot(vec_w, Cross(isect_to_pin, vec_v));
        auto beta  = Dot(vec_w, Cross(vec_u, isect_to_pin));
        if (!Interior(alpha, beta, isect))
            return false;

        isect.coords = ray(t);
        isect.time = t;
        isect.material = material;
        isect.SetOutward(ray, normal);
        return true;
    }
    virtual bool Interior(double _a, double _b, Intersection& isect) const {
        Interval unit_interval = Interval(0, 1);
        if (!unit_interval.Contains(_a) || !unit_interval.Contains(_b))
            return false;
        isect.u = _a;
        isect.v = _b;
        return true;
    }

private:
    // Members
    Point3 pin;
    Vector3 vec_u, vec_v, vec_w, normal;
    shared_ptr<Material> material;
    Bounds3 bbox;
    double constant;
    Transform transform;
};


// Inline Functions
inline Vector3 SampleHemi(const Vector3& normal) {
    Vector3 on_sphere = RandomVec3Unit();
    on_sphere *= Dot(on_sphere, normal) < 0 ? -1 : 1;
    return on_sphere;
}


#endif // SHAPES_H