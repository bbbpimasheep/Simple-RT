#pragma once
#ifndef SHAPES_H
#define SHAPES_H

#include "global.h"
#include "bounds.h"
#include "mathematics.h"

class Material;
class Shapes;

struct Intersection {
    Point3 coords;
    Vector3 normal;
    double time;
    double u, v;
    bool outside; // True if ray is outside the object
    shared_ptr<Material> material;
    shared_ptr<const Shapes> object;

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
    virtual bool Shines() const { return false; };
    virtual Bounds3 BBox() const = 0;
    virtual double Area() const = 0;
    virtual void Sample(Intersection& isect, double& pdf) const {};
};

class Sphere : public Shapes, public std::enable_shared_from_this<Sphere> {
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
    double Area() const override { return 4 * M_PI * Sqr(radius); }
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
        isect.object = shared_from_this();
        CountUV(outward_normal, isect.u, isect.v);
        
        return true;
    }
    void Sample(Intersection& isect, double& pdf) const override {
        auto unit_vec = RandomVec3Unit();
        auto sample_point = centre0 + radius * unit_vec;
        isect.coords = sample_point;
        isect.time = 0.0;
        isect.material = material;
        isect.SetOutward(Ray(sample_point, unit_vec), unit_vec);
        pdf = 1.0 / (4*M_PI * Sqr(radius));
    }
    bool Shines() const override;

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


class Quad: public Shapes, public std::enable_shared_from_this<Quad> {
public:
    // Constructors
    Quad(const Point3& _vertex, const Vector3& _u, const Vector3& _v, shared_ptr<Material> _material) 
         : vert0(_vertex), vec_u(_u), vec_v(_v), material(_material), transform(Transform()) { 
        // Compute the normal and constant of the plane equation
        normal = Normalize(Cross(vec_u, vec_v)); 
        back_culling = false;
        CountBBox(); 
    }
    Quad(const Point3& _vertex, const Vector3& _u, const Vector3& _v, shared_ptr<Material> _material,
         const Transform& _transform) 
         : transform(_transform), 
           vert0(_transform.Apply(Homogeneous(_vertex, 1.0))), 
           vec_u(_transform.Apply(Homogeneous(_u, 0.0))), 
           vec_v(_transform.Apply(Homogeneous(_v, 0.0))), 
           material(_material) { 
        // Compute the normal and constant of the plane equation
        normal = Normalize(Cross(vec_u, vec_v)); 
        back_culling = false;
        CountBBox(); 
    }
    Quad(const Point3& _vertex, const Vector3& _u, const Vector3& _v, shared_ptr<Material> _material,
         const Vector3& _normal, const Transform& _transform) 
         : transform(_transform), 
           vert0(_transform.Apply(Homogeneous(_vertex, 1.0))), 
           vec_u(_transform.Apply(Homogeneous(_u, 0.0))), 
           vec_v(_transform.Apply(Homogeneous(_v, 0.0))), 
           material(_material) { 
        // Compute the normal and constant of the plane equation
        normal = _transform.Apply(Homogeneous(_normal, 0.0)); 
        normal = Normalize(normal); 
        back_culling = true;
        CountBBox(); 
    }

    // Methods
    virtual void CountBBox() {
        // Compute the bounding box of all four vertices.
        auto bbox_diagonal1 = Bounds3(vert0, vert0 + vec_u + vec_v);
        auto bbox_diagonal2 = Bounds3(vert0 + vec_u, vert0 + vec_v);
        bbox = Union(bbox_diagonal1, bbox_diagonal2);
    }
    Bounds3 BBox() const override { return bbox; }
    double Area() const override { return Length(Cross(vec_u, vec_v)); }
    bool Intersect(const Ray& ray, Interval ray_time, Intersection& isect) const override {
        if (back_culling && Dot(normal, ray.dir) > 0)
            return false;
        auto vert1 = vert0 + vec_u;
        auto vert2 = vert0 + vec_v;
        auto vert3 = vert0 + vec_u + vec_v;
        auto u=.0, v=.0, t=.0;
        if (TriangleIsect(vec_u, vec_v, vert0, ray, t, u, v)) {
            isect.u = u; isect.v = v;
        } else if (TriangleIsect(-vec_u, -vec_v, vert3, ray, t, u, v)) {
            isect.u = 1 - u; isect.v = 1 - v;
        } else return false;

        if (!ray_time.Contains(t)) return false;
        isect.coords = ray(t);
        isect.time = t;
        isect.material = material;
        isect.object = shared_from_this();
        if (back_culling) 
            isect.normal = normal;
        else 
            isect.SetOutward(ray, normal);
        return true;
    }
    void Sample(Intersection& isect, double& pdf) const override {
        auto u = RandomFloat(), v = RandomFloat();
        auto p = vert0 + u*vec_u + v*vec_v;
        isect.coords = p;
        isect.time = 0.0;
        isect.material = material;
        if (back_culling) 
            isect.normal = normal;
        pdf = 1.0 / Area();
    }
    bool Shines() const override;

private:
    // Members
    Point3 vert0;
    Vector3 vec_u, vec_v;
    Vector3 normal;
    Bounds3 bbox;
    Transform transform;
    bool back_culling;
    shared_ptr<Material> material;
};


#endif // SHAPES_H