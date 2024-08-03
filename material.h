#pragma once
#ifndef MATERIAL_H
#define MATERIAL_H

#include "global.h"
#include "ray.h"
#include "colour.h"
#include "shapes.h"

class Material {
public:
    // Deconstructor
    virtual ~Material() = default;

    // Methods
    virtual bool Scatter(const Ray& ray_in, const Intersection& isect, Colour& attenuation, Ray& scattered) 
    const { return false; }
};


class Lambertian : public Material {
public:
    // Constructor
    Lambertian(const Colour& _albedo) : albedo(_albedo) {}

    // Methods
    bool Scatter(const Ray& ray_in, const Intersection& isect, Colour& attenuation, Ray& scattered)
    const override {
        auto random_dir = isect.normal + RandomVec3Unit();
        Unitize(random_dir);
        scattered = Ray(isect.coords, random_dir);
        attenuation = albedo;
        return true;
    }

    // Members
    Colour albedo;
};

class Metal : public Material {
public:
    // Constructor
    Metal(const Colour& _albedo, double _fuzziness)
     : albedo(_albedo), fuzziness(_fuzziness < 1 ? _fuzziness : 1) {}

    // Methods
    bool Scatter(const Ray& ray_in, const Intersection& isect, Colour& attenuation, Ray& scattered)
    const override {
        auto reflect_dir = Reflect(-ray_in.dir, isect.normal) + RandomVec3Unit() * fuzziness;
        Unitize(reflect_dir);
        scattered = Ray(isect.coords, reflect_dir);
        attenuation = albedo;
        return Dot(scattered.dir, isect.normal) > 0;
    }

    // Members
    Colour albedo;
    double fuzziness;
};

class Dielectric : public Material {
public:
    // Constructor
    Dielectric(double _refractive_index) : refractive_index(_refractive_index) {}

    // Methods
    bool Scatter(const Ray& ray_in, const Intersection& isect, Colour& attenuation, Ray& scattered)
    const override {
        double eta = isect.outside ? refractive_index : (1.0 / refractive_index);
        attenuation = Colour(1.0, 1.0, 1.0);
        Vector3 transmit_dir;
        double cos_theta = Min(Dot(-ray_in.dir, isect.normal), 1.0);
        if (!Refract(-ray_in.dir, transmit_dir, isect.normal, eta) ||
            RandomFloat() < Fresnel(cos_theta, eta)) 
            scattered = Ray(isect.coords, Reflect(-ray_in.dir, isect.normal));
        else
            scattered = Ray(isect.coords, transmit_dir);
        return true;
    }

private:
    // Methods
    static double Fresnel(double cos_i, double eta) {
        // wi outward n // etat / etai
        auto sin2i = 1 - Sqr(cos_i);
        auto sin2r = sin2i / Sqr(eta);
        if (sin2r >= 1) return 1.0; // Total Internal Reflection
        auto cos_r = Sqrt(Max(EPS_QUAT, 1-sin2r));

        auto fr_parl = (eta * cos_i - cos_r) / (eta * cos_i + cos_r);
        auto fr_perp = (cos_i - eta * cos_r) / (cos_i + eta * cos_r);
        return (Sqr(fr_parl) + Sqr(fr_perp)) / 2;
    }

    // Members
    double refractive_index;
};

#endif // MATERIAL_H