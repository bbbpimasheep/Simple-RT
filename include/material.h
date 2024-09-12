#pragma once
#ifndef MATERIAL_H
#define MATERIAL_H

#include "global.h"
#include "mathematics.h"
#include "shapes.h"
#include "objects.h"
#include "texture.h"

class Material {
public:
    // Deconstructor
    virtual ~Material() = default;

    // Methods
    virtual bool Scatter(const Ray& ray_in, const Intersection& isect, Colour& attenuation, Ray& scattered) const { return false; }
    virtual bool Shines() const { return false; }
    virtual bool Glossy() const { return false; }
    virtual bool Transmissive() const { return false; }
    virtual double EtaIndex() const { return 1.0; }
    virtual Colour Emission(double u, double v, const Point3& p) const { return Colour(0.0); }
    virtual double PDFunc(const Vector3& wi, const Vector3& wo, const Vector3& normal) const { return 0.0; }
    virtual Colour BRDFunc(const Vector3& wi, const Vector3& wo, const Vector3& normal,
                           const Colour& attenuation) const { return Colour(0.0); }

protected:
    // Methods
    static double Fresnel(double cos_i, double eta) {
        if (cos_i < 0) {    // Potentially flip interface orientation for Fresnel equations
            cos_i = -cos_i; eta = 1 / eta;
        }
        // wi outward n // etat / etai
        auto sin2_i = 1 - Sqr(cos_i);
        auto sin2_t = sin2_i / Sqr(eta);
        if (sin2_t >= 1) return 1.0; // Total Internal Reflection
        auto cos_t = Sqrt(Max(EPS_QUAT, 1-sin2_t));

        auto fr_parl = (eta * cos_i - cos_t) / (eta * cos_i + cos_t);
        auto fr_perp = (cos_i - eta * cos_t) / (cos_i + eta * cos_t);
        return (Sqr(fr_parl) + Sqr(fr_perp)) / 2;
    }
    static Vector3 FresnelSchlick(double cos_theta, const Vector3& f0) {
        return f0 + (Vector3(1) - f0) * Pow(1.0 - cos_theta, 5.0);
    }
};


class Lambertian : public Material {
public:
    // Constructor
    Lambertian(const Colour& _albedo) : texture(make_shared<SolidColour>(_albedo)) {}
    Lambertian(shared_ptr<Texture> _texture) : texture(_texture) {}

    // Methods
    bool Scatter(const Ray& ray_in, const Intersection& isect, Colour& attenuation, Ray& scattered)
    const override {
        auto random_dir = SampleScatter(-ray_in.dir, isect.normal);
        scattered = Ray(isect.coords, random_dir, ray_in.time);
        attenuation = texture->Value(isect.u, isect.v, isect.coords);
        return true;
    }
    double PDFunc(const Vector3& wi, const Vector3& wo, const Vector3& normal) const override {
        if (Dot(wi, normal) < 0) 
            return 0.0;
        return double(0.5 / M_PI);
    }
    Colour BRDFunc(const Vector3& wi, const Vector3& wo, const Vector3& normal,
                   const Colour& attenuation) const override {
        if (Dot(wi, normal) < 0)
            return Colour(0.0);
        return attenuation / M_PI;
    }
    Vector3 SampleScatter(const Vector3& wi, const Vector3& normal) const {
        return SampleU_Hemisphere(normal);
    }

    // Members
    shared_ptr<Texture> texture;
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
        scattered = Ray(isect.coords, reflect_dir, ray_in.time);
        attenuation = albedo;
        return Dot(scattered.dir, isect.normal) > 0;
    }
    bool Glossy() const override { return true; }

    // Members
    Colour albedo;
    double fuzziness;
};


class Light : public Material {
public:
    // Constructor
    Light(shared_ptr<Texture> _texture) : texture(_texture) {}
    Light(const Colour& _colour) : texture(make_shared<SolidColour>(_colour)) {}

    // Methods
    Colour Emission(double u, double v, const Point3& p) const override {
        return texture->Value(u, v, p);
    }
    bool Shines() const override { return true; }

private:
    // Members
    shared_ptr<Texture> texture;
};


#endif // MATERIAL_H