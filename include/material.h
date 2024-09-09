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
    virtual bool SuperGlossy() const { return false; }
    virtual Colour Emission(double u, double v, const Point3& p) const { return Colour(0.0); }
    virtual double PDFunc(const Vector3& wi, const Vector3& wo, const Vector3& normal) const { return 0.0; }
    virtual Colour BRDFunc(const Vector3& wi, const Vector3& wo, const Vector3& normal,
                           const Colour& attenuation) const { return Colour(0.0); }

protected:
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
    bool SuperGlossy() const override { return true; }

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
            scattered = Ray(isect.coords, Reflect(-ray_in.dir, isect.normal), ray_in.time);
        else
            scattered = Ray(isect.coords, transmit_dir, ray_in.time);
        return true;
    }

private:
    // Members
    double refractive_index;
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


class MicroFacet : public Material {
public:
    // Constructor
    MicroFacet(const Colour& base_reflect, double _x, double _y, double _metalness)
     : alpha_x(_x), alpha_y(_y), metalness(_metalness) { 
        r0 = Lerp(Colour(0.4), base_reflect, metalness); 
        texture = make_shared<SolidColour>(r0);
    }
    MicroFacet(shared_ptr<Texture> _texture, const Colour& base_reflect, double _x, double _y, double _metalness)
     : texture(_texture), alpha_x(_x), alpha_y(_y), metalness(_metalness) 
    { r0 = Lerp(Colour(0.4), base_reflect, metalness); }

    // Methods
    Colour BRDFunc(const Vector3& _wi, const Vector3& _wo, const Vector3& normal,
                    const Colour& attenuation) const override {
        auto wi = ToLocal(normal, _wi);
        auto wo = ToLocal(normal, _wo);
        auto wm = Normalize(wi + wo);
        auto cos_theta_o = Abs(CosΘ(wo));
        auto cos_theta_i = Abs(CosΘ(wi));
        if (cos_theta_i == 0 || cos_theta_o == 0) 
            return Colour(0.0);
        auto fresnel = FresnelSchlick(Abs(Dot(wi, wm)), r0);
        auto dis_ggx = DistributionGGX(wm);
        auto shading = ShadingSmith(wi, wo);
        return fresnel * dis_ggx * shading / (4.0 * cos_theta_i * cos_theta_o);
    }
    double PDFunc(const Vector3& wi, const Vector3& wo, const Vector3& normal) const override {
        auto wi_local = ToLocal(normal, wi);
        auto wo_local = ToLocal(normal, wo);
        auto wm_local = wi_local + wo_local;
        if (Length2(wm_local) == 0) 
            return 0;
        wm_local = Normalize(wm_local);
        if (wm_local.z < 0) 
            wm_local *= -1;
        auto pdf_wm = MaskingFunc(wi_local) / Abs(CosΘ(wi_local)) * 
                      DistributionGGX(wm_local) * 
                      Abs(Dot(wi_local, wm_local));
        return pdf_wm / (4.0 * Abs(Dot(wm_local, wo_local)));
    }
    bool Scatter(const Ray& ray_in, const Intersection& isect, Colour& attenuation, Ray& scattered) const override {
        auto wo = ToLocal(isect.normal, -ray_in.dir);
        auto wm = SampleHalfVector(wo);
        auto wi = Reflect(wo, wm);
        if (wo.z * wi.z < 0) 
            return false;
        scattered = Ray(isect.coords, ToWorld(isect.normal, wi), ray_in.time);
        attenuation = texture->Value(isect.u, isect.v, isect.coords);
        return true;
    }
    bool Glossy() const override { return Max(alpha_x, alpha_y) < 0.1; }
    bool SuperGlossy() const override { return Max(alpha_x, alpha_y) < .001; }

private:
    // Methods
    double DistributionGGX(Vector3& _wm) const {     // GGX Distribution
        auto tan2_theta = Tan2Θ(_wm);
        if (std::isinf(tan2_theta)) return 0;
        auto cos4_theta = Sqr(Cos2Θ(_wm));
        auto product = tan2_theta * (Sqr(Cosφ(_wm) / alpha_x) +
                                     Sqr(Sinφ(_wm) / alpha_y));
        return 1.0 / (M_PI * alpha_x * alpha_y * cos4_theta * Sqr(1 + product));
    }
    double Lambda(Vector3& _dir) const {
        auto tan2_theta = Tan2Θ(_dir);
        if (std::isinf(tan2_theta)) return 0;
        auto alpha2 = Sqr(alpha_x * Cosφ(_dir)) + Sqr(alpha_y * Sinφ(_dir));
        return (Sqrt(1 + alpha2 * tan2_theta) - 1) / 2.0;
    }
    double ShadingSmith(Vector3& wi, Vector3& wo) const { return 1.0 / (1.0 + Lambda(wi) + Lambda(wo)); }
    double MaskingFunc(Vector3& _w) const { return 1.0 / (1 + Lambda(_w)); }
    Vector3 SampleHalfVector(const Vector3& wo) const {
        auto wh = Normalize(Vector3(alpha_x * wo.x, alpha_y * wo.y, wo.z));
        if (wh.z < 0) wh *= -1;
        auto t1 = (wh.z <= (1-EPS_DEUX)) ? Normalize(Cross(Vector3(0, 0, 1), wh)) : Vector3(1, 0, 0);
        auto t2 = Cross(wh, t1);

        Point3 point = RandomVec3DiskPolar();
        point.y = Lerp(Sqrt(1 - Sqr(point.x)), point.y, (1 + wh.z) / 2);
        point.z = Sqrt(Max(0.0, 1 - Sqr(point.x) - Sqr(point.y)));
        auto nh = point.x * t1 + point.y * t2 + point.z * wh;
        auto wm = Normalize(Vector3(nh.x*alpha_x, nh.y*alpha_y, Max(EPS_DEUX, nh.z)));
        return wm;
    }

    // Members
    shared_ptr<Texture> texture;
    double alpha_x, alpha_y;
    double metalness;
    Colour r0;
};


#endif // MATERIAL_H