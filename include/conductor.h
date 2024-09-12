#pragma once
#ifndef CONDUCTOR_H
#define CONDUCTOR_H

#include "material.h"

// Conductor Material

class MicroFacet : public Material {
public:
    // Constructor
    MicroFacet(const Colour& base_fresnel, double _x, double _y, double _metalness)
     : alpha_x(_x), alpha_y(_y), metalness(_metalness) { 
        r0 = Lerp(Colour(0.4), base_fresnel, metalness); 
        texture = make_shared<SolidColour>(r0);
    }
    MicroFacet(shared_ptr<Texture> _texture, const Colour& base_fresnel, double _x, double _y, double _metalness)
     : texture(_texture), alpha_x(_x), alpha_y(_y), metalness(_metalness) 
    { r0 = Lerp(Colour(0.4), base_fresnel, metalness); }

    // Methods
    bool Glossy() const override { return Max(alpha_x, alpha_y) < 0.1; }
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

class Specular : public Material {
public:
    // Constructor
    Specular(const Colour& base_fresnel, double _metalness) : metalness(_metalness) {
        r0 = Lerp(Colour(0.4), base_fresnel, metalness); 
        texture = make_shared<SolidColour>(r0);
    }
    Specular(shared_ptr<Texture> _texture, const Colour& base_fresnel, double _metalness)
     : texture(_texture), metalness(_metalness)
    { r0 = Lerp(Colour(0.4), base_fresnel, metalness); }

    // Methods
    bool Glossy() const override { return true; }
    bool Scatter(const Ray& ray_in, const Intersection& isect, Colour& attenuation, Ray& scattered)
    const override {
        auto wi = Reflect(-ray_in.dir, isect.normal);
        if (Dot(wi, isect.normal) < 0)
            return false;
        scattered = Ray(isect.coords, wi, ray_in.time);
        attenuation = texture->Value(isect.u, isect.v, isect.coords);
        return true;
    }
    double PDFunc(const Vector3& wi, const Vector3& wo, const Vector3& normal) const override { return 1.0; }
    Colour BRDFunc(const Vector3& _wi, const Vector3& _wo, const Vector3& normal, const Colour& attenuation) const override {
        auto wi = ToLocal(normal, _wi);
        auto fresnel = FresnelSchlick(Abs(CosΘ(wi)), r0);
        return fresnel / Abs(CosΘ(wi));
    }

    // Members
    shared_ptr<Texture> texture;
    double metalness;
    Colour r0;
};

class Conductor : public Material {
public:
    // Constructor
    Conductor(const Colour& base_fresnel, double _x, double _y, double _metalness) {
        super_glossy = SuperGlossy(_x, _y);
        if (!super_glossy) 
            micr = make_shared<MicroFacet>(base_fresnel, _x, _y, _metalness);
        else 
            spec = make_shared<Specular>(base_fresnel, _metalness);
    }
    Conductor(shared_ptr<Texture> _texture, const Colour& base_fresnel, double _x, double _y, double _metalness) {
        super_glossy = SuperGlossy(_x, _y);
        if (!super_glossy) 
            micr = make_shared<MicroFacet>(_texture, base_fresnel, _x, _y, _metalness);
        else 
            spec = make_shared<Specular>(_texture, base_fresnel, _metalness);
    }

    // Methods
    bool Glossy() const override { 
        if (super_glossy) 
            return spec->Glossy();
        else
            return micr->Glossy();
    }
    bool Scatter(const Ray& ray_in, const Intersection& isect, Colour& attenuation, Ray& scattered) const override {
        if (!isect.outside) return false;
        if (super_glossy) 
            return spec->Scatter(ray_in, isect, attenuation, scattered);
        else 
            return micr->Scatter(ray_in, isect, attenuation, scattered);
    }
    Colour BRDFunc(const Vector3& _wi, const Vector3& _wo, const Vector3& normal, const Colour& attenuation) const override {
        if (super_glossy) 
            return spec->BRDFunc(_wi, _wo, normal, attenuation);
        else 
            return micr->BRDFunc(_wi, _wo, normal, attenuation);
    }
    double PDFunc(const Vector3& wi, const Vector3& wo, const Vector3& normal) const override {
        if (super_glossy) 
            return spec->PDFunc(wi, wo, normal);
        else 
            return micr->PDFunc(wi, wo, normal);
    }

private:
    // Members
    shared_ptr<MicroFacet> micr;
    shared_ptr<Specular> spec;
    bool super_glossy;

    // Methods
    bool SuperGlossy(double x, double y) { return Max(x, y) <= .001; }
};


#endif // CONDUCTOR_H