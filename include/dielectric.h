#pragma once
#ifndef DIELECTRIC_H
#define DIELECTRIC_H

#include "material.h"

// Dielectric Material

enum class ScatterType {
    Reflection,     // Refraction
    Transmission,   // Transmission
};

class SpecularDielectric : public Material {
public:
    // Constructor
    SpecularDielectric(double refract_index) : eta(refract_index), etap(refract_index) {}

    // Methods
    bool Glossy() const override { return true; }
    bool Transmissive() const override { return type == ScatterType::Transmission; }
    double EtaIndex() const override { return etap; }
    bool Scatter(const Ray& ray_in, const Intersection& isect, Colour& attenuation, Ray& scattered) const override {
        auto wo = ToLocal(isect.normal, -ray_in.dir);
        auto wi = Vector3(0);
        auto pr = Fresnel(CosΘ(wo), eta), pt = 1 - pr;
        // if (pr < 1) {
        //     pr += Sqr(pt) * pr / (1 - Sqr(pr));
        //     pt = 1 - pr;
        // }
        reflect  = Min(pr / (pr + pt), 1-EPS_UNIT); 
        transmit = Min(pt / (pr + pt), 1-EPS_UNIT);

        if (RandomFloat() <= reflect) {
            type = ScatterType::Reflection;
            wi = Vector3(-wo.x,-wo.y, wo.z);
        } else {
            type = ScatterType::Transmission;
            if (!Refract(wo, wi, Vector3(0,0,1), eta, etap)) 
                return false;
        }
        scattered = Ray(isect.coords, ToWorld(isect.normal, wi), ray_in.time);
        attenuation = Colour(1);
        return true;
    }
    double PDFunc(const Vector3& wi, const Vector3& wo, const Vector3& normal) const override {
        if (type == ScatterType::Reflection)   return reflect;
        if (type == ScatterType::Transmission) return transmit;
        return 1;
    }
    Colour BRDFunc(const Vector3& wi, const Vector3& wo, const Vector3& normal, const Colour& attenuation) const override {
        if (type == ScatterType::Reflection)   return (reflect  / Abs(Dot(wi, normal))) * Colour(1);
        if (type == ScatterType::Transmission) return (transmit / Abs(Dot(wi, normal))) * Colour(1) / Sqr(etap);
        return Colour(1);
    }

private:
    // Members
    mutable ScatterType type;
    double eta;
    mutable double etap;
    mutable double reflect;
    mutable double transmit;
};


class RoughDielectric : public Material {
public:
    // Constructor
    RoughDielectric(double refractive_index, double _x, double _y) : eta(refractive_index), alpha_x(_x), alpha_y(_y) {}

    // Methods
    bool Glossy() const override { return true; }
    bool Transmissive() const override { return type == ScatterType::Transmission; }
    double EtaIndex() const override { return etap; }
    bool Scatter(const Ray& ray_in, const Intersection& isect, Colour& attenuation, Ray& scattered) const override {
        auto wo = ToLocal(isect.normal, -ray_in.dir);
        auto wi = Vector3(0);
        auto wm = SampleHalfVector(wo);
        auto pr = Fresnel(CosΘ(wo), eta), pt = 1 - pr;
        // pt = 0;
        reflect  = Min(pr / (pr + pt), 1-EPS_UNIT); 
        transmit = Min(pt / (pr + pt), 1-EPS_UNIT);

        if (RandomFloat() <= reflect) {
            type = ScatterType::Reflection;
            wi = Reflect(wo, wm);
            if (wo.z * wi.z < 0) 
                return false;
        } else {
            type = ScatterType::Transmission;
            if (!Refract(wo, wi, wm, eta, etap) || wi.z == 0 || wo.z * wi.z >= 0) 
                return false;
        }
        scattered = Ray(isect.coords, ToWorld(isect.normal, wi), ray_in.time);
        attenuation = Colour(1);
        return true;
    }
    double PDFunc(const Vector3& _wi, const Vector3& _wo, const Vector3& normal) const override {
        auto wi = ToLocal(normal, _wi);
        auto wo = ToLocal(normal, _wo);
        auto cos_theta_o = Abs(CosΘ(wo));
        auto cos_theta_i = Abs(CosΘ(wi));
        // auto is_reflect = Dot(wi, wo) > 0;
        if (type == ScatterType::Transmission) 
            etap = cos_theta_o > 0 ? eta : (1.0 / eta);
        auto wm = wi * etap + wo;
        if (cos_theta_i == 0 || cos_theta_o == 0 || Length2(wm) == 0)
            return 0;
        wm = wm.z > 0 ? wm : -wm;
        wm = Normalize(wm);

        auto pdf_wm = MaskingFunc(wi) / Abs(CosΘ(wi)) * 
                      DistributionGGX(wm) * Abs(Dot(wi, wm));
        if (type == ScatterType::Reflection) {
            if (wm.z < 0) 
                wm *= -1;
            auto pdf_wi = pdf_wm / (4.0 * Abs(Dot(wm, wo))) * reflect;
            return pdf_wi;
        }
        if (type == ScatterType::Transmission) {
            auto denom   = Sqr(Dot(wi, wm) + Dot(wo, wm) / etap);
            auto dwm_dwi = Abs(Dot(wi, wm)) / denom;
            auto pdf_wi  = pdf_wm * dwm_dwi * transmit;
            return pdf_wi;
        }
        return 1;
    }
    Colour BRDFunc(const Vector3& _wi, const Vector3& _wo, const Vector3& normal, const Colour& attenuation) const override {
        auto wi = ToLocal(normal, _wi);
        auto wo = ToLocal(normal, _wo);
        auto cos_theta_o = Abs(CosΘ(wo));
        auto cos_theta_i = Abs(CosΘ(wi));
        auto is_reflect = Dot(wi, wo) < 0;
        if (!is_reflect) 
            etap = cos_theta_o > 0 ? eta : (1.0 / eta);
        auto wm = wi * etap + wo;
        if (cos_theta_i == 0 || cos_theta_o == 0 || Length2(wm) == 0)
            return 0;
        wm.z *= wm.z > 0 ? 1 : -1;
        wm = Normalize(wm);

        auto fresnel = Fresnel(cos_theta_o, eta);
        auto dis_ggx = DistributionGGX(wm);
        auto shading = ShadingSmith(wi, wo);
        if (type == ScatterType::Reflection) {
            if (cos_theta_i == 0 || cos_theta_o == 0) 
                return Colour(0.0);
            return fresnel  * dis_ggx * shading / (4.0 * cos_theta_i * cos_theta_o);
        }
        if (type == ScatterType::Transmission) {
            auto denom = Sqr(Dot(wi, wm) + Dot(wo, wm) / etap) * cos_theta_i * cos_theta_o;
            return transmit * dis_ggx * shading * Abs(Dot(wi, wm) * Dot(wo, wm) / denom) / Sqr(etap);
        }
        return Colour(1);
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
    double MaskingFunc (Vector3& _w) const { return 1.0 / (1 + Lambda(_w)); }
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
    mutable ScatterType type;
    double eta;
    mutable double etap;
    double alpha_x, alpha_y;
    mutable double reflect;
    mutable double transmit;
};


class Dielectric : public Material {
public:
    // Constructor
    Dielectric(double refractive_index, double _x, double _y) {
        super_glossy = SuperGlossy(_x, _y);
        if (super_glossy)
            specl = make_shared<SpecularDielectric>(refractive_index);
        else
            rough = make_shared<RoughDielectric>(refractive_index, _x, _y);
    }

    // Methods
    bool Glossy() const override { 
        if (super_glossy)
            return specl->Glossy();
        else
            return rough->Glossy();
    }
    bool Scatter(const Ray& ray_in, const Intersection& isect, Colour& attenuation, Ray& scattered) const override {
        if (super_glossy)
            return specl->Scatter(ray_in, isect, attenuation, scattered);
        else
            return rough->Scatter(ray_in, isect, attenuation, scattered);
    }
    Colour BRDFunc(const Vector3& _wi, const Vector3& _wo, const Vector3& normal, const Colour& attenuation) const override {
        if (super_glossy)
            return specl->BRDFunc(_wi, _wo, normal, attenuation);
        else
            return rough->BRDFunc(_wi, _wo, normal, attenuation);
    }
    double PDFunc(const Vector3& _wi, const Vector3& _wo, const Vector3& normal) const override {
        if (super_glossy)
            return specl->PDFunc(_wi, _wo, normal);
        else
            return rough->PDFunc(_wi, _wo, normal);
    }
    double EtaIndex() const override {
        if (super_glossy)
            return specl->EtaIndex();
        else
            return rough->EtaIndex();
    }

private:
    // Methods
    bool SuperGlossy(double x, double y) { return Max(x, y) <= .001; }

    // Members
    double _eta;
    shared_ptr<SpecularDielectric> specl;
    shared_ptr<RoughDielectric> rough;
    bool super_glossy;
};


#endif // DIELECTRIC_H