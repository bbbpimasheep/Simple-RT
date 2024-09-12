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
        auto hpos = isect.coords;
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
        scattered = Ray(hpos, ToWorld(isect.normal, wi), ray_in.time);
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

class Dielectric : public Material {
public:
    // Constructor
    Dielectric(double refractive_index) : _eta(refractive_index) {}

    // Methods
    bool Scatter(const Ray& ray_in, const Intersection& isect, Colour& attenuation, Ray& scattered)
    const override {
        double eta = isect.outside ? _eta : (1.0 / _eta);
        attenuation = Colour(1.0, 1.0, 1.0);
        Vector3 transmit_dir;
        double cos_theta = Min(Dot(-ray_in.dir, isect.normal), 1.0);
        auto etap = eta;
        if (!Refract(-ray_in.dir, transmit_dir, isect.normal, eta, etap) ||
            RandomFloat() < Fresnel(cos_theta, eta)) 
            scattered = Ray(isect.coords, Reflect(-ray_in.dir, isect.normal), ray_in.time);
        else
            scattered = Ray(isect.coords, transmit_dir, ray_in.time);
        return true;
    }

private:
    // Members
    double _eta;
};


#endif // DIELECTRIC_H