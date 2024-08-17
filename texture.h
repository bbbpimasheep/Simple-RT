#pragma once
#ifndef TEXTURE_H
#define TEXTURE_H

#include "global.h"
#include "mathematics.h"

class Texture {
public:
    // Destructor
    virtual ~Texture() = default;

    // Methods
    virtual Colour Value(double u, double v, const Point3& p) const = 0;
};

class SolidColour : public Texture {
public:
    // Constructor
    SolidColour(double r, double g, double b) : SolidColour(Colour(r, g, b)) {}
    SolidColour(const Colour& _albedo) : albedo(_albedo) {}

    // Methods
    Colour Value(double u, double v, const Point3& p) const override { return albedo; }

private:
    // Members
    Colour albedo;
};

class CheckerTexture : public Texture {
public:
    // Constructor
    CheckerTexture(double _scale, shared_ptr<Texture> _even_texture, shared_ptr<Texture> _odd_texture)
        : scale_inv(1.0/_scale), even_texture(_even_texture), odd_texture(_odd_texture) {}
    CheckerTexture(double _scale, const Colour& _even_colour, const Colour& _odd_colour)
        : CheckerTexture(_scale, make_shared<SolidColour>(_even_colour), make_shared<SolidColour>(_odd_colour)) {}

    // Methods
    Colour Value(double u, double v, const Point3& p) const override {
        auto x_int = static_cast<int>(p.x * scale_inv);
        auto y_int = static_cast<int>(p.y * scale_inv);
        auto z_int = static_cast<int>(p.z * scale_inv);
        bool is_even = (x_int + y_int + z_int) % 2 == 0;
        return is_even ? even_texture->Value(u, v, p) : odd_texture->Value(u, v, p);
    }

private:
    // Members
    double scale_inv;
    shared_ptr<Texture> even_texture;
    shared_ptr<Texture> odd_texture;
};


#endif // TEXTURE_H