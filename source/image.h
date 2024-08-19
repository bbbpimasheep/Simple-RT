#pragma once
#ifndef IMAGE_H
#define IMAGE_H

// Disable strict warnings for this header from the Microsoft Visual C++ compiler.
#ifdef _MSC_VER
    #pragma warning (push, 0)
#endif

#define STB_IMAGE_IMPLEMENTATION
#define STBI_FAILURE_USERMSG
#include "../external/stb_image.h"
#include "global.h"
#include "mathematics.h"

#include <cstdlib>
#include <iostream>

class Image {
public:
    // Constructor
    Image() = default;
    Image(const char* _filename) {
        auto filename = std::string(_filename);
        auto imagedir = getenv("TEXTURE_IMAGES");
        // Hunt for the image file in some likely locations.
        if (imagedir && Load(std::string(imagedir) + "/" + filename)) return;
        if (Load(filename)) return;
        if (Load("images/" + filename)) return;
        if (Load("../images/" + filename)) return;

        std::cerr << "ERROR: Could not load image file '" << filename << "'.\n";
    }
    // Destructor
    ~Image() {
        delete[] b_data;
        STBI_FREE(f_data);
    }

    // Methods
    bool Load(const std::string& _filename) {
        auto origin_bpp = bytes_per_pixel;   // Dummy out parameter: original components per pixel
        f_data = stbi_loadf(_filename.c_str(), &image_width, &image_height, &origin_bpp, bytes_per_pixel);
        if (f_data == nullptr) return false;
        bytes_per_scanline = image_width * bytes_per_pixel;

        uint32_t total_bytes = image_width * image_height * bytes_per_pixel;
        b_data = new unsigned char[total_bytes];
        auto* b_ptr = b_data;
        auto* f_ptr = f_data;
        for (int i = 0; i < total_bytes; i+=1, b_ptr+=1, f_ptr+=1)
            *b_ptr = FloattoByte(*f_ptr);
        return true;
    }
    const unsigned char* PixelData(int x, int y) const { 
        static unsigned char magenta[] = { 255, 0, 255 };
        if (b_data == nullptr) return magenta;
        x = Interval(0, image_width -1).Clamp(x);
        y = Interval(0, image_height-1).Clamp(y);
        return b_data + bytes_per_scanline * y + bytes_per_pixel * x;
    }
    int Width()  const { return (f_data==nullptr) ? 0 : image_width; }
    int Height() const { return (f_data==nullptr) ? 0 : image_height; }

private:
    // Members
    const int      bytes_per_pixel = 3;
    float*         f_data = nullptr;    // Linear floating point pixel data
    unsigned char* b_data = nullptr;    // Linear 8-bit pixel data
    int            image_width = 0;
    int            image_height = 0;
    int            bytes_per_scanline = 0;

    // Methods
    static unsigned char FloattoByte(double _value) {
        if (_value <= 0.0)
            return 0;
        if (_value >= 1.0)
            return 255;
        return static_cast<unsigned char>(256.0 * _value);
    }
};


// Restore MSVC compiler warnings
#ifdef _MSC_VER
    #pragma warning (pop)
#endif

#endif // IMAGE_H