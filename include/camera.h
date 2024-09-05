#pragma once
#ifndef CAMERA_H
#define CAMERA_H

#include <omp.h>

#include "global.h"
#include "shapes.h"
#include "mathematics.h"
#include "material.h"

class Camera {
public:
    // Methods
    void RenderScene(const Shapes& scene) {
        InitializeCamera();
        
        std::clog << "Rendering Scene... \n";

        std::vector<Colour> frame_buffer(image_width * image_height);
        int progress = 0;
        #pragma omp parallel for shared(progress)
        for (int y = 0; y < image_height; y += 1) {
            #pragma omp parallel for
            for (int x = 0; x < image_width; x += 1) {
                auto pixel_colour = Colour(0, 0, 0);
                for (int s = 0; s < sample_ppixel; s += 1) {
                    Ray ray = CastRay(x, y, s);
                    // std::clog << "Rendering pixel (" << x << ", " << y << ") sample " << s << " \n";
                    pixel_colour += RayColour(ray, scene, max_depth);
                    // std::clog << "Pixel Colour: " << Str(pixel_colour) << "\n";
                }
                frame_buffer[y * image_width + x] = pixel_colour * spp_inv;
            }
            #pragma omp critical
            ProgressBar(double(progress)/image_height);
            progress += 1;
        }
        ProgressBar(1.0); 
        
        std::clog << "\nRendering Complete! \n";
        std::clog << "Drawing Frame Buffer... \n";

        std::cout << "P3\n" << image_width << " " << image_height << "\n255\n";
        for (int i = 0; i < image_width * image_height; i += 1)
            WriteColour(frame_buffer[i], std::cout);
    }

    // Members
    int image_width     = 1024;
    int sample_ppixel   = 16;
    int max_depth       = 32;
    double roulette     = 0.8;
    double aspect_ratio = 1.0;
    double verticle_fov = 90.0;
    Colour background   = Colour(0.0);

    Vector3 view_up = Vector3(0, 1, 0);
    Point3 view_des = Point3(0, 0,-1);
    Point3 view_pos = Point3(0, 0, 0);

    double focal_dist     = 10.0;
    double defocus_angle  = 0.0;

private:
    // Methods
    void InitializeCamera() {
        image_height = int(image_width / aspect_ratio);
        image_height = image_height < 1 ? 1 : image_height;

        camera_centre = view_pos;
        auto theta = DegtoRad(verticle_fov);
        auto viewport_h = 2 * Tan(theta/2) * focal_dist;
        auto viewport_w = viewport_h * double(image_width)/image_height;

        w = Normalize(view_pos - view_des);
        u = Normalize(Cross(view_up, w));
        v = Cross(w, u);

        auto viewport_u = viewport_w *  u;
        auto viewport_v = viewport_h * -v;
        auto viewport_centre = camera_centre + focal_dist*-w;
        pixel_du = viewport_u / image_width;
        pixel_dv = viewport_v / image_height;

        auto viewport_upperleft = viewport_centre - viewport_u/2 - viewport_v/2;
        pixel00_centre = viewport_upperleft + pixel_du/2 + pixel_dv/2;

        auto aperture_radius = focal_dist * Tan(DegtoRad(defocus_angle/2));
        aperture_u = u * aperture_radius;
        aperture_v = v * aperture_radius;

        spp_root = int(Sqrt(sample_ppixel));  // For Antialiasing
        spp_inv = 1.0 / sample_ppixel;
        sample_du = pixel_du / (spp_root+1);
        sample_dv = pixel_dv / (spp_root+1);
    }
    Colour RayColour(const Ray& ray, const Shapes& _world, int depth) {

        const Scene& world = dynamic_cast<const Scene&>(_world);
        Intersection isect;
        if (!world.Intersect(ray, Interval(EPS_DEUX, POS_INF), isect))
            return background;
        
        // Emission
        Colour emission, attenuation;
        Ray scattered;  // Scattered Ray
        if (!isect.material->Scatter(ray, isect, attenuation, scattered)) {
            emission = isect.material->Emission(isect.u, isect.v, isect.coords);
            return emission;
        }

        Colour radiance(emission), incidence(0);
        double PDF_illu, PDF_brdf, PDF_mult;

        auto wi = scattered.dir;
        auto wo = -ray.dir;
        auto No = isect.normal;
        auto BRDF = isect.material->BRDFunc(wi, wo, No, attenuation);
        
        if (RandomFloat() <= roulette) {
            // Light Sampling
            Intersection samp_isect;
            world.SampleLights(samp_isect, PDF_illu);
            auto Li_dis = samp_isect.coords - isect.coords;
            auto Li_dir = Normalize(Li_dis);
            auto Li_ray = Ray(isect.coords, Li_dir);

            auto r = RandomFloat(), prob = 0.5;
            if (r <= prob) {
                // Visibility Test
                if ((world.Intersect(Li_ray, Interval(EPS_DEUX, POS_INF), samp_isect)) &&
                    (Length(Li_dis) - Length(isect.coords + No*EPS_DEUX - samp_isect.coords) <= EPS_UNIT)) {
                    // Direct Illumination
                    auto Li = samp_isect.material->Emission(samp_isect.u, samp_isect.v, samp_isect.coords);
                    auto wi = Li_dir;
                    auto Ns = samp_isect.normal;
                    // Multiple Importance Sampling
                    PDF_brdf = isect.material->PDFunc(wi, wo, No);
                    PDF_brdf = PDF_brdf * Abs(Dot(wi, No)*Dot(-wi, Ns)) / Length2(Li_dis);    // BRDF to Light
                    PDF_mult = PDF_brdf + PDF_illu;

                    incidence = BRDF * Li * Abs(Dot(wi, No)*Dot(-wi, Ns)) / Length2(Li_dis) / (PDF_mult * prob);
                } 
            } else {
                // Indirect Illumination
                if ((world.Intersect(scattered, Interval(EPS_DEUX, POS_INF), samp_isect)) &&
                    (Dot(wi, No) >= 0)) {
                    PDF_brdf = isect.material->PDFunc(wi, wo, No);
                    // Multiple Importance Sampling
                    Intersection test_isect;
                    if (samp_isect.material->Shines())
                        samp_isect.object->Sample(test_isect, PDF_illu);
                    else 
                        PDF_illu = 0.0;
                    PDF_illu = PDF_illu * Length2(Li_dis) / Abs(Dot(wi, No)*Dot(-wi, samp_isect.normal)); // Light to BRDF
                    PDF_mult = PDF_brdf + PDF_illu;

                    incidence = PDF_mult > EPS_DEUX ? 
                                BRDF * RayColour(scattered, world, depth-1) * Dot(wi, No) / (PDF_mult * (1-prob)) : 
                                Colour(0);
                }
            }
        }

        radiance += incidence / roulette;
        return Vector3::Min(Vector3::Max(radiance, Colour(0)), Colour(5));
    }
    Ray CastRay(int x, int y, int s) {
        auto pixel_centre = pixel00_centre + pixel_du*x + pixel_dv*y;
        auto pixel_offset = pixel_centre - pixel_du/2 - pixel_dv/2;
        auto pixel_sample = pixel_offset + sample_du*(s%spp_root+1) + sample_dv*(s/spp_root+1);

        auto ray_origin = (defocus_angle > 0.0)
                        ? SampleLens()
                        : camera_centre;
        auto ray_direction = Normalize(pixel_sample - ray_origin);
        auto ray_time = RandomFloat();
        return Ray(ray_origin, ray_direction, ray_time);
    }
    Ray CastRay(int x, int y) {
        auto sample_offset = Sample05();
        auto pixel_sample = pixel00_centre 
                          + pixel_du * (x+sample_offset.x) 
                          + pixel_dv * (y+sample_offset.y);  
        auto ray_origin = (defocus_angle > 0.0)
                        ? SampleLens()
                        : camera_centre;
        auto ray_direction = Normalize(pixel_sample - ray_origin);
        auto ray_time = RandomFloat();
        return Ray(ray_origin, ray_direction, ray_time);                                     
    }
    Point3 Sample05() {
        return Point3(RandomFloat()-.5f, RandomFloat()-.5f, 0);
    }
    Point3 SampleLens() {
        auto random_point = RandomVec3Disk();
        return camera_centre + aperture_u * random_point.x 
                             + aperture_v * random_point.y;
    }

    // Members
    int image_height, spp_root;
    double spp_inv;
    Point3 camera_centre, pixel00_centre;
    Vector3 pixel_du, pixel_dv;
    Vector3 sample_du, sample_dv;
    Vector3 u, v, w;    // Camera Basis Vectors
    Vector3 aperture_u, aperture_v;
};

#endif // CAMERA_H