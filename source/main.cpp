#include "global.h"
#include "mathematics.h"
#include "shapes.h"
#include "ray.h"
#include "scene.h"
#include "bvhtree.h"
#include "camera.h"
#include "material.h"
#include "texture.h"

Point3 RandomCentre(double x, double y, double z)
{ return Point3(x,y,z) + Point3(0.9*RandomFloat(),0,0.9*RandomFloat()); }

void BouncingBalls(uint32_t& minutes, uint32_t& seconds) {
    Scene scene;
    
    auto checker_texture = make_shared<CheckerTexture>(0.32, Colour(0.2, 0.3, 0.1), Colour(0.9, 0.9, 0.9));
    scene.AddObject(make_shared<Sphere>(Point3(0, -1000, 0), 1000, make_shared<Lambertian>(checker_texture)));

    auto radius_small = 0.2;
    auto radius_large = 1.0;

    std::clog << "Generating Scene... \n";

    vector<Point3> centre_arr(30*30);
    for (int x = -15; x < 15; x += 1) {
        auto r = radius_small;
        for (int z = -15; z < 15; z += 1) {
            double i_left = (z+15) * 30 + (x+15) - 1;
            double i_up   = (z+14) * 30 + (x+15);
            Point3 centre = RandomCentre(x, r, z);
            while ((i_left >= 0 && Length(centre_arr[i_left]-centre) < 2*r) ||
                   (i_up >= 0 && Length(centre_arr[i_up]-centre) < 2*r)) 
                centre = RandomCentre(x, r, z);
            if ((Length(centre - Point3(-6, r, 0)) <= 0.9) || 
                (Length(centre - Point3(-1, r, 0)) <= 0.9) ||
                (Length(centre - Point3( 4, r, 0)) <= 0.9)) { 
                centre_arr[(z+15) * 30 + (x+15)] = Point3(-15,-15,-15);
                continue; 
            }
            centre_arr[(z+15) * 30 + (x+15)] = centre;

            double random_option = RandomFloat();
            shared_ptr<Material> MATsphere;
            if (random_option < 0.6) {
                // Diffuse Sphere
                auto albedo = RandomColour() * RandomColour();
                MATsphere = make_shared<Lambertian>(albedo);
                auto centre_next = centre + Vector3(0, RandomFloat(0,.5), 0);
                scene.AddObject(make_shared<Sphere>(centre, centre_next, r, MATsphere));
            } else if (random_option < 0.9) {
                // Metal Sphere
                auto albedo    = RandomColour(0.5, 1.0);
                auto fuzziness = RandomFloat(0.0, 0.5);
                MATsphere = make_shared<Metal>(albedo, fuzziness);
                scene.AddObject(make_shared<Sphere>(centre, r, MATsphere));
            } else {
                // Glass Sphere
                MATsphere = make_shared<Dielectric>(1.5);
                scene.AddObject(make_shared<Sphere>(centre, r, MATsphere));
            }
            ProgressBar(((z+15) * 30 + (x+15) + 1)/ 900.0);
        }
    }

    std::clog << "\nGenerating Scene Complete! \n";

    auto MATdiffuse    = make_shared<Lambertian>(Colour(0.4, 0.2, 0.1));
    auto MATdielectric = make_shared<Dielectric>(1.5);
    auto MATmetalllic  = make_shared<Metal>(Colour(0.7, 0.6, 0.5), 0.0);
    scene.AddObject(make_shared<Sphere>(Point3(-6, 1, 0), radius_large, MATdiffuse));
    scene.AddObject(make_shared<Sphere>(Point3(-1, 1, 0), radius_large, MATdielectric));
    scene.AddObject(make_shared<Sphere>(Point3( 4, 1, 0), radius_large, MATmetalllic));

    scene = Scene(make_shared<BVHNode>(scene));

    Camera camera;
    camera.aspect_ratio  = 1.778;
    camera.image_width   = 512;
    camera.sample_ppixel = 64;
    // camera.max_depth  = 64;
    camera.roulette      = 0.8;

    camera.verticle_fov  = 20;
    camera.view_up       = Vector3(0,1,0);
    camera.view_pos      = Point3(12,2,3);
    camera.view_des      = Point3(0,0,0);
    camera.defocus_angle = 0.60;
    camera.focal_dist    = 9.0;

    auto start = std::chrono::system_clock::now();
    camera.RenderScene(scene);
    auto stop = std::chrono::system_clock::now();
    minutes = std::chrono::duration_cast<std::chrono::minutes>(stop - start).count();
    seconds = std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() - 60*minutes;
}

void CheckboardBalls(uint32_t& minutes, uint32_t& seconds) {
    Scene scene;

    auto checker_texture = make_shared<CheckerTexture>(0.32, Colour(0.2, 0.3, 0.1), Colour(0.9, 0.9, 0.9));

    scene.AddObject(make_shared<Sphere>(Point3(0,-10, 0), 10, make_shared<Lambertian>(checker_texture)));
    scene.AddObject(make_shared<Sphere>(Point3(0, 10, 0), 10, make_shared<Lambertian>(checker_texture)));

    Camera camera;
    camera.aspect_ratio  = 1.778;
    camera.image_width   = 512;
    camera.sample_ppixel = 64;
    camera.roulette      = 0.8;

    camera.verticle_fov  = 20;
    camera.view_up       = Vector3(0,1,0);
    camera.view_pos      = Point3(13,2,3);
    camera.view_des      = Point3(0,0,0);
    camera.defocus_angle = 0.0;
    
    auto start = std::chrono::system_clock::now();
    camera.RenderScene(scene);
    auto stop = std::chrono::system_clock::now();
    minutes = std::chrono::duration_cast<std::chrono::minutes>(stop - start).count();
    seconds = std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() - 60*minutes;
}

void PlanetEarth(uint32_t& minutes, uint32_t& seconds) {
    auto earth_texture = make_shared<ImageTexture>("../textures/earthmap.jpg");
    auto earth = make_shared<Sphere>(Point3(0), 2, make_shared<Lambertian>(earth_texture));

    Camera camera;
    camera.aspect_ratio  = 1.778;
    camera.image_width   = 1280;
    camera.sample_ppixel = 512;
    camera.roulette      = 0.8;

    camera.verticle_fov  = 20;
    camera.view_up       = Vector3(0,1,0);
    camera.view_pos      = Point3(-3,4,-12);
    camera.view_des      = Point3(0,0,0);
    camera.defocus_angle = 0.0;

    auto start = std::chrono::system_clock::now();
    camera.RenderScene(Scene(earth));
    auto stop = std::chrono::system_clock::now();
    minutes = std::chrono::duration_cast<std::chrono::minutes>(stop - start).count();
    seconds = std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() - 60*minutes;
}

int main() {
    uint32_t minutes=0, seconds=0;
    switch (3) {
        case 1: BouncingBalls(minutes, seconds);    break;
        case 2: CheckboardBalls(minutes, seconds);  break;
        case 3: PlanetEarth(minutes, seconds);      break;
    }
    std::clog << "Render complete: \n";
    std::clog << "Time taken: " << minutes << " Minutes and " << seconds << " Seconds.\n";
    return 0;
}