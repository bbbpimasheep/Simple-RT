#include "global.h"
#include "mathematics.h"
#include "shapes.h"
#include "scene.h"
#include "bvhtree.h"
#include "camera.h"
#include "objects.h"
#include "materials.h"
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
                MATsphere = make_shared<SpecularDielectric>(1.5);
                scene.AddObject(make_shared<Sphere>(centre, r, MATsphere));
            }
            ProgressBar(((z+15) * 30 + (x+15) + 1)/ 900.0);
        }
    }

    std::clog << "\nGenerating Scene Complete! \n";

    auto MATdiffuse    = make_shared<Lambertian>(Colour(0.4, 0.2, 0.1));
    auto MATDielectric = make_shared<SpecularDielectric>(1.5);
    auto MATmetalllic  = make_shared<Metal>(Colour(0.7, 0.6, 0.5), 0.0);
    scene.AddObject(make_shared<Sphere>(Point3(-6, 1, 0), radius_large, MATdiffuse));
    scene.AddObject(make_shared<Sphere>(Point3(-1, 1, 0), radius_large, MATDielectric));
    scene.AddObject(make_shared<Sphere>(Point3( 4, 1, 0), radius_large, MATmetalllic));

    scene = Scene(make_shared<BVHNode>(scene));

    Camera camera;
    camera.aspect_ratio  = 1.778;
    camera.image_width   = 512;
    camera.sample_ppixel = 64;
    camera.background    = Colour(0.7, 0.8, 1.0);
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
    camera.background    = Colour(0.7, 0.8, 1.0);
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
    camera.background    = Colour(0.7, 0.8, 1.0);
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

void PerlinSpheres(uint32_t& minutes, uint32_t& seconds) {
    Scene scene;

    auto perlin_texture = make_shared<NoiseTexture>(4);
    scene.AddObject(make_shared<Sphere>(Point3(0, -1000, 0), 1000, make_shared<Lambertian>(perlin_texture)));
    scene.AddObject(make_shared<Sphere>(Point3(0,  2,    0), 2, make_shared<Lambertian>(perlin_texture)));

    Camera camera;
    camera.aspect_ratio  = 1.778;
    camera.image_width   = 512;
    camera.sample_ppixel = 256;
    camera.background    = Colour(0.7, 0.8, 1.0);
    camera.roulette      = 0.8;

    camera.verticle_fov  = 20;
    camera.view_up       = Vector3(0,1,0);
    camera.view_pos      = Point3(13,2,3);
    camera.view_des      = Point3(0,0,0);
    camera.defocus_angle = 0.0;

    auto start = std::chrono::system_clock::now();
    camera.RenderScene(Scene(scene));
    auto stop = std::chrono::system_clock::now();
    minutes = std::chrono::duration_cast<std::chrono::minutes>(stop - start).count();
    seconds = std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() - 60*minutes;
}

void TestSquares(uint32_t& minutes, uint32_t& seconds) {
    Scene scene;

    // Materials
    auto left_aka    = make_shared<Lambertian>(Colour(1.0, 0.2, 0.2));
    auto back_midori = make_shared<Lambertian>(Colour(0.2, 1.0, 0.2));
    auto right_ai    = make_shared<Lambertian>(Colour(0.2, 0.2, 1.0));
    auto upper_kiiro = make_shared<Lambertian>(Colour(1.0, 0.5, 0.0));
    auto lower_ao    = make_shared<Lambertian>(Colour(0.2, 0.8, 0.8));

    // Quadrilaterals
    scene.AddObject(make_shared<Quad>(Point3(-3,-2, 5), Vector3(0, 0,-4), Vector3(0, 4, 0), left_aka));
    scene.AddObject(make_shared<Quad>(Point3(-2,-2, 0), Vector3(4, 0, 0), Vector3(0, 4, 0), back_midori));
    scene.AddObject(make_shared<Quad>(Point3( 3,-2, 1), Vector3(0, 0, 4), Vector3(0, 4, 0), right_ai));
    scene.AddObject(make_shared<Quad>(Point3(-2, 3, 1), Vector3(4, 0, 0), Vector3(0, 0, 4), upper_kiiro));
    scene.AddObject(make_shared<Quad>(Point3(-2,-3, 5), Vector3(4, 0, 0), Vector3(0, 0,-4), lower_ao));

    Camera camera;
    camera.aspect_ratio  = 1.0;
    camera.image_width   = 512;
    camera.sample_ppixel = 64;
    camera.background    = Colour(0.7, 0.8, 1.0);
    camera.roulette      = 0.8;

    camera.verticle_fov  = 80;
    camera.view_up       = Vector3(0,1,0);
    camera.view_pos      = Point3(0,0,9);
    camera.view_des      = Point3(0,0,0);
    camera.defocus_angle = 0.0;

    auto start = std::chrono::system_clock::now();
    camera.RenderScene(scene);
    auto stop = std::chrono::system_clock::now();
    minutes = std::chrono::duration_cast<std::chrono::minutes>(stop - start).count();
    seconds = std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() - 60*minutes;
}

void SingleLight(uint32_t& minutes, uint32_t& seconds) {
    Scene scene;
    scene.AddObject(make_shared<Sphere>(Point3(0,-1000,0), 1000, make_shared<Lambertian>(Colour(1.0, 0.2, 0.2))));
    scene.AddObject(make_shared<Sphere>(Point3(0,2,0), 2, make_shared<Lambertian>(Colour(1.0, 0.2, 0.2))));

    auto material_light = make_shared<Light>(Colour(8.0));
    scene.AddObject(make_shared<Quad>(Point3(3,1,-2), Vector3(2,0,0), Vector3(0,2,0), material_light));

    Camera camera;
    camera.aspect_ratio  = 1.778;
    camera.image_width   = 512;
    camera.sample_ppixel = 1024;
    camera.background    = Colour(0.0);
    camera.roulette      = 0.8;

    camera.verticle_fov  = 20;
    camera.view_up       = Vector3(0,1,0);
    camera.view_pos      = Point3(26,3,6);
    camera.view_des      = Point3(0,2,0);
    camera.defocus_angle = 0.0;

    auto start = std::chrono::system_clock::now();
    camera.RenderScene(scene);
    auto stop = std::chrono::system_clock::now();
    minutes = std::chrono::duration_cast<std::chrono::minutes>(stop - start).count();
    seconds = std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() - 60*minutes;
}

void CornellBox(uint32_t& minutes, uint32_t& seconds) {
    Scene scene;

    auto red   = make_shared<Lambertian>(Colour(.65, .05, .05));
    auto white = make_shared<Lambertian>(Colour(.73, .71, .68));
    auto green = make_shared<Lambertian>(Colour(.12, .45, .15));
    auto light = make_shared<Light>(Colour(1.0, .97, .86) * 60);
    auto metCu = make_shared<Conductor>(Colour(.95, .64, .54), .002, .002, 1.0);
    auto metAg = make_shared<Conductor>(Colour(.95, .93, .88),  0.5,  0.5, 0.8);
    auto metAu = make_shared<Conductor>(Colour(1.0, .71, .29), .001, .001, 1.0);
    auto metAl = make_shared<Conductor>(Colour(.91, .92, .92),  0.3,  0.3, 0.4);
    auto glass = make_shared<Dielectric>(1.5, 0.5, 0.5);

    auto NORM_L = Vector3(-1,  0,  0);
    auto NORM_R = Vector3( 1,  0,  0);
    auto NORM_B = Vector3( 0,  1,  0);
    auto NORM_T = Vector3( 0, -1,  0);
    auto NORM_Z = Vector3( 0,  0,  1);
    auto NORM_G = Vector3( 0,  0, -1);

    scene.AddObject(make_shared<Quad>(Point3( 555,   0,   0), Vector3(   0, 555,   0), Vector3(   0,   0, 555), green, NORM_L));
    scene.AddObject(make_shared<Quad>(Point3(   0,   0,   0), Vector3(   0, 555,   0), Vector3(   0,   0, 555), red,   NORM_R));
    scene.AddObject(make_shared<Quad>(Point3( 343, 554, 332), Vector3(-130,   0,   0), Vector3(   0,   0,-105), light, NORM_T));
    scene.AddObject(make_shared<Quad>(Point3(   0,   0,   0), Vector3( 555,   0,   0), Vector3(   0,   0, 555), white, NORM_B));
    scene.AddObject(make_shared<Quad>(Point3( 555, 555, 555), Vector3(-555,   0,   0), Vector3(   0,   0,-555), white, NORM_T));
    scene.AddObject(make_shared<Quad>(Point3(   0,   0, 555), Vector3( 555,   0,   0), Vector3(   0, 555,   0), white, NORM_G));

    auto rotate1 = RotateY(15.0);
    auto trans1  = Translate(Vector3(265, 0, 295));
    scene.AddObject(CreateBox(Point3(0, 0, 0), Point3(165, 330, 165), glass, trans1*rotate1));
    auto rotate2 = RotateY(-18.0);
    auto trans2  = Translate(Vector3(130, 0, 65));
    scene.AddObject(CreateBox(Point3(0, 0, 0), Point3(165, 165, 165), metAg, trans2*rotate2));

    Camera camera;
    camera.aspect_ratio  = 1.0;
    camera.image_width   = 1024;
    camera.sample_ppixel = 1024;
    camera.background    = Colour(0.0);
    camera.roulette      = 0.8;

    camera.verticle_fov  = 40;
    camera.view_up       = Vector3(0,1,0);
    camera.view_pos      = Point3(278,278,-800);
    camera.view_des      = Point3(278,278,0);
    camera.defocus_angle = 0.0;

    auto start = std::chrono::system_clock::now();
    camera.RenderScene(scene);
    auto stop = std::chrono::system_clock::now();
    minutes = std::chrono::duration_cast<std::chrono::minutes>(stop - start).count();
    seconds = std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() - 60*minutes;
}

int main() {
    uint32_t minutes=0, seconds=0;
    switch (7) {
        case 1: BouncingBalls(minutes, seconds);    break;
        case 2: CheckboardBalls(minutes, seconds);  break;
        case 3: PlanetEarth(minutes, seconds);      break;
        case 4: PerlinSpheres(minutes, seconds);    break;
        case 5: TestSquares(minutes, seconds);      break;
        case 6: SingleLight(minutes, seconds);      break;
        case 7: CornellBox(minutes, seconds);       break;
        default: std::clog << "Invalid choice.\n";  break;
    }
    std::clog << "Render complete: \n";
    std::clog << "Time taken: " << minutes << " Minutes and " << seconds << " Seconds.\n";
    return 0;
}