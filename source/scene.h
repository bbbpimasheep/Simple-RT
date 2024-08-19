#pragma once
#ifndef SCENE_H
#define SCENE_H

#include "shapes.h"
#include "global.h"

using std::vector;

class Scene : public Shapes {
public:
    // Constructor
    Scene() = default;
    Scene(shared_ptr<Shapes> object) { objects.push_back(object); }

    // Methods
    void AddObject(shared_ptr<Shapes> object) { 
        objects.push_back(object); 
        bounds = Union(bounds, object->BBox());
    }
    void Clear() { objects.clear(); }
    bool Intersect(const Ray& ray, Interval ray_time, Intersection& isect) const override {
        Intersection temp_isect;
        bool happened = false;
        double closest = ray_time._max;
        for (const auto& object : objects) {
            if (object->Intersect(ray, Interval(ray_time._min, closest), temp_isect)) {
                happened = true;
                closest = temp_isect.time;
                isect = temp_isect;
            }
        }
        return happened;
    }
    Bounds3 BBox() const override { return bounds; }

    // Members
    std::vector<shared_ptr<Shapes>> objects;

private:
    Bounds3 bounds;
};


#endif // SCENE_H