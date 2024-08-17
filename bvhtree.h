#pragma once
#ifndef BVHTREE_H
#define BVHTREE_H

#include <algorithm>

#include "global.h"
#include "mathematics.h"
#include "shapes.h"
#include "scene.h"
#include "bounds.h"

class BVHNode : public Shapes {
public:
    // Constructors
    BVHNode(Scene scene) : BVHNode(scene.objects, 0, scene.objects.size()) {}
    BVHNode(std::vector<shared_ptr<Shapes>>& objects, uint32_t start=0, uint32_t end=0) {
        bounds = Bounds3::Empty;
        uint32_t object_length = end - start;
        if (object_length == 0) {
            // Create empty BVHNode
            left = nullptr; right = nullptr;
            core = nullptr;
        } else if (object_length == 1) {
            // Create leaf BVHNode
            left = nullptr; right = nullptr;
            core = objects[start];
            bounds = objects[start]->BBox();
        } else if (object_length == 2) {
            left  = objects[start]; right = objects[start+1];
            bounds = Union(left->BBox(), right->BBox());
        } else {
            for (auto& obj : objects) 
                bounds = Union(bounds, obj->BBox());
            int axis = bounds.MaxAxis();
            // std::clog << "Splitting along axis " << axis << std::endl;
            // std::clog << "Objects Sorted: " << objects.size() << " " << end << std::endl;
            std::sort(objects.begin()+start, objects.begin()+end, 
                     [axis](const shared_ptr<Shapes> o1, const shared_ptr<Shapes> o2) {
                if (!o1 || !o2) {
                    throw std::runtime_error("Null pointer detected in objects");
                }
                return o1->BBox()[axis].Centroid() < 
                       o2->BBox()[axis].Centroid();
            });
            // Split objects into left and right subsets
            // Recursively build left and right subtrees
            uint32_t mid = start + object_length/2;
            // std::clog << "Sorted Middles: " << mid << std::endl;
            left  = make_shared<BVHNode>(objects, start, mid);
            right = make_shared<BVHNode>(objects, mid, end);
        }
    }

    // Methods
    bool Intersect(const Ray& ray, Interval ray_time, Intersection& isect) const override {
        if (!bounds.Intersect(ray, ray_time)) 
            return false;
        if (left == nullptr && right == nullptr && core != nullptr) 
            return core->Intersect(ray, ray_time, isect);
        
        auto isect_l = left ->Intersect(ray, ray_time, isect);
        auto isect_r = right->Intersect(ray, Interval(ray_time._min, (isect_l ? isect.time : ray_time._max)), isect);
        return isect_l || isect_r;
    }
    Bounds3 BBox() const override { return bounds; }

private:
    // Members
    shared_ptr<Shapes> left;
    shared_ptr<Shapes> right;
    shared_ptr<Shapes> core;
    Bounds3 bounds;
};


#endif // BVHTREE_H