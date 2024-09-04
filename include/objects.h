#pragma once
#ifndef OBJECTS_H
#define OBJECTS_H

#include "shapes.h"
#include "scene.h"

inline shared_ptr<Scene> CreateBox(const Point3& a, const Point3& b, const shared_ptr<Material> material,
                                   const Transform& transform=Transform()) {
    auto sides = make_shared<Scene>();

    auto _min = Point3(Min(a.x, b.x), Min(a.y, b.y), Min(a.z, b.z));
    auto _max = Point3(Max(a.x, b.x), Max(a.y, b.y), Max(a.z, b.z));
    auto delta_x = Vector3(_max.x - _min.x, 0, 0);
    auto delta_y = Vector3(0, _max.y - _min.y, 0);
    auto delta_z = Vector3(0, 0, _max.z - _min.z);

    sides->AddObject(make_shared<Quad>(Point3(_min.x, _min.y, _max.z),  delta_x,  delta_y, material, transform));    // Front
    sides->AddObject(make_shared<Quad>(Point3(_max.x, _min.y, _min.z), -delta_x,  delta_y, material, transform));    // Back
    sides->AddObject(make_shared<Quad>(Point3(_min.x, _min.y, _min.z),  delta_z,  delta_y, material, transform));    // Left
    sides->AddObject(make_shared<Quad>(Point3(_max.x, _min.y, _max.z), -delta_z,  delta_y, material, transform));    // Right
    sides->AddObject(make_shared<Quad>(Point3(_min.x, _max.y, _max.z),  delta_x, -delta_z, material, transform));    // Top
    sides->AddObject(make_shared<Quad>(Point3(_min.x, _min.y, _min.z),  delta_x,  delta_z, material, transform));    // Bottom

    return sides;
}

#endif // OBJECTS_H