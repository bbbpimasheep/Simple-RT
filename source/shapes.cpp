#include "shapes.h"
#include "material.h"

bool Sphere::Shines() const { 
    if (Length(material->Shines()) > EPS_UNIT)
        return true;
    return false;
}
bool Quad::Shines() const { 
    if (Length(material->Shines()) > EPS_UNIT)
        return true;
    return false;
}