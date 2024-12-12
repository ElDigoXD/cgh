#pragma once

#include "Color.h"

class Material {
public:
    Color albedo = Color(0.5, 0.5, 0.7);
    bool is_diffuse = true;
    bool is_specular = false;
    bool is_dielectric = false;
    bool is_metallic = false;
    Color specular_color = Color(1, 1, 1);
    Real shininess = 10; // [0, 1000]
    Real index_of_refraction = 1;
    Real roughness = 0;
};