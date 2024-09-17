/* Infinite plane representation of the sun by a point and a normal vector */

#pragma once

#include <stdlib.h>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

struct SunPlane {
    glm::vec3 point;
    glm::vec3 normal;

    double a;
    double b;
    double c;
    double d;
    double irradiance;

    // NOTE: By convention, we use a right handed coordinate system where the +z axis points up towards
    // the sky. Therefore, the returned normal vector is really just the normalized position of the sun in
    // the sky and will always have +ve z component
    SunPlane(double azimuth, double elevation, std::string direction, double bearing, const glm::vec3 min_values,
            glm::vec3 max_values, double irr = 1000.0);
};
