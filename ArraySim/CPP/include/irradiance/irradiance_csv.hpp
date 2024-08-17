/* All utilities for calculating the effective irradiance csv */

#pragma once

#include <iostream>
#include <stdlib.h>
#include "Luts.hpp"
#include "model.hpp"
#include "sun_plane.hpp"

class IrradianceCSV {
public:
    static std::vector<glm::vec2> convexHull2D(const std::vector<glm::vec2>& points);
    static glm::vec3 findOrthogonalVector(const glm::vec3& normal);

    IrradianceCSV(std::shared_ptr<SunPositionLUT> sun_position_lut, std::shared_ptr<Model> car_model,
                  double bearing, std::string direction);
private:
    std::shared_ptr<SunPositionLUT> sun_position;
    std::shared_ptr<Model> car;

    const int NUM_RAYS = 3000; // Number of rays to generate for each partially shaded triangles
    void compute_canopy_shadow(std::shared_ptr<SunPlane>& sun_plane);
};
