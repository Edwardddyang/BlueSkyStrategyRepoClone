#include "sun_plane.hpp"
#include <cmath>
#include <iostream>

constexpr double degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

SunPlane::SunPlane(double azimuth, double elevation, std::string direction, double bearing, const glm::vec3 min_values,
                  glm::vec3 max_values, double irr) {
    irradiance = irr;
    // Get corrected azimuth which is the azimuth relative to the bearing of the car instead
    // of from true north
    double corrected_azimuth;
    if (bearing <= azimuth) {
        corrected_azimuth = azimuth - bearing;
    } else {
        corrected_azimuth = 360.0 - (bearing - azimuth);
    }

    // Calculate normal vector
    double elevation_rad = degToRad(elevation);
    double azimuth_rad = degToRad(corrected_azimuth);

    double x = std::cos(elevation_rad) * std::cos(azimuth_rad);
    double y = std::cos(elevation_rad) * std::sin(azimuth_rad);
    double z = std::sin(elevation_rad);

    // Correct the normal based on the direction of the nose of the car
    if (direction == "+x") {
        y *= -1.0;
    } else if (direction == "-x") {
        x *= -1.0;
    } else if (direction == "+y") {
        std::swap(x, y);
    } else if (direction == "-y") {
        std::swap(x, y);
        x *= -1.0;
        y *= -1.0;
    } else {
        std::string error_message = "Nose direction must be one of {-x, +x, -y, +y}";
        throw std::runtime_error(error_message);
    }
    normal = glm::vec3(x, y, z);

    // Make sure the point on the plane is outside of the stl mesh along any dimension
    glm::vec3 furthest_point = glm::vec3(
        std::max(std::abs(max_values.x), std::abs(min_values.x)),
        std::max(std::abs(max_values.y), std::abs(min_values.y)),
        std::max(std::abs(max_values.z), std::abs(min_values.z))
    );
    float offset_distance = glm::length(furthest_point) + 1.0;  // Add buffer

    point = normal * offset_distance;

    // Define the plane equation form
    a = x;
    b = y;
    c = z;
    d = -1.0 * (a * point.x + b * point.y + c * point.z);
}