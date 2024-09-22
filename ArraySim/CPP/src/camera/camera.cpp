#include "camera.hpp"

const float Camera::DEFAULT_SPEED = 200.0f;
const float Camera::DEFAULT_SENSITIVITY = 0.1f;
const float Camera::DEFAULT_FOV = 45.0f;
const float Camera::DEFAULT_PHI = 0.0;
const float Camera::DEFAULT_THETA = 0.0;

// Camera placed on the +z axis
const glm::vec3 Camera::DEFAULT_POSITION(0.0f, 0.0f, 3.0f);
// World up is the +y axis
const glm::vec3 Camera::DEFAULT_UP(0.0f, 1.0f, 0.0f);
// Camera looks towards the -z axis direction
const glm::vec3 Camera::DEFAULT_DIRECTION(0.0f, 0.0f, -1.0f);
