#include "camera.hpp"

const float Camera::DEFAULT_YAW = -90.0f;
const float Camera::DEFAULT_PITCH = 0.0f;
const float Camera::DEFAULT_SPEED = 25.0f;
const float Camera::DEFAULT_SENSITIVITY = 0.1f;
const float Camera::DEFAULT_FOV = 45.0f;

// Camera placed on the +z axis
const glm::vec3 Camera::DEFAULT_POSITION(0.0f, 0.0f, 3.0f);
// World up is the +y axis
const glm::vec3 Camera::DEFAULT_UP(0.0f, 1.0f, 0.0f);
// Camera looks towards the -z axis direction
const glm::vec3 Camera::DEFAULT_FRONT(0.0f, 0.0f, -1.0f);
