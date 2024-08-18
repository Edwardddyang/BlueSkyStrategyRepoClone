#ifndef CAMERA_H
#define CAMERA_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum class Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};


// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
class Camera
{
public:

    // Default camera attributes
    static const float DEFAULT_YAW;  // Such that the camera looks in the direction of th -z axis
    static const float DEFAULT_PITCH;
    static const float DEFAULT_SPEED;
    static const float DEFAULT_SENSITIVITY;
    static const float DEFAULT_FOV;  // Ranges from 1.0 to 45.0 degrees
    static const float DEFAULT_PHI;
    static const float DEFAULT_THETA;

    static const glm::vec3 DEFAULT_POSITION;
    static const glm::vec3 DEFAULT_UP;
    static const glm::vec3 DEFAULT_DIRECTION;

    // camera Attributes
    glm::vec3 Position; // Camera position
    glm::vec3 Direction; // The direction that the camera is looking
    glm::vec3 Up; // Up vector of the camera
    glm::vec3 Right;
    glm::vec3 WorldUp;
    float Radius; // Distance of the camera from the model's bounding box center
    glm::vec3 center; // Center of the model
    // rotation angles
    float Phi;
    float Theta;

    // Use quarternion to get around the gimbal lock when using Euler Angles
    glm::quat q_camera = glm::quat(1, 0, 0, 0); // No initial rotation
    // camera options
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    Camera(glm::vec3 position = DEFAULT_POSITION, glm::vec3 camera_direction = DEFAULT_DIRECTION, glm::vec3 up = DEFAULT_UP,
        float phi = DEFAULT_PHI, float theta = DEFAULT_THETA, float camera_speed = DEFAULT_SPEED,
        float mouse_sensitivity = DEFAULT_SENSITIVITY, float fov = DEFAULT_FOV, float camera_distance = 5.0f,
        glm::vec3 model_center = glm::vec3(0.0f,0.0f,0.0f)) : Direction(camera_direction), MovementSpeed(camera_speed),
        MouseSensitivity(mouse_sensitivity), Zoom(fov), Position(position), WorldUp(up), Theta(theta), Phi(phi),
        center(model_center), Radius(camera_distance)
    {
        updateCameraVectors();
    }

    // returns the view matrix calculated using Euler Angles and the LookAt Matrix
    glm::mat4 GetViewMatrix()
    {
        // Phi (Vertical direction) rotates the camera about its right vector
        q_camera = glm::angleAxis(glm::radians(-Phi), Right);
        // Theta (Horizaontal direction) rotates the camera about its up vector
        q_camera *= glm::angleAxis(glm::radians(Theta), Up);
        // API: (camera position, the point that the camera is looking at, world up)
        // We rotate by the rotation matrix
        return glm::lookAt(Position, center, Up) * glm::mat4_cast(q_camera);
    }

    // processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
    void ProcessKeyboard(Camera_Movement direction, float deltaTime)
    {
        float velocity = MovementSpeed * deltaTime;
        if (direction == Camera_Movement::FORWARD)
            Position += Direction * velocity;
        if (direction == Camera_Movement::BACKWARD)
            Position -= Direction * velocity;
        if (direction == Camera_Movement::LEFT)
            Position -= Right * velocity;
        if (direction == Camera_Movement::RIGHT)
            Position += Right * velocity;
    }

    // Process orbiting movement
    void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true)
    {
        xoffset *= MouseSensitivity;
        yoffset *= MouseSensitivity;

        Theta -= xoffset;
        Phi -= yoffset;
    }

    // processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
    void ProcessMouseScroll(float yoffset)
    {
        Zoom -= (float)yoffset;
        if (Zoom < 1.0f)
            Zoom = 1.0f;
        if (Zoom > 45.0f)
            Zoom = 45.0f;
    }

private:
    // calculates the Direction vector from the Camera's (updated) Euler Angles
    void updateCameraVectors()
    {
        // Get the camera's right and up vector
        Right = glm::normalize(glm::cross(Direction, WorldUp));
        Up    = glm::normalize(glm::cross(Right, Direction));
    }
};
#endif