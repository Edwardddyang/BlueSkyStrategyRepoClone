#ifndef CAMERA_H
#define CAMERA_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

/** Models the camera for an OpenGL scene */
class Camera
{
public:
    // Defines 4 directions for camera movement on a plane perpendicular to the camera
    enum class CameraMovement {
        FORWARD,  // Into the screen
        BACKWARD,  // Out of the screen
        LEFT,
        RIGHT
    };

    // Default camera attributes
    static const float DEFAULT_SPEED;
    static const float DEFAULT_SENSITIVITY;
    static const float DEFAULT_FOV;
    static const float DEFAULT_PHI;
    static const float DEFAULT_THETA;
    static const glm::vec3 DEFAULT_POSITION;
    static const glm::vec3 DEFAULT_UP;
    static const glm::vec3 DEFAULT_DIRECTION;

    // Camera attribute states
    glm::vec3 Position; // Camera position
    glm::vec3 Direction; // The direction that the camera is looking in
    glm::vec3 Up; // Up vector of the camera
    glm::vec3 Right; // Right vector of the camera
    glm::vec3 WorldUp; // Up vector for the entire world
    float Radius; // Distance of the camera from the model's bounding box center
    glm::vec3 center; // Center of the model
    // rotation angles
    float Phi;
    float Theta;
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    // Use quarternion to get around the gimbal lock when using Euler Angles
    glm::quat q_camera = glm::quat(1, 0, 0, 0); // No initial rotation

    Camera(glm::vec3 position = DEFAULT_POSITION, glm::vec3 camera_direction = DEFAULT_DIRECTION, glm::vec3 up = DEFAULT_UP,
        float phi = DEFAULT_PHI, float theta = DEFAULT_THETA, float camera_speed = DEFAULT_SPEED,
        float mouse_sensitivity = DEFAULT_SENSITIVITY, float fov = DEFAULT_FOV, float camera_distance = 5.0f,
        glm::vec3 model_center = glm::vec3(0.0f,0.0f,0.0f)) : Direction(camera_direction), MovementSpeed(camera_speed),
        MouseSensitivity(mouse_sensitivity), Zoom(fov), Position(position), WorldUp(up), Theta(theta), Phi(phi),
        center(model_center), Radius(camera_distance)
    {
        // Initialize the camera vectors
        Right = glm::normalize(glm::cross(Direction, WorldUp));
        Up    = glm::normalize(glm::cross(Right, Direction));
    }

    /** @brief Get the view matrix for the current camera position */
    glm::mat4 GetViewMatrix()
    {
        // Phi (Vertical direction) rotates the camera about its right vector
        q_camera = glm::angleAxis(glm::radians(Phi), Right);
        // Theta (Horizaontal direction) rotates the camera about its up vector
        q_camera *= glm::angleAxis(glm::radians(Theta), Up);
        // API: (camera position, the point that the camera is looking at, world up)
        // We rotate by the rotation matrix
        return glm::lookAt(Position, center, Up) * glm::mat4_cast(q_camera);
    }

    /** @brief Process right, up, forward, backward movement from keyboard input
     * @param direction: The keyboard input
     * @param deltaTime: The amount of time since the last frame
     */
    void ProcessKeyboard(CameraMovement direction, float deltaTime)
    {
        float velocity = MovementSpeed * deltaTime;
        if (direction == CameraMovement::FORWARD)
            Position += Direction * velocity;
        if (direction == CameraMovement::BACKWARD)
            Position -= Direction * velocity;
        if (direction == CameraMovement::LEFT)
            Position -= Right * velocity;
        if (direction == CameraMovement::RIGHT)
            Position += Right * velocity;
    }

    /** @brief Process orbital movement controlled by the mouse
     * @param xoffset: The amount of movement in the "x" axis created by the mouse
     * @param yoffset: The amount of movement in the "y" axis created by the mouse
     */
    void ProcessMouseMovement(float xoffset, float yoffset)
    {
        xoffset *= MouseSensitivity;
        yoffset *= MouseSensitivity;

        Theta -= xoffset;
        Phi -= yoffset;
    }

    /** @brief Process mouse scrolling 
     * @param yoffset: The amount of scrolling performed by the mouse in the y direction
     * 
     * Note: xoffset for fancier scrollwheels aren't supported
    */
    void ProcessMouseScroll(float yoffset)
    {
        Zoom -= (float)yoffset;
        if (Zoom < 1.0f)
            Zoom = 1.0f;
        if (Zoom > 45.0f)
            Zoom = 45.0f;
    }
};
#endif