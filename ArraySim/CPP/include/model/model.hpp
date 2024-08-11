#ifndef MODEL_H
#define MODEL_H

#include <string>
#include "Shader.hpp"
#include "mesh.hpp"
#include "camera.hpp"
#include <filesystem>
#include <memory>

class Model {
public:
    Model() : loaded_canopy(false), loaded_array(false),
    max_values(glm::vec3(std::numeric_limits<float>::lowest())),
    min_values(glm::vec3(std::numeric_limits<float>::max())) {}
    
    // Functions to load canopy file and array cells
    void loadCanopy(const std::filesystem::path& path);
    void loadArray(const std::filesystem::path& path);

    bool loaded_canopy;
    bool loaded_array;

    // Initialize the camera
    void init_camera();
    // Initialize shaders
    void init_shaders();

    void Draw(double window_width, double window_height);

    glm::vec3 get_max_values() const {return max_values;}
    glm::vec3 get_min_values() const {return min_values;}
    glm::vec3 get_center() const {return center;}
    float get_bbox_length() const {return bbox_length;}
    float get_bsphere_radius() const {return bsphere_radius;}
    float get_camera_distance() const {return camera_distance;}

    // For viewing and navigating the scene
    std::shared_ptr<Camera> camera;

private:
    std::shared_ptr<Shader> shaders;
    // A model is composed of many meshes
    std::shared_ptr<Mesh> canopy_mesh;
    std::vector<std::shared_ptr<Mesh>> array_cell_meshes;

    // Max and min coordinate values along each dimension
    glm::vec3 max_values;
    glm::vec3 min_values;

    // Center of bounding box defined by max_values and min_values
    glm::vec3 center;
    // Diagonal length of the bounding box
    float bbox_length;
    // Radius of the bounding sphere
    float bsphere_radius;
    // The distance of the camera relative to the center of the bounding box
    float camera_distance;
    // Position of the camera
    glm::vec3 camera_position;
    // Direction that the camera is pointing in
    glm::vec3 camera_direction;
    void update_max_min_values(const std::shared_ptr<Mesh>& mesh);
};

#endif /* MODEL_H */