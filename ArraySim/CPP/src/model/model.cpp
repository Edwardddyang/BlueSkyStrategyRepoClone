#define STB_IMAGE_IMPLEMENTATION
#include "model.hpp"
#include <stdlib.h>
#include <iostream>
#include <glad/glad.h> 
#include <stb_image.h>

void printMat4(const glm::mat4& mat) {
    for (int i = 0; i < 4; ++i) {
        std::cout << "| ";
        for (int j = 0; j < 4; ++j) {
            std::cout << mat[i][j] << " ";
        }
        std::cout << "|" << std::endl;
    }
}

void printMat3(const glm::mat3& mat) {
    for (int i = 0; i < 3; ++i) {
        std::cout << "| ";
        for (int j = 0; j < 3; ++j) {
            std::cout << mat[i][j] << " ";
        }
        std::cout << "|" << std::endl;
    }
}

void Model::Draw(Shader &shader, double window_width, double window_height)
{
    shader.use();
    
    // Create projection, view, model matrices
    float near_plane = bsphere_radius / 10.0f;
    float far_plane = (camera_distance + bsphere_radius) * 1.2f;
    glm::mat4 projection = glm::perspective(glm::radians(camera->Zoom),
                                            (float)window_width / (float)window_height,
                                            near_plane, far_plane);
    glm::mat4 view = camera->GetViewMatrix();
    glm::mat4 model = glm::mat4(1.0f); // Identity model matrix (no change from local to world coordinates)
    
    // Set the uniform matrices
    shader.setMat4("projection", projection);
    shader.setMat4("view", view);
    shader.setMat4("model", model);

    // Set uniform lighting variables
    // shader.setVec3("material.specular", 0.5f, 0.5f, 0.5f);
    // shader.setFloat("material.shininess", 32.0f);
    // shader.setVec3("material.diffuse", 1.0f, 1.0f, 1.0f);
    // shader.setVec3("viewPos", camera.Position);
    // /* Set light properties */
    // shader.setVec3("light.direction", -0.2f, -1.0f, -0.3f);
    // shader.setVec3("light.ambient",  0.2f, 0.2f, 0.2f);
    // shader.setVec3("light.diffuse",  0.8f, 0.8f, 0.8f); // darken diffuse light a bit
    // shader.setVec3("light.specular", 1.0f, 1.0f, 1.0f); 

    for(unsigned int i = 0; i < meshes.size(); i++) {
        meshes[i]->Draw(shader);
    }
}

void Model::loadCanopy(const std::filesystem::path& path) {
    if (!std::filesystem::is_regular_file(path)) {
        std::cout << "Canopy file: " << path << " is not a file" << std::endl;
        return;
    }

    std::shared_ptr<Mesh> canopy_mesh = std::make_shared<Mesh>(path);
    meshes.push_back(canopy_mesh);
    update_max_min_values(canopy_mesh);
    loaded_canopy = true;
}

void Model::loadArray(const std::filesystem::path& path) {
    if (!std::filesystem::is_directory(path)) {
        std::cout << "Array directory: " << path << " is not a directory" << std::endl;
        return;
    }
    // Load all stl files in the current directory
    for (const auto& entry : std::filesystem::directory_iterator(path))
    {
        std::shared_ptr<Mesh> cell_mesh = std::make_shared<Mesh>(entry.path());
        meshes.push_back(cell_mesh);
        update_max_min_values(cell_mesh);
    }
}

void Model::init_camera() {
    center = glm::vec3((max_values.x + min_values.x) / 2.0f,
                        (max_values.y + min_values.y) / 2.0f,
                        (max_values.z + min_values.z) / 2.0f);
    bbox_length = glm::length(max_values - min_values);
    bsphere_radius = bbox_length / 2.0f;
    camera_distance = bsphere_radius * 1.5f;
    camera_position = center + glm::vec3(0.0f, 0.0f, camera_distance);
    camera_direction = glm::normalize(center - camera_position);

    camera = std::make_shared<Camera>(camera_position, camera_direction,
                                      Camera::DEFAULT_UP, Camera::DEFAULT_PHI,
                                      Camera::DEFAULT_THETA, Camera::DEFAULT_SPEED,
                                      Camera::DEFAULT_SENSITIVITY, Camera::DEFAULT_FOV,
                                      camera_distance, center);
}

void Model::update_max_min_values(const std::shared_ptr<Mesh>& mesh) {
    const glm::vec3 mesh_max_values = mesh->get_max_values();
    const glm::vec3 mesh_min_values = mesh->get_min_values();

    max_values.x = std::max(max_values.x, mesh_max_values.x);
    max_values.y = std::max(max_values.y, mesh_max_values.y);
    max_values.z = std::max(max_values.z, mesh_max_values.z);

    min_values.x = std::min(min_values.x, mesh_min_values.x);
    min_values.y = std::min(min_values.y, mesh_min_values.y);
    min_values.z = std::min(min_values.z, mesh_min_values.z);
}
