#define STB_IMAGE_IMPLEMENTATION
#include "model.hpp"
#include <stdlib.h>
#include <iostream>
#include <glad/glad.h> 
#include <stb_image.h>
#include "Globals.h"

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

void Model::Draw(double window_width, double window_height, std::vector<glm::vec3> colours, int cell_idx, bool outline)
{
    RUNTIME_ASSERT(shaders != nullptr, "Shaders not created for model draw");
    shaders->use();
    
    // Create projection, view, model matrices
    float near_plane = bsphere_radius / 10.0f;
    float far_plane = (camera_distance + bsphere_radius) * 1.2f;
    glm::mat4 projection = glm::perspective(glm::radians(camera->Zoom),
                                            (float)window_width / (float)window_height,
                                            near_plane, far_plane);
    glm::mat4 view = camera->GetViewMatrix();
    glm::mat4 model = glm::mat4(1.0f); // Identity model matrix (no change from local to world coordinates)
    
    // Set the uniform matrices
    shaders->setMat4("projection", projection);
    shaders->setMat4("view", view);
    shaders->setMat4("model", model);

    // Render array cells
    size_t num_cells = array_cell_meshes.size();
    for(unsigned int i = 0; i < num_cells; i++) {
        // For each array cell, shade it according to some colour
        glm::vec3 fill_colour = glm::vec3(1.0f, 1.0f, 1.0f);
        bool highlight = false;
        if (i == cell_idx) highlight = true;
        if (colours.size() == num_cells) {
            fill_colour = colours[i];
            array_cell_meshes[i]->Draw(shaders, fill_colour, outline, highlight);
        } else {
            array_cell_meshes[i]->Draw(shaders, fill_colour, outline, highlight);
        }
    }

    // Render canopy
    canopy_mesh->Draw(shaders, glm::vec3(1.0f, 1.0f, 1.0f), true, false);
}

void Model::loadCanopy(const std::filesystem::path& path) {
    RUNTIME_ASSERT(std::filesystem::is_regular_file(path), std::string("Canopy file: ") + path.string() + " is not a file");
    canopy_mesh = std::make_shared<Mesh>(path, centroid, num_vertices);
    canopy_vertices = canopy_mesh->get_vertices();
    loaded_canopy = true;
}

void Model::loadArray(const std::filesystem::path& path) {
    RUNTIME_ASSERT(std::filesystem::is_directory(path), std::string("Array directory: ") + path.string() + " is not a directory");
    // Load all stl files in the current directory
    for (const auto& entry : std::filesystem::directory_iterator(path))
    {
        std::shared_ptr<Mesh> cell_mesh = std::make_shared<Mesh>(entry.path(), centroid, num_vertices);
        array_cell_meshes.push_back(cell_mesh);
        array_cell_vertices.push_back(cell_mesh->get_vertices());
    }
    loaded_array = true;
}

void Model::init_camera() {
    camera_distance = bsphere_radius * 1.5f;
    camera_position = center + glm::vec3(0.0f, 0.0f, camera_distance);  // On positive z axis
    camera_direction = glm::normalize(center - camera_position);

    camera = std::make_unique<Camera>(camera_position, camera_direction,
                                      Camera::DEFAULT_UP, Camera::DEFAULT_PHI,
                                      Camera::DEFAULT_THETA, Camera::DEFAULT_SPEED,
                                      Camera::DEFAULT_SENSITIVITY, Camera::DEFAULT_FOV,
                                      camera_distance, center);
    initialized_camera = true;
}

void Model::init_geometries() {
    // 1. Calculate centroids
    centroid /= static_cast<float>(num_vertices);

    // 2. Center the model around the origin
    array_cell_vertices.resize(0);
    for (std::shared_ptr<Mesh>& mesh : array_cell_meshes) {
        mesh->center_mesh(centroid, true);
        array_cell_vertices.push_back(mesh->get_vertices());
        update_max_min_values(mesh);
    }

    canopy_mesh->center_mesh(centroid, true);
    canopy_vertices = canopy_mesh->get_vertices();
    update_max_min_values(canopy_mesh);

    // 3. Calculate bounding box and bounding sphere lengths
    center = glm::vec3((max_values.x + min_values.x) / 2.0f,
                        (max_values.y + min_values.y) / 2.0f,
                        (max_values.z + min_values.z) / 2.0f);
    bbox_length = glm::length(max_values - min_values);
    bsphere_radius = bbox_length;

    initialized_geometries = true;
}

void Model::init_shaders() {
    shaders = std::make_unique<Shader>("../data/shaders/model.vs", "../data/shaders/model.fs");
    initialized_shaders = true;
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
