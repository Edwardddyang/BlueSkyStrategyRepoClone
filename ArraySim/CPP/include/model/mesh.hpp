#ifndef MESH_H
#define MESH_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <string>
#include "Shader.hpp"
#include "stl_reader.h"
#include <filesystem>
#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
};

class Mesh {
public:
    static const int NUM_TRI_VERTS = 3;
    // Mesh data with 
    std::vector<Vertex> vertices;
    std::vector<std::array<size_t, NUM_TRI_VERTS>> faces; // Each face is a triangle
    std::vector<glm::vec3> face_normals; // face_normals[i] is the normal for faces[i]
    std::vector<unsigned int> indices;

    Mesh(const std::filesystem::path& path);

    // Pass in a centroid and vertex count by reference. Used for visualizations
    Mesh(const std::filesystem::path& path, glm::vec3& centroid, size_t& num_vertices);

    void Draw(const std::unique_ptr<Shader> &shader, glm::vec3 fill_colour, bool outline, bool highlight);

    glm::vec3 get_max_values() const {return max_values;}
    glm::vec3 get_min_values() const {return min_values;}

    std::vector<Vertex> get_vertices() {return vertices;}
    // Center the mesh according to the some centroid
    void center_mesh(glm::vec3& centroid, bool update_min_max_values=false);

private:
    // Largest and smallest coordinate values along each dimension
    glm::vec3 max_values;
    glm::vec3 min_values;

    // Vertex buffers
    unsigned int VAO, VBO, EBO;
    void setupMesh();
};

#endif /* MESH_H */