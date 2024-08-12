#include "mesh.hpp"
#include "glad/glad.h"
#include <iostream>
#include "stl_reader.h"

void Mesh::setupMesh() {
    // Create all OpenGL buffers and state for this specific mesh
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    // Load mesh vertices
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

    // Load indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    // Flatten faces vector for the indices vector
    indices.reserve(faces.size() * NUM_TRI_VERTS);
    for (const auto& face : faces) {
        for (size_t i = 0; i < NUM_TRI_VERTS; ++i) {
            if (face[i] > std::numeric_limits<unsigned int>::max()) {
                std::cerr << "Index value too large to fit in an unsigned int: " << face[i] << std::endl;
                // Handle the error appropriately, such as exiting or continuing with a default value.
                continue;
            }
            indices.push_back(static_cast<unsigned int>(face[i]));
        }
    }
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

    // Vertex positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    // Vertex normals
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    glBindVertexArray(0);
}

Mesh::Mesh(const std::filesystem::path& path) {
    try {
        max_values = glm::vec3(std::numeric_limits<float>::lowest());
        min_values = glm::vec3(std::numeric_limits<float>::max());
        stl_reader::StlMesh <float, unsigned int> mesh (path.string());
        size_t num_tris = mesh.num_tris();
        size_t vert_idx = 0;

        // Iterate through triangles
        for(size_t itri = 0; itri < num_tris; ++itri) {
            // Iterate through 3 vertices of triangle
            std::array<size_t, NUM_TRI_VERTS> face = {};
            for(size_t icorner = 0; icorner < NUM_TRI_VERTS; ++icorner) {
                const float* c = mesh.tri_corner_coords(itri, icorner);
                Vertex vertex;
                glm::vec3 vector;
                vector.x = c[0];
                vector.y = c[1];
                vector.z = c[2];
                vertex.position = vector;

                // Update maximum values
                max_values.x = std::max(max_values.x, vector.x);
                max_values.y = std::max(max_values.y, vector.y);
                max_values.z = std::max(max_values.z, vector.z);
                
                // Update minimum values
                min_values.x = std::min(min_values.x, vector.x);
                min_values.y = std::min(min_values.y, vector.y);
                min_values.z = std::min(min_values.z, vector.z);

                face[icorner] = vert_idx;
                vertices.push_back(vertex);
                vert_idx++;
            }
            faces.push_back(face);
        
            const float* n = mesh.tri_normal(itri);
            face_normals.push_back(glm::vec3(n[0], n[1], n[2]));
        }

        assert(face_normals.size() == faces.size() && faces.size() == num_tris);

        // Calculate the normal for each vertex
        // For each vertex:
        // 1. Add up the normals of the faces associated with the vertex
        // 2. Normalize the accumulated normals at each vertex
        size_t num_faces = faces.size();
        for (size_t i=0; i<num_faces; i++) {
            const std::array<size_t, NUM_TRI_VERTS> face = faces[i];
            const glm::vec3 face_normal = face_normals[i];

            vertices[face[0]].normal += face_normal;
            vertices[face[1]].normal += face_normal;
            vertices[face[2]].normal += face_normal;
        }
        for (size_t i=0; i<vert_idx; i++) {
            vertices[i].normal = glm::normalize(vertices[i].normal);
        }
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }

    setupMesh();
}

Mesh::Mesh(const std::filesystem::path& path, glm::vec3& centroid, size_t& num_vertices) {
    try {
        max_values = glm::vec3(std::numeric_limits<float>::lowest());
        min_values = glm::vec3(std::numeric_limits<float>::max());
        stl_reader::StlMesh <float, unsigned int> mesh (path.string());
        size_t num_tris = mesh.num_tris();
        size_t vert_idx = 0;

        // Iterate through triangles
        for(size_t itri = 0; itri < num_tris; ++itri) {
            // Iterate through 3 vertices of triangle
            std::array<size_t, NUM_TRI_VERTS> face = {};
            for(size_t icorner = 0; icorner < NUM_TRI_VERTS; ++icorner) {
                const float* c = mesh.tri_corner_coords(itri, icorner);
                Vertex vertex;
                glm::vec3 vector;
                vector.x = c[0];
                vector.y = c[1];
                vector.z = c[2];
                vertex.position = vector;

                centroid += vector;

                face[icorner] = vert_idx;
                vertices.push_back(vertex);
                vert_idx++;
                num_vertices++;
            }
            faces.push_back(face);
        
            const float* n = mesh.tri_normal(itri);
            face_normals.push_back(glm::vec3(n[0], n[1], n[2]));
        }

        assert(face_normals.size() == faces.size() && faces.size() == num_tris);

        // Calculate the normal for each vertex
        // For each vertex:
        // 1. Add up the normals of the faces associated with the vertex
        // 2. Normalize the accumulated normals at each vertex
        size_t num_faces = faces.size();
        for (size_t i=0; i<num_faces; i++) {
            const std::array<size_t, NUM_TRI_VERTS> face = faces[i];
            const glm::vec3 face_normal = face_normals[i];

            vertices[face[0]].normal += face_normal;
            vertices[face[1]].normal += face_normal;
            vertices[face[2]].normal += face_normal;
        }
        for (size_t i=0; i<vert_idx; i++) {
            vertices[i].normal = glm::normalize(vertices[i].normal);
        }
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }

    setupMesh();
}

void Mesh::center_mesh(glm::vec3& centroid, bool update_min_max_values) {
    for (Vertex& vert : vertices) {
        vert.position -= centroid;

        if (update_min_max_values) {
            // Update the minimum and maximum values
            max_values.x = std::max(max_values.x, vert.position.x);
            max_values.y = std::max(max_values.y, vert.position.y);
            max_values.z = std::max(max_values.z, vert.position.z);
            
            // Update minimum values
            min_values.x = std::min(min_values.x, vert.position.x);
            min_values.y = std::min(min_values.y, vert.position.y);
            min_values.z = std::min(min_values.z, vert.position.z);
        }
    }
}

void Mesh::Draw(std::shared_ptr<Shader> &shader) 
{
    // draw mesh
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}  