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

/** Holds the position and normal vector of a point in a CAD mesh.
 * The normal is calculated by adding up the normal vectors of the faces
 * that the point is apart of.
 */
struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
};

/** Representation of a 3D CAD mesh */
class Mesh {
public:
    static const int NUM_TRI_VERTS = 3;
    
    // All 3D vertices extracted from the mesh
    std::vector<Vertex> vertices;

    // The faces extracted from the mesh. We only deal with triangulated meshes.
    std::vector<std::array<size_t, NUM_TRI_VERTS>> faces;

    // The normalized normal vectors for each face. Note that
    // face_normals[i] is the normal for faces[i]
    std::vector<glm::vec3> face_normals;

    // The flattened array of the faces vector. Used to make OpenGL draw calls
    std::vector<unsigned int> indices;

    /** @brief Constructor that loads a mesh from a file. Also updates a centroid variable passed
     * in from the caller
     * @param path: Absolute path to a 3D triangulated mesh
     * @param centroid: The 3D centroid point to update as a pointer. This will be modified
     */
    Mesh(const std::filesystem::path& path, glm::vec3* centroid = nullptr);

    /** @brief OpenGL draw call to draw both the canopy and array
     * @param shader: Shader object holding both the fragment and vertex shader
     * @param fill_colour: The colour that will fill the drawn shape
     * @param outline: Whether the mesh being drawn should be outlined in black
     * @param highlight: Whether the mesh being drawn should be highlighted in another colour
     */
    void Draw(const std::unique_ptr<Shader> &shader, glm::vec3 fill_colour, bool outline, bool highlight);

    inline glm::vec3 get_max_values() const {return max_values;}
    inline glm::vec3 get_min_values() const {return min_values;}
    inline size_t get_num_vertices() const {return num_vertices;}
    inline std::vector<Vertex> get_vertices() {return vertices;}

    /** @brief Center the mesh (both canopy and array) around the origin by subtracting the centroid from all points
     * @param centroid: Centroid of the canopy and array
     * @param update_min_max_values: Whether to update the minimum and maximum values of the model after shifting
     */
    void center_mesh(glm::vec3& centroid, bool update_min_max_values=false);

private:
    // Largest and smallest coordinate values along each dimension
    glm::vec3 max_values;
    glm::vec3 min_values;

    // Total number of vertices
    size_t num_vertices = 0;

    // Vertex buffers
    unsigned int VAO, VBO, EBO;

    // Colour for highlighting the mesh
    const glm::vec3 highlight_colour = glm::vec3(0.65, 0.16, 0.16);

    /** @brief Setup the mesh by creating the vertex buffers */
    void setupMesh();
};

#endif /* MESH_H */