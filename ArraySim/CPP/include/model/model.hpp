#ifndef MODEL_H
#define MODEL_H

#include <string>
#include "Shader.hpp"
#include "mesh.hpp"
#include "camera.hpp"
#include <filesystem>
#include <memory>

/* Holds the 3D model CAD object formed by both the array cells and canopy */
class Model {
public:
    Model()
        : loaded_canopy(false), loaded_array(false),
        max_values(glm::vec3(std::numeric_limits<float>::lowest())),
        min_values(glm::vec3(std::numeric_limits<float>::max())),
        centroid(glm::vec3(0.0,0.0,0.0)), num_vertices(0) {};
    
    /** @brief Load the canopy .stl file
     * @param path: Full path to a .stl file
     */
    void loadCanopy(const std::filesystem::path& path);

    /** @brief Load the array .stl files
     * @param path: Full path to a directory containing exclusively .stl files
     */
    void loadArray(const std::filesystem::path& path);

    /** @brief Center the model (array and canopy) around 0,0,0 */
    void center_model();

    /** @brief Calculate the centroid of the entire model */
    void calc_centroid();

    bool loaded_canopy;
    bool loaded_array;

    /** @brief Initialize the camera object to view the 3D scene within an OpenGL context */
    void init_camera();

    /** @brief Initialize the shaders for viewing the 3D OpenGL scene */
    void init_shaders();

    /** @brief Draw the canopy and array within an OpenGL context
     * @param window_width: Width of the window in pixels
     * @param window_height: Height of the window in pixels
     * @param colours: Colour to fill in for each array cell
     * @param cell_idx: The array cell referenced in array_cell_meshes[idx] that will be highlighted
     * @param outline: Whether the triangles should be outlined in black. This makes it harder
     * to distinguish the array cells but are necessary to see them at all
     */
    void Draw(double window_width, double window_height, std::vector<glm::vec3> colours, int cell_idx = 1, bool outline = false);

    /** @brief  */
    glm::vec3 get_max_values() const {return max_values;}
    glm::vec3 get_min_values() const {return min_values;}
    glm::vec3 get_center() const {return center;}
    float get_bbox_length() const {return bbox_length;}
    float get_bsphere_radius() const {return bsphere_radius;}
    float get_camera_distance() const {return camera_distance;}

    // For viewing and navigating the scene
    std::shared_ptr<Camera> camera;

    std::shared_ptr<Mesh> get_canopy_mesh() {return canopy_mesh;}
    std::vector<std::shared_ptr<Mesh>> get_array_cell_meshes() {return array_cell_meshes;}

    std::vector<Vertex> get_canopy_vertices() {return canopy_mesh->get_vertices();}
    std::vector<std::vector<Vertex>> get_array_cell_vertices() {return array_cell_vertices;}
private:
    std::shared_ptr<Shader> shaders;
    // A model is composed of many meshes
    std::shared_ptr<Mesh> canopy_mesh;
    std::vector<std::shared_ptr<Mesh>> array_cell_meshes;

    // Max and min coordinate values along each dimension
    glm::vec3 max_values;
    glm::vec3 min_values;

    // Vertices
    std::vector<std::vector<Vertex>> array_cell_vertices;
    std::vector<Vertex> canopy_vertices;

    // Total number of vertices in both the array and canopy
    size_t num_vertices;

    // Centroid of all points from the array and canopy
    glm::vec3 centroid;
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