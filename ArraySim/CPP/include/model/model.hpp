#ifndef MODEL_H
#define MODEL_H

#include <string>
#include "Shader.hpp"
#include "Globals.h"
#include "mesh.hpp"
#include "camera.hpp"
#include <filesystem>
#include <memory>

/* Holds the 3D model CAD object formed by both the array cells and canopy 
 * Standard instantiation for drawing is as follows:
 * loadCanopy()
 * loadArray()
 * init_geometries()
 * init_camera()
 * init_shaders()
*/
class Model {
public:
    /** @brief Default constructor */
    Model()
        : loaded_canopy(false), loaded_array(false),
        max_values(glm::vec3(std::numeric_limits<float>::lowest())),
        min_values(glm::vec3(std::numeric_limits<float>::max())),
        centroid(glm::vec3(0.0,0.0,0.0)), num_vertices(0) {};
    
    /** @brief Load the canopy .stl file
     * @param path: Absolute path to a .stl file
     */
    void loadCanopy(const std::filesystem::path& path);

    /** @brief Load the array .stl files
     * @param path: Absolute path to a directory containing exclusively cell .stl files.
     * 
     * Note: If the directory contains the canopy as well, then CellIrradianceSim will not
     * work lol
     */
    void loadArray(const std::filesystem::path& path);

    /** @brief Initialize geometries
     * - Calculate the centroid
     * - Center the model around the origin
     * - Calculate the bounding box and bounding sphere lengths
     */
    void init_geometries();

    /** @brief Initialize the camera object to view the 3D scene within an OpenGL context */
    void init_camera();

    /** @brief Initialize the fragment and vertex shaders for viewing the 3D OpenGL scene */
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

    // Getters
    inline glm::vec3 get_max_values() const {return max_values;}
    inline glm::vec3 get_min_values() const {return min_values;}
    inline glm::vec3 get_center() const {return center;}
    inline float get_bbox_length() const {return bbox_length;}
    inline float get_bsphere_radius() const {return bsphere_radius;}
    inline float get_camera_distance() const {return camera_distance;}
    inline std::shared_ptr<Mesh> get_canopy_mesh() {return canopy_mesh;}
    inline std::vector<std::shared_ptr<Mesh>> get_array_cell_meshes() {return array_cell_meshes;}
    inline std::vector<Vertex> get_canopy_vertices() {
        RUNTIME_ASSERT(canopy_mesh != nullptr, "Canopy Mesh not loaded. Did you call loadCanopy()?");
        return canopy_mesh->get_vertices();
    }
    inline std::vector<std::vector<Vertex>> get_array_cell_vertices() {return array_cell_vertices;}

    // Camera object used to navigate the CAD model within an OpenGL context
    std::unique_ptr<Camera> camera;

    // Load statuses
    bool loaded_canopy = false;
    bool loaded_array = false;
    bool initialized_geometries = false;
    bool initialized_camera = false;
    bool initialized_shaders = false;
private:
    // Shader used to draw the CAD model in an OpenGL context
    std::unique_ptr<Shader> shaders;

    // Mesh representations for the canopy and array cells
    std::shared_ptr<Mesh> canopy_mesh;
    std::vector<std::shared_ptr<Mesh>> array_cell_meshes;

    // Max and min coordinate values along each cartesian dimension
    glm::vec3 max_values;
    glm::vec3 min_values;

    // Vertices making up the canopy and array cells
    std::vector<std::vector<Vertex>> array_cell_vertices;
    std::vector<Vertex> canopy_vertices;

    // Total number of vertices from both the array and canopy
    size_t num_vertices;

    // Centroid of the entire array+canopy model
    glm::vec3 centroid;

    // Center of the rectangular bounding box defined by max_values and min_values
    glm::vec3 center;

    // Diagonal length of the bounding box
    float bbox_length;

    // Radius of the bounding sphere
    float bsphere_radius;

    // The scalar distance of the camera relative to the center of the bounding box
    float camera_distance;

    // Position of the camera
    glm::vec3 camera_position;

    // Direction that the camera is pointing in
    glm::vec3 camera_direction;

    // Fill colour for the canopy and array (white)
    const glm::vec3 fill_colour = glm::vec3(1.0,1.0,1.0);

    // Helper to update the max_values, min_values when loading a new mesh
    void update_max_min_values(const std::shared_ptr<Mesh>& mesh);
};

#endif /* MODEL_H */