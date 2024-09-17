/* All utilities for calculating the effective irradiance csv */

#pragma once

#include <iostream>
#include <stdlib.h>
#include "Luts.hpp"
#include "model.hpp"
#include "sun_plane.hpp"
#include <unordered_set>
#include <unordered_map>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Random.h>

// Use CGAL's exact kernel
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Ray_3 Ray;
typedef K::Point_3 Point;
typedef K::Triangle_3 Triangle;
typedef K::Direction_3 Direction;
typedef std::vector<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;
typedef Tree::Primitive_id Primitive_id;

class CellIrradianceSim {
public:
    // Supported simulation types. Static means that the car is stationary in one location.
    // Dynamic means that the car is moving through a route
    enum class SimType {
        DYNAMIC,
        STATIC
    };

    /** @brief Compute the normal vector of a triangle's centroid in 3D space
     * @param triangle: CGAL triangle with points in 3D space
     * @return glm vec3 type
     */
    static glm::vec3 compute_triangle_norm(const Triangle& triangle);

    /** @brief Constructor with default imprecise shadow calculations. Specifically, if a triangle in a cell stl
     * is slightly shaded, then the entire triangle is considered shaded. Setting precise_shadows to true is
     * significantly slower and should only be used to generate the final lookup table for the chosen array
     */
    CellIrradianceSim(std::shared_ptr<Model> model, bool precise_shadows = false);

    /** @brief Run a dynamic simulation (Car moves through a route) and create an irradiance csv
     * @param sun_position_lut: SunPositionLUT csv object describing the path of the sun in the sky
     * @param route_lut: RouteLUT csv object describing the points of the route
     * @param speed: Speed of the car in m/s
     * @param direction: Axis that the nose of the car points in. Must be one of {"-x", "-y", "+x", "+y"}
     * @param start_time: Starting time of the dynamic simulation
     * @param end_time: Ending time of the dynamic simulation
    */
    void run_dynamic_sim(const std::unique_ptr<SunPositionLUT>& sun_position_lut,
                        const std::unique_ptr<RouteLUT>& route_lut, double speed, std::string direction,
                        const std::unique_ptr<Time>& start_time, const std::unique_ptr<Time>& end_time);

    /** @brief Run a static simulation (Car is stationary) and create an irradiance csv
     * @param sun_position_lut: SunPositionLUT csv object describing the path of the sun in the sky
     * @param bearing: Bearing of the nose of the car in degrees
     * @param direction: Axis that the nose of the car points in. Must be one of {"-x", "-y", "+x", "+y"}
     */
    void run_static_sim(const std::unique_ptr<SunPositionLUT>& sun_position_lut,double bearing, std::string direction);

    /** @brief Write the static csv created after run_static_sim(...) */
    void write_static_csv(const std::filesystem::path& csv_path) const;
    /** @brief Write the dynamic csvs created after run_dynamic_sim(...) */
    void write_dynamic_csv(const std::filesystem::path& irr_csv_path, const std::filesystem::path& metadata_csv_path) const;
private:
    // Store the results of the simulation
    std::shared_ptr<CellIrradianceCsv> cell_irradiance_csv = nullptr;

    // The car (canopy + array) being simulated
    std::shared_ptr<Model> car_model = nullptr;
    std::vector<Vertex> canopy_vertices;
    std::vector<std::vector<Vertex>> array_cells;

    // Type of simulation ran
    SimType sim_type;

    // Whether the partially shaded cells were calculated accurately or approximated
    bool partial_shadows;

    // Number of rays to generate for each partially shaded triangle
    const int NUM_RAYS = 3000;

    /** @brief Create a row of the output irradiance csv
     * Note: Represents the effective irradiance on each cell for a specific sun position
     * 
     * @param sun_plane: The position of the sun
     */
    std::vector<double> construct_csv_row(const std::unique_ptr<SunPlane>& sun_plane);

    double get_bearing(Coord src_coord, Coord dest_coord);
    double get_distance(Coord src_coord, Coord dest_coord);
};
