/* All utilities for calculating the effective irradiance csv */

#pragma once

#include <iostream>
#include <stdlib.h>
#include "Luts.hpp"
#include "model.hpp"
#include "sun_plane.hpp"
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
#include <unordered_set>
#include <unordered_map>
#include <CGAL/Random.h>
#include "time.hpp"

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
     * is slightly shaded, then the entire triangle is considered shaded. Setting precise_shadows is significantly
     * slower and should only be used to generate the final lookup table for the chosen array
     */
    CellIrradianceSim(bool precise_shadows = false) : partial_shadows(precise_shadows) {};

    /** @brief Run a dynamic simulation (Car moves through a route) 
     * @param sun_position_lut: SunPositionLUT csv object describing the path of the sun in the sky
     * @param car_model: Model object describing the car (array and canopy)
     * @param route_lut: RouteLUT csv object describing the points of the route
     * @param speed: Speed of the car in m/s
     * @param direction: Direction of the nose of the car. Must be {"-x", "-y", "+x", "-x"}
     * @param start_time: Starting time of the dynamic simulation
     * @param end_time: Ending time of the dynamic simulation
    */
    void run_dynamic_sim(const std::shared_ptr<SunPositionLUT>& sun_position_lut, const std::shared_ptr<Model>& car_model,
                        const std::shared_ptr<RouteLUT>& route_lut, double speed, std::string direction,
                        const std::shared_ptr<Time>& start_time, const std::shared_ptr<Time>& end_time);

    // Run static simulation
    void run_static_sim(const std::shared_ptr<SunPositionLUT>& sun_position_lut, const std::shared_ptr<Model>& car_model,
                        double bearing, std::string direction);

    // Load an irradiance CSV from a file path
    CellIrradianceSim(std::filesystem::path csv_path);

    // Load an irradiance csv, bearing csv and coordinate csv previously generated from a dynamic simulation
    CellIrradianceSim(std::filesystem::path irradiance_csv_path,
                      std::filesystem::path metadata_csv_path);

    // Write the irradiance CSV from a dynamic simulation
    void write_static_csv(const std::string& csv_name);

    // Write irradiance CSV for a dynamic simulation and a CSV storing the bearing, coordinates, times and
    // sun position cache indices
    void write_dynamic_csv(const std::string& irradiance_csv_name, const std::string& metadata_csv_name);

    // Get a certain value within the csv
    double get_irr_value(const size_t row_idx, const size_t col_idx) const;

    inline double get_min_irradiance_value() {return min_irradiance_value;}
    inline double get_max_irradiance_value() {return max_irradiance_value;}
    inline std::pair<double, double> get_irradiance_limits() {return irradiance_limits;}
    inline std::vector<std::vector<double>> get_irradiance_csv() {return irradiance_csv;}
    inline double get_bearing_value(size_t idx) {return bearings[idx];}
    inline Coord get_coordinate_value(size_t idx) {return coordinates[idx];}
    inline std::string get_time_string_value(size_t idx) {return time_strings[idx];}
    inline size_t get_sun_position_cache_value(size_t idx) {return sun_position_caches[idx];}
private:
    SimType sim_type;
    bool partial_shadows;

    double min_irradiance_value;
    double max_irradiance_value;

    std::pair<double, double> irradiance_limits;

    const int NUM_RAYS = 3000; // Number of rays to generate for each partially shaded triangle
    std::vector<std::vector<double>> irradiance_csv;  // stores the output csv
    std::vector<double> bearings;  // For a dynamic simulation, this stores the bearing of the car at each row
    std::vector<Coord> coordinates;  // For a dynamic simulation, this stores the coordinates of the car at each row
    std::vector<Time> times; // For a dynamic simulation, this stores the times of the car at each row
    std::vector<size_t> sun_position_caches;  // For a dynamic simulation, this stores the sun position row indices
    std::vector<std::string> time_strings;  
    void construct_csv_row(const std::shared_ptr<SunPlane>& sun_plane, const std::shared_ptr<Model>& car_model,
                            const size_t& row_idx, double irradiance, bool precise_shadows);

    double get_bearing(Coord src_coord, Coord dest_coord);
    double get_distance(Coord src_coord, Coord dest_coord);
};
