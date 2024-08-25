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

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Point_3 Point;
typedef K::Point_2 Point2;
typedef K::Triangle_3 Triangle;
typedef K::Triangle_2 Triangle2;
typedef K::Direction_3 Direction;
typedef K::Segment_3 Segment;
typedef K::Vector_3 Vector;
typedef std::vector<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;
typedef CGAL::Polygon_2<K> Polygon2;
typedef Tree::Primitive_id Primitive_id;
typedef boost::optional< Tree::Intersection_and_primitive_id<CGAL::Ray_3<K> >::Type > Ray_intersection;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> PrimitiveI;
typedef CGAL::AABB_traits<K, PrimitiveI> AABB_traits_i;
typedef CGAL::AABB_tree<AABB_traits_i> Tree_i;
typedef CGAL::Alpha_shape_vertex_base_2<K> Vb;
typedef CGAL::Alpha_shape_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds> Triangulation;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay2;
typedef CGAL::Alpha_shape_2<Triangulation> Alpha_shape;
typedef CGAL::Side_of_triangle_mesh<Polyhedron, K> Side_of_triangle_mesh;
typedef Delaunay2::Finite_faces_iterator Face_iterator;

enum class SimType {
    DYNAMIC,
    STATIC
};

class CellIrradianceSim {
public:
    static glm::vec3 compute_triangle_norm(const Triangle& triangle);

    CellIrradianceSim(bool precise_shadows = false) : partial_shadows(precise_shadows) {};

    // Run dynamic simulation
    void run_dynamic_sim(std::shared_ptr<SunPositionLUT> sun_position_lut, std::shared_ptr<Model> car_model,
                        std::shared_ptr<RouteLUT> route_lut, double speed, std::string direction,
                        std::string start_time, std::string end_time);

    // Run static simulation
    void run_static_sim(std::shared_ptr<SunPositionLUT> sun_position_lut, std::shared_ptr<Model> car_model,
                        double bearing, std::string direction);

    // Create an irradiance csv based on the given configurations
    CellIrradianceSim(std::shared_ptr<SunPositionLUT> sun_position_lut, std::shared_ptr<Model> car_model,
                  double bearing, std::string direction, bool precise_shadows = true);

    // Load a CSV from a file path
    CellIrradianceSim(std::filesystem::path csv_path);
    void write_csv(const std::string& csv_name);

    // Get a certain value within the csv
    double get_irr_value(const size_t row_idx, const size_t col_idx) const;

    inline double get_min_irradiance_value() {return min_irradiance_value;}
    inline double get_max_irradiance_value() {return max_irradiance_value;}
    inline std::pair<double, double> get_irradiance_limits() {return irradiance_limits;}
    inline std::vector<std::vector<double>> get_irradiance_csv() {return irradiance_csv;}
private:
    SimType sim_type;
    bool partial_shadows;

    std::shared_ptr<SunPositionLUT> sun_position;
    std::shared_ptr<Model> car;
    double min_irradiance_value;
    double max_irradiance_value;

    std::pair<double, double> irradiance_limits;

    const int NUM_RAYS = 3000; // Number of rays to generate for each partially shaded triangle
    std::vector<std::vector<double>> irradiance_csv;  // stores the output csv
    void construct_csv_row(std::shared_ptr<SunPlane>& sun_plane, size_t row_idx, double irradiance, bool precise_shadows);

    double get_bearing(Coord src_coord, Coord dest_coord);
    double get_distance(Coord src_coord, Coord dest_coord);
};
