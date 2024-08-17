#include "irradiance_csv.hpp"
#include "QuickHull.hpp"
#include <clipper2/clipper.h>
#include <list>
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

IrradianceCSV::IrradianceCSV(std::shared_ptr<SunPositionLUT> sun_position_lut, std::shared_ptr<Model> car_model,
                            double bearing, std::string direction) {
    sun_position = sun_position_lut;
    car = car_model;

    car->calc_centroid();
    car->center_model();
    car->init_camera();  // Get bounding box characteristics

    size_t num_sun_positions = sun_position_lut->get_num_rows();
    for (size_t i=0; i<num_sun_positions; i++) {
        double azimuth = sun_position_lut->get_azimuth_value(i);
        double elevation = sun_position_lut->get_elevation_value(i);
        double irradiance = sun_position_lut->get_irradiance_value(i);

        // Create sun plane
        std::shared_ptr<SunPlane> sun_plane = std::make_shared<SunPlane>(
            azimuth, elevation, direction, bearing, car->get_min_values(),
            car->get_max_values()
        );

        std::cout << "INDEX: " << i << std::endl;
        // Compute the shadow on the array cells from the canopy
        compute_canopy_shadow(sun_plane);
    }
}

glm::vec3 IrradianceCSV::findOrthogonalVector(const glm::vec3& normal) {
    if (std::abs(normal.x) > std::abs(normal.y)) {
        return glm::normalize(glm::vec3(-normal.z, 0, normal.x));
    } else {
        return glm::normalize(glm::vec3(0, normal.z, -normal.y));
    }
}

void export_to_obj(const std::vector<Triangle>& triangles, const std::string& filename) {
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "Cannot open file for writing: " << filename << std::endl;
        return;
    }

    int vertex_idx = 1;
    for (const Triangle& tri : triangles) {
        file << "v " << tri.vertex(0).x() << " " << tri.vertex(0).y() << " " << tri.vertex(0).z() << std::endl;
        file << "v " << tri.vertex(1).x() << " " << tri.vertex(1).y() << " " << tri.vertex(1).z() << std::endl;
        file << "v " << tri.vertex(2).x() << " " << tri.vertex(2).y() << " " << tri.vertex(2).z() << std::endl;
        file << "f " << vertex_idx << " " << vertex_idx + 1 << " " << vertex_idx + 2 << std::endl;
        vertex_idx += 3;
    }
    file.close();
}

void IrradianceCSV::compute_canopy_shadow(std::shared_ptr<SunPlane>& sun_plane) {
    std::vector<Vertex> canopy_vertices = car->get_canopy_mesh()->get_vertices();
    std::vector<std::vector<Vertex>> array_cells = car->get_array_cell_vertices();
    glm::vec3& sun_plane_normal = sun_plane->normal;

    // Construct AABB tree of the array cells.
    // Note: triangle index refers to the index of a triangle in array_triangles or
    // flattened_array_triangles. Cell index refers to the index of a cell in
    // array_cells
    std::vector<Triangle> array_triangles;
    std::vector<Triangle2> flattened_array_triangles; // Flattened onto the XY plane
    std::unordered_map<size_t, size_t> triangle_to_cell;  // Map of triangle indices to array cell indices
    size_t num_array_cells = array_cells.size();
    size_t triangle_idx = 0;
    for (size_t cell_idx=0; cell_idx<num_array_cells; cell_idx++) {
        for (size_t vert_idx=0; vert_idx<array_cells[cell_idx].size(); vert_idx+=3) {
            Point point1(array_cells[cell_idx][vert_idx].position.x,
                        array_cells[cell_idx][vert_idx].position.y,
                        array_cells[cell_idx][vert_idx].position.z);
            Point point2(array_cells[cell_idx][vert_idx+1].position.x,
                        array_cells[cell_idx][vert_idx+1].position.y,
                        array_cells[cell_idx][vert_idx+1].position.z);
            Point point3(array_cells[cell_idx][vert_idx+2].position.x,
                        array_cells[cell_idx][vert_idx+2].position.y,
                        array_cells[cell_idx][vert_idx+2].position.z);

            Point2 point1_2d(point1.x(), point1.y());
            Point2 point2_2d(point2.x(), point2.y());
            Point2 point3_2d(point3.x(), point3.y());

            flattened_array_triangles.push_back(Triangle2(point1_2d, point2_2d, point3_2d));
            array_triangles.push_back(Triangle(point1, point2, point3));
            triangle_to_cell.insert({triangle_idx, cell_idx});
            triangle_idx++;
        }
    }
    Tree array_tree(array_triangles.begin(), array_triangles.end());

    // Construct AABB tree of the canopy
    std::vector<Triangle> canopy_triangles;
    size_t num_canopy_points = canopy_vertices.size();
    for (int vert_idx=0; vert_idx<num_canopy_points; vert_idx+=3) {
        Point point1(canopy_vertices[vert_idx].position.x,
                    canopy_vertices[vert_idx].position.y,
                    canopy_vertices[vert_idx].position.z);
        Point point2(canopy_vertices[vert_idx+1].position.x,
                    canopy_vertices[vert_idx+1].position.y,
                    canopy_vertices[vert_idx+1].position.z);
        Point point3(canopy_vertices[vert_idx+2].position.x,
                    canopy_vertices[vert_idx+2].position.y,
                    canopy_vertices[vert_idx+2].position.z);
        canopy_triangles.push_back(Triangle(point1, point2, point3));
    }
    Tree canopy_tree(canopy_triangles.begin(), canopy_triangles.end());

    // Cast rays from all points of the canopy towards the array and query the intersection points
    std::vector<Point> projected_canopy_points;
    size_t num_projected_canopy_points = 0;
    // Direction of the ray must run towards the array
    Direction direction(sun_plane_normal.x * -1.0f, sun_plane_normal.y * -1.0f, sun_plane_normal.z * -1.0f);
    for (size_t idx=0; idx < num_canopy_points; idx++) {
        Vertex canopy_vertex = canopy_vertices[idx];
        Point ray_origin(canopy_vertex.position.x, canopy_vertex.position.y, canopy_vertex.position.z);
        Ray canopy_ray = Ray(ray_origin, direction);
        boost::optional<Primitive_id> intersection = array_tree.first_intersected_primitive(canopy_ray);
        // Get intersection point
        if (intersection) {
            Iterator it = *intersection;
            Triangle triangle = *it;
            // Filter out false positives
            auto result = CGAL::intersection(canopy_ray, triangle);
            if (result) {
                if(const Point* p = boost::get<Point>(&*result)) {
                    projected_canopy_points.push_back(*p);
                    num_projected_canopy_points++;
                } else if (const Segment* s = boost::get<Segment>(&*result)) {
                    // Intersection is a segment, get source (beginning) point of segment
                    projected_canopy_points.push_back(s->source());
                    num_projected_canopy_points++;
                }
            }
        }
    }

    // Flatten the projected canopy points down to the xy plane and compute its convex hull
    std::vector<Point2> flattened_canopy_points;
    std::vector<Point2> canopy_convex_hull;
    std::cout << num_projected_canopy_points << " " << projected_canopy_points.size() << std::endl;
    for (size_t vert_idx=0; vert_idx<num_projected_canopy_points; vert_idx++) {
        Point p = projected_canopy_points[vert_idx];
        flattened_canopy_points.push_back(Point2(p.x(), p.y()));
    }
    CGAL::convex_hull_2(flattened_canopy_points.begin(),
                        flattened_canopy_points.end(),
                        std::back_inserter(canopy_convex_hull));
    Delaunay2 convex_canopy_dt;
    convex_canopy_dt.insert(canopy_convex_hull.begin(), canopy_convex_hull.end());

    // Query flattened array triangle overlap with canopy convex hull to determine if a triangle is
    // completely shaded, not shaded, or partially shaded
    std::unordered_set<size_t> partially_shaded_triangles;
    std::unordered_set<size_t> completely_shaded_triangles;
    std::unordered_map<size_t, double> triangle_shaded_areas;  // Triangle index -> its shaded area
    std::unordered_map<size_t, double> triangle_areas;  // Triangle index -> its area
    size_t num_array_triangles = flattened_array_triangles.size();
    int num_tri = 0;
    for (size_t tri_idx=0; tri_idx<num_array_triangles; tri_idx++) {
        triangle_shaded_areas.insert({tri_idx, 0.0});
    }
    std::cout << "INITIALIZED" << std::endl;
    for (size_t tri_idx=0; tri_idx<num_array_triangles; tri_idx++) {
        Triangle2 tri = flattened_array_triangles[tri_idx];
        bool intersects = false;
        for (Face_iterator fit = convex_canopy_dt.finite_faces_begin(); fit != convex_canopy_dt.finite_faces_end(); ++fit) {
            Point2 p1 = fit->vertex(0)->point();
            Point2 p2 = fit->vertex(1)->point();
            Point2 p3 = fit->vertex(2)->point();

            CGAL::Object result = CGAL::intersection(tri, Triangle2(p1, p2, p3));
            if (const Triangle2* int_tri = CGAL::object_cast<Triangle2>(&result)) {
                triangle_shaded_areas[tri_idx] += std::abs(CGAL::area(int_tri->vertex(0),
                                                                     int_tri->vertex(1),
                                                                     int_tri->vertex(2)));
                intersects = true;
            }
        }
        num_tri = intersects ? num_tri + 1 : num_tri;
    }
    int num_completely_shaded_triangles = 0;
    for (auto it=triangle_shaded_areas.begin(); it != triangle_shaded_areas.end(); it++) {
        double shaded_area = it->second;  // Shaded area on the xy plane. Not representative of actual shaded area
        if (shaded_area > 0.0) {
            Triangle2 tri = flattened_array_triangles[it->first];
            double tri_area = std::abs(CGAL::area(tri[0], tri[1], tri[2]));
            triangle_areas[it->first] = tri_area;
            if (std::abs(shaded_area - tri_area) < 1.0 || shaded_area > tri_area) {
                num_completely_shaded_triangles++;
                completely_shaded_triangles.insert(it->first);
            } else {
                partially_shaded_triangles.insert(it->first);
            }
        }
    }
    std::cout << "NUM TRIANGLE INTERSECTIONS: " << num_tri << " | Total completely shaded triangles " << num_completely_shaded_triangles << " | Total partially shaded triangles " << partially_shaded_triangles.size() << std::endl;

    // For each partially shaded triangle, cast NUM_RAYS from the triangle towards the sun plane. The numerical approximation
    // for shaded area is given by (Num rays intersecting with canopy / NUM_RAYS) * triangle area
    Direction normal_direction(sun_plane_normal.x, sun_plane_normal.y, sun_plane_normal.z);
    auto random_gen = CGAL::Random(0);
    for (auto it = partially_shaded_triangles.begin(); it != partially_shaded_triangles.end(); it++) {
        CGAL::Random_points_in_triangle_3<Point> generator(array_triangles[*it], random_gen);

        int num_rays_intersected = 0;
        for (int i=0; i<NUM_RAYS; i++) {
            Point origin_point = *generator++;
            Ray triangle_ray(origin_point, normal_direction);

            if (canopy_tree.any_intersected_primitive(triangle_ray)) {
                num_rays_intersected++;
            }
        }

        std::cout << "PARTIALLY SHADED TRIANGLE AREA: " << triangle_areas[*it] << " | SHADED AREA: " << ((num_rays_intersected /(double) NUM_RAYS)) * triangle_areas[*it] << " | NUM RAYS INTERSECTED: " << num_rays_intersected << std::endl;
    }
}
