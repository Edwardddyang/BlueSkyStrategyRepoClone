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
    glm::vec3 max_values = car->get_max_values();
    glm::vec3 min_values = car->get_min_values();
    std::cout << "MAX VALUES: " << max_values.x << " " << max_values.y << " " << max_values.z << std::endl;
    std::cout << "MIN VALUES: " << min_values.x << " " << min_values.y << " " << min_values.z << std::endl;

    glm::vec3 max_values_canopy = car->get_canopy_mesh()->get_max_values();
    glm::vec3 min_values_canopy = car->get_canopy_mesh()->get_min_values();
    std::cout << "CANOPY MAX VALUES: " << max_values_canopy.x << " " << max_values_canopy.y << " " << max_values_canopy.z << std::endl;
    std::cout << "CANOPY MIN VALUES: " << min_values_canopy.x << " " << min_values_canopy.y << " " << min_values_canopy.z << std::endl;
    
    std::vector<std::shared_ptr<Mesh>> array_meshes = car->get_array_cell_meshes();
    glm::vec3 max_array_values = glm::vec3(std::numeric_limits<float>::lowest());
    glm::vec3 min_array_values = glm::vec3(std::numeric_limits<float>::max());

    for (int i=0; i<array_meshes.size(); i++) {
        glm::vec3 maxd = array_meshes[i]->get_max_values();
        glm::vec3 mind = array_meshes[i]->get_min_values();
        max_array_values.x = std::max(max_array_values.x, maxd.x);
        max_array_values.y = std::max(max_array_values.y, maxd.y);
        max_array_values.z = std::max(max_array_values.z, maxd.z);

        min_array_values.x = std::min(min_array_values.x, mind.x);
        min_array_values.y = std::min(min_array_values.y, mind.y);
        min_array_values.z = std::min(min_array_values.z, mind.z);
    }

    std::cout << "ARRAY MAX VALUES: " << max_array_values.x << " " << max_array_values.y << " " << max_array_values.z << std::endl;
    std::cout << "ARRAY MIN VALUES: " << min_array_values.x << " " << min_array_values.y << " " << min_array_values.z << std::endl;

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


// Function to compute the 2D convex hull (Andrew's monotone chain algorithm)
std::vector<glm::vec2> IrradianceCSV::convexHull2D(const std::vector<glm::vec2>& points) {
    auto cmp = [](const glm::vec2& a, const glm::vec2& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    };

    std::vector<glm::vec2> sortedPoints = points;
    std::sort(sortedPoints.begin(), sortedPoints.end(), cmp);

    std::vector<glm::vec2> lowerHull, upperHull;

    // Lower hull
    for (const auto& p : sortedPoints) {
        while (lowerHull.size() >= 2 && (lowerHull[lowerHull.size() - 1].x - lowerHull[lowerHull.size() - 2].x) * (p.y - lowerHull[lowerHull.size() - 1].y) -
                                        (lowerHull[lowerHull.size() - 1].y - lowerHull[lowerHull.size() - 2].y) * (p.x - lowerHull[lowerHull.size() - 1].x) <= 0) {
            lowerHull.pop_back();
        }
        lowerHull.push_back(p);
    }

    // Upper hull
    for (auto it = sortedPoints.rbegin(); it != sortedPoints.rend(); ++it) {
        const auto& p = *it;
        while (upperHull.size() >= 2 && (upperHull[upperHull.size() - 1].x - upperHull[upperHull.size() - 2].x) * (p.y - upperHull[upperHull.size() - 1].y) -
                                        (upperHull[upperHull.size() - 1].y - upperHull[upperHull.size() - 2].y) * (p.x - upperHull[upperHull.size() - 1].x) <= 0) {
            upperHull.pop_back();
        }
        upperHull.push_back(p);
    }

    // Remove the last point of each half because it's repeated
    lowerHull.pop_back();
    upperHull.pop_back();

    // Concatenate lower and upper hulls
    lowerHull.insert(lowerHull.end(), upperHull.begin(), upperHull.end());

    return lowerHull;
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

double manual_area(const Point2& p1, const Point2& p2, const Point2& p3) {
    return 0.5 * std::abs(p1.x() * (p2.y() - p3.y()) + p2.x() * (p3.y() - p1.y()) + p3.x() * (p1.y() - p2.y()));
}

void IrradianceCSV::compute_canopy_shadow(std::shared_ptr<SunPlane>& sun_plane) {
    // Project canopy points onto the plane of the sun
    // https://stackoverflow.com/questions/23472048/projecting-3d-points-to-2d-plane
    std::vector<Vertex> canopy_vertices = car->get_canopy_mesh()->get_vertices();
    std::vector<std::vector<Vertex>> array_vertices = car->get_array_cell_vertices();
    // std::vector<glm::vec2> projected_canopy_vertices;
    // projected_canopy_vertices.reserve(canopy_vertices.size());
    glm::vec3& sun_plane_normal = sun_plane->normal;
    
    // // Define an orthonormal basis for the sun plane
    glm::vec3 basis_vec_1 = findOrthogonalVector(sun_plane_normal);
    // glm::vec3 basis_vec_2 = glm::normalize(glm::cross(sun_plane_normal, basis_vec_1));

    // // Now that I have an orthonormal basis for the 3D plane i.e. Two orthogonal 3D vectors,
    // // then I project the vector = (point on plane -> point to project) onto the plane in order
    // // to get the coordinates of the projected point with respect to the orthonormal basis.
    // // This effectively creates a 2D point
    // for (const auto& point : canopy_vertices) {
    //     glm::vec3 p = point.position - sun_plane->point;
    //     double x2D = glm::dot(p, basis_vec_1);
    //     double y2D = glm::dot(p, basis_vec_2);
    //     projected_canopy_vertices.push_back(glm::vec2(x2D, y2D));
    // }
    // // Compute convex hull projection of canopy onto plane
    // std::vector<quickhull::Vector3<float>> qhPoints;
    // for (const glm::vec2& p : projected_canopy_vertices) {
    //     qhPoints.emplace_back(p.x, p.y, 0.0);  // Attach a 3rd dimension in order to get quickhull to work and recognize that 
    //                                            // the points are coplanar
    // }

    // std::vector<glm::vec2> homemade_convex_hull = convexHull2D(projected_canopy_vertices);

    // // Get the 3d points of the convex hull by multiplying with the orthonormal basis again
    // std::vector<glm::vec3> convex_hull_3d;
    // convex_hull_3d.reserve(homemade_convex_hull.size());
    // for (const glm::vec2& p : homemade_convex_hull) {
    //     convex_hull_3d.push_back(p.x * basis_vec_1 + p.y * basis_vec_2 + sun_plane->point);
    // }

    // Construct AABB tree of the array cells
    std::vector<Triangle> array_triangles;
    std::vector<Triangle2> flattened_array_points;
    for (int j=0; j<array_vertices.size(); j++) {
        for (int i=0; i<array_vertices[j].size(); i+=3) {
            Point point1(array_vertices[j][i].position.x, array_vertices[j][i].position.y, array_vertices[j][i].position.z);
            Point point2(array_vertices[j][i+1].position.x, array_vertices[j][i+1].position.y, array_vertices[j][i+1].position.z);
            Point point3(array_vertices[j][i+2].position.x, array_vertices[j][i+2].position.y, array_vertices[j][i+2].position.z);

            Point2 point1_2d(array_vertices[j][i].position.x, array_vertices[j][i].position.y);
            Point2 point2_2d(array_vertices[j][i+1].position.x, array_vertices[j][i+1].position.y);
            Point2 point3_2d(array_vertices[j][i+2].position.x, array_vertices[j][i+2].position.y);

            flattened_array_points.push_back(Triangle2(point1_2d, point2_2d, point3_2d));
            array_triangles.push_back(Triangle(point1, point2, point3));
        }
    }
    Tree array_tree(array_triangles.begin(), array_triangles.end());

    // Construct AABB tree of the canopy
    std::vector<Triangle> canopy_triangles;
    for (int j=0; j<canopy_vertices.size(); j+=3) {
        Point point1(canopy_vertices[j].position.x, canopy_vertices[j].position.y, canopy_vertices[j].position.z);
        Point point2(canopy_vertices[j+1].position.x, canopy_vertices[j+1].position.y, canopy_vertices[j+1].position.z);
        Point point3(canopy_vertices[j+2].position.x, canopy_vertices[j+2].position.y, canopy_vertices[j+2].position.z);

        canopy_triangles.push_back(Triangle(point1, point2, point3));
    }
    Tree canopy_tree(canopy_triangles.begin(), canopy_triangles.end());

    std::vector<Ray> canopy_rays;
    // Direction of the ray must run towards the array
    Direction direction(sun_plane_normal.x * -1.0f, sun_plane_normal.y * -1.0f, sun_plane_normal.z * -1.0f);
    for (const Vertex& canopy_vertex : canopy_vertices) {
        Point ray_origin(canopy_vertex.position.x, canopy_vertex.position.y, canopy_vertex.position.z);
        canopy_rays.push_back(Ray(ray_origin, direction));
    }

    // Cast all rays from the canopy towards the array and query intersection points
    int inner_count = 0;
    int count = 0;
    std::vector<Point> projected_canopy_points;
    for (const Ray& c_ray : canopy_rays) {
        boost::optional<Primitive_id> intersection = array_tree.first_intersected_primitive(c_ray);

        // Get intersection point
        if (intersection) {
            Iterator it = *intersection;
            Triangle triangle = *it;

            auto result = CGAL::intersection(c_ray, triangle);
            if (result) {
                // Check if the result is a point
                if(const Point* p = boost::get<Point>(&*result)) {
                    // p is the intersection point
                    count++;
                    projected_canopy_points.push_back(*p);
                } else if (const Segment* s = boost::get<Segment>(&*result)) {
                    // Intersection is a segment, get source (beginning) point of segment
                    count++;
                    projected_canopy_points.push_back(s->source());
                }
            }
        }
    }
    std::cout << "NUM INTERSECTIONS OF THE CANOPY WITH THE ARRAY: " << projected_canopy_points.size() << std::endl;

    // Get convex hull of projected canopy points
    Polyhedron poly;
    CGAL::convex_hull_3(projected_canopy_points.begin(), projected_canopy_points.end(), poly);
    Vector expansion_vector(basis_vec_1.x*5.0, basis_vec_1.y*5.0, basis_vec_1.z*5.0);
    std::vector<Point> expanded_points;
    for (const auto& point : projected_canopy_points) {
        expanded_points.push_back(point + expansion_vector);
        expanded_points.push_back(point - expansion_vector);
    }
    expanded_points.insert(expanded_points.end(), projected_canopy_points.begin(), projected_canopy_points.end());
    Polyhedron P_expanded;
    CGAL::convex_hull_3(expanded_points.begin(), expanded_points.end(), P_expanded);
    Side_of_triangle_mesh side_of(P_expanded);

    //Tree_i canopy_tree(faces(poly).first, faces(poly).second, poly);
    //std::cout << "Created convex hull" << std::endl;
    // Flatten all points down to the xy plane and query overlapping areas. Keep track of partially shaded array triangles
    std::vector<Point2> flattened_canopy_points;
    size_t num_points = projected_canopy_points.size();
    Polygon2 convex_hull_2d;
    for (size_t i=0; i<num_points; i++) {
        Point p = projected_canopy_points[i];
        flattened_canopy_points.push_back(Point2(p.x(), p.y()));
        //convex_hull_2d.push_back(Point2(v->point().x(), v->point().y()));
    }

    std::vector<Point2> result;
    CGAL::convex_hull_2(flattened_canopy_points.begin(), flattened_canopy_points.end(), std::back_inserter(result));

    Delaunay2 dt;
    dt.insert(result.begin(), result.end());

    // Keep track of triangle indices that are partially shaded

    size_t num_array_triangles = flattened_array_points.size();
    int count1 = 0;
    int num_tri = 0;
    std::unordered_map<size_t, double> triangle_areas; // idx in flattened_array_points -> area
    for (size_t k=0; k<num_array_triangles; k++) {
        triangle_areas.insert({k, 0.0});
    }
    for (size_t i=0; i<num_array_triangles; i++) {
        Triangle2 tri = flattened_array_points[i];
        bool intersects = false;
        for (Face_iterator fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); ++fit) {
            Point2 p1 = fit->vertex(0)->point();
            Point2 p2 = fit->vertex(1)->point();
            Point2 p3 = fit->vertex(2)->point();

            CGAL::Object result = CGAL::intersection(tri, Triangle2(p1, p2, p3));
            if (const Point* p = CGAL::object_cast<Point>(&result)) {
                //std::cout << "Intersection is a point: " << *p << std::endl;
                count1++;
            } else if (const std::vector<Point>* poly = CGAL::object_cast<std::vector<Point>>(&result)) {
                count1++;
                // std::cout << "Intersection is a polygon with vertices: ";
                // for (const auto& pt : *poly) {
                //     std::cout << pt << " ";
                // }
                // std::cout << std::endl;
            } else if (const Triangle2* tria = CGAL::object_cast<Triangle2>(&result)) {
                triangle_areas[i] += std::abs(CGAL::area(tria->vertex(0), tria->vertex(1), tria->vertex(2)));
                count1++;
                intersects = true;
            } else {
                //std::cout << "No intersection or intersection is of another type." << std::endl;
            }
        }
        num_tri = intersects ? num_tri + 1 : num_tri;
    }

    std::unordered_set<size_t> partially_shaded_triangles;
    int num_completely_shaded_triangles = 0;
    size_t idx = 0;
    std::unordered_set<size_t> completely_shaded_triangles;
    for (auto it=triangle_areas.begin(); it != triangle_areas.end(); it++) {
        double shaded = it->second;
        if (shaded > 0.0) {
            Triangle2 t = flattened_array_points[it->first];
            double are = std::abs(CGAL::area(t[0], t[1], t[2]));
            //std::cout << "FLATTENED TRIANGLE AREA: " << are << std::endl;
            //std::cout << "SHADED AREA: " << shaded << std::endl;
            if (std::abs(shaded-are) < 1.0 || shaded > are) {
                num_completely_shaded_triangles++;
                completely_shaded_triangles.insert(it->first);
            } else {
                partially_shaded_triangles.insert(it->first);
            }
        }
    }
    std::cout << "NUM TRIANGLE INTERSECTIONS: " << num_tri << " | Total completely shaded triangles " << num_completely_shaded_triangles << " | Total partially shaded triangles " << partially_shaded_triangles.size() << std::endl;

    // ------ NEXT STEPS ---------
    // SHOOT A TON OF RAYS FROM PARTIALLY SHADED TRIANGLE TOWARDS CANOPY AND (# rays hit / total rays cast) * triangle area
    // is the partially shaded area. Use Random_points_in_triangle_3 to generate rays

    int NUM_RAYS = 2000;
    Direction normal_direction(sun_plane_normal.x, sun_plane_normal.y, sun_plane_normal.z);
    auto random_gen = CGAL::Random();
    for (auto it = partially_shaded_triangles.begin(); it != partially_shaded_triangles.end(); it++) {
        CGAL::Random_points_in_triangle_3<Point> generator(array_triangles[*it], random_gen);

        std::vector<Point> triangle_points;
        int num_rays_intersected = 0;
        Direction triangle_ray;
        for (int i=0; i<NUM_RAYS; i++) {
            Point origin_point = *generator++;
            Ray triangle_ray(origin_point, normal_direction);

            if (canopy_tree.any_intersected_primitive(triangle_ray)) {
                num_rays_intersected++;
            }
        }

        std::cout << "PARTIALLY SHADED TRIANGLE AREA: " << triangle_areas[*it] << " | SHADED AREA: " << ((num_rays_intersected /(double) NUM_RAYS)) * triangle_areas[*it] << " | NUM RAYS INTERSECTED: " << num_rays_intersected << std::endl;
    }
    //
    // int num_illegal_triangles = 0;
    // for (auto it=partially_shaded_triangles.begin(); it != partially_shaded_triangles.end(); it++) {
    //     size_t jk = *it;

    //     Triangle partial = array_triangles[jk];

    //     Point p1 = partial[0];
    //     Point p2 = partial[1];
    //     Point p3 = partial[2];

    //     bool p1_inside = false; * 
    //     bool p2_inside = false;
    //     bool p3_inside = false;

    //     int num_points_inside = 0;

    //     // Tree_i::Point_and_primitive_id closest = canopy_tree.closest_point_and_primitive(p1);
    //     // Point closest_point = closest.first;

    //     // std::cout << "The closest point on the mesh from point 1 is: " << closest_point << std::endl;
    //     // std::cout << "The distance to the mesh is: " << std::sqrt(CGAL::squared_distance(p1, closest_point)) << std::endl;

    //     // closest = canopy_tree.closest_point_and_primitive(p2);
    //     // closest_point = closest.first;

    //     // std::cout << "The closest point on the mesh from point 2 is: " << closest_point << std::endl;
    //     // std::cout << "The distance to the mesh is: " << std::sqrt(CGAL::squared_distance(p2, closest_point)) << std::endl;

    //     // closest = canopy_tree.closest_point_and_primitive(p3);
    //     // closest_point = closest.first;

    //     // std::cout << "The closest point on the mesh from point 3 is: " << closest_point << std::endl;
    //     // std::cout << "The distance to the mesh is: " << std::sqrt(CGAL::squared_distance(p3, closest_point)) << std::endl;
    //     CGAL::Bounded_side result = side_of(p1);
    //     if (result == CGAL::ON_BOUNDED_SIDE || result == CGAL::ON_BOUNDARY) {
    //         p1_inside = true;
    //         num_points_inside++;
    //     }

    //     result = side_of(p2);
    //     if (result == CGAL::ON_BOUNDED_SIDE || result == CGAL::ON_BOUNDARY) {
    //         p2_inside = true;
    //         num_points_inside++;
    //     }

    //     result = side_of(p3);
    //     if (result == CGAL::ON_BOUNDED_SIDE || result == CGAL::ON_BOUNDARY) {
    //         p3_inside = true;
    //         num_points_inside++;
    //     }

    //     if (num_points_inside == 1) {
            
    //     } else if (num_points_inside == 2) {

    //     } else {
    //         num_illegal_triangles++;
    //         std::cout << "Number of points inside: " << num_points_inside << std::endl;
    //     }

    // }

    // std::cout << "Number of illegal triangles: " << num_illegal_triangles << std::endl;
    // ------ NEXT STEPS ---------
    // Triangulate the 2d canopy convex hull and query intersections between the convex hull and flattened array cells
    // Create a side_of_triangle_mesh with the 3d convex hull of the canopy
    // Create an aabb tree of the 3d convex hull of the canopy (tree)
    // For partially shaded triangles:
    //      query the vertices of the triangle (a,b,c) with side_of_triangle_mesh to see which ones lie inside the mesh
    //      If one vertex (a) lies inside mesh:
    //          Query intersection points between segment (a,b) and tree and segment (a,c) and tree. This creates the smaller
    //          triangle that lies in the shaded area. Calculate it and done
    //      else if two vertices (b,c) lie inside mesh:
    //          Query intersection points between segment (a,b) and tree and segment (a,c) and tree. This creates the smaller
    //          triangle that lies in the shaded area. Calculate it and done
    //       








    //Alpha_shape alpha_shape(flattened_canopy_points.begin(), flattened_canopy_points.end(), 10000.0, Alpha_shape::GENERAL);

    // for (const Triangle2& triangle : flattened_array_points) {
    //     std::vector<CGAL::Object> intersections;
    //     CGAL::intersection(convex_hull_2d, triangle, std::back_inserter(intersections));
    //     double overlapping_area = 0.0;
    //     for (const auto& obj : intersections) {
    //         if (const Polygon2* polygon = CGAL::object_cast<Polygon2>(&obj)) {
    //             overlapping_area += polygon->area();
    //         }
    //     }
    // }

    // Extend convex hull of projected canopy points into prism parallel to the sun plane
    // For all partially shaded triangles:
    //      Query intersection area between triangle and the prism

    // Cast rays from the convex hull points towards the array (i.e. normal to the sun plane)
    // for (const glm::vec3& p : convex_hull_3d) {
    //     Point ray_origin(p.x, p.y, p.z);
    //     Point direction(sun_plane_normal.x*-1.0f, sun_plane_normal.y*-1.0f, sun_plane_normal.z*-1.0f);
    //     Ray canopy_ray(ray_origin, direction);
    //     canopy_rays.push_back(canopy_ray);
    // }
    // if(intersection)
    // {
    //     // If there is an intersection, we can get the intersection point
    //     const K::Point_3* p = boost::get<K::Point_3>(&*intersection);
    //     if(p)
    //     {
    //         std::cout << "First intersection point: " << *p << std::endl;
    //     }
    // }
    //std::cout << "NUMBER OF INTERSECTIONS: " << array_tree.number_of_intersected_primitives(canopy_rays[100]) << std::endl;


    // double area = Clipper2Lib::Area(canopy_path);
    // std::cout << "Area of the homemade polygon: " << area << std::endl;

    // area = Clipper2Lib::Area(path_2);
    // std::cout << "Area of the clipper2 polygon: " << area << std::endl;
    // std::cout << "NUM VERTICES IN CANOPY CONVEX HULL: " << hull.getVertexBuffer().size() << std::endl;
    // std::cout << "NUM VERTICES IN CANOPY CONVEX HULL HOMEMADE: " << homemade_convex_hull.size() << std::endl;
    // std::cout << "SUN PLANE NORMAL: " << sun_plane_normal[0] << " " << sun_plane_normal[1] << " " << sun_plane_normal[2] << std::endl;
    // std::cout << "SUN PLANE POINT: " << sun_plane->point[0] << " " << sun_plane->point[1] << " " << sun_plane->point[2] << std::endl;

    // // Compute convex hull projection of all array cells onto sun plane
    // // Project cell meshes onto the plane
    // std::vector<std::vector<Vertex>> cell_vertices = car->get_array_cell_vertices();
    // std::vector<std::vector<glm::vec2>> projected_array_vertices;
    // projected_array_vertices.reserve(cell_vertices.size());

    // size_t idx = 0;
    // for (const std::vector<Vertex>& cell : cell_vertices) {
    //     std::vector<glm::vec2> projected_cell;
    //     projected_cell.reserve(cell.size());
    //     projected_array_vertices.push_back(projected_cell);
    //     for (const Vertex& point : cell) {
    //         glm::vec3 p = point.position - sun_plane->point;
    //         double x2D = glm::dot(p, basis_vec_1);
    //         double y2D = glm::dot(p, basis_vec_2);
    //         projected_array_vertices[idx].push_back(glm::vec2(x2D, y2D));
    //     }
    //     idx++;
    // }
    // Clipper2Lib::Paths64 cell_paths;
    // std::vector<std::vector<glm::vec2>> cell_convex_hulls;
    // std::vector<Clipper2Lib::Path64> cell_convex_hull_paths;
    // std::vector<double> cell_areas;
    // cell_convex_hulls.reserve(cell_vertices.size());
    // for (size_t i=0; i<cell_vertices.size(); i++) {
    //     cell_convex_hulls.push_back(convexHull2D(projected_array_vertices[i]));

    //     Clipper2Lib::Path64 path;
    //     for (const glm::vec2& vertex : cell_convex_hulls[i]) {
    //         path.push_back(Clipper2Lib::Point64(vertex.x, vertex.y));
    //         cell_convex_hull_paths.push_back(path);
    //     }
    // }
    // Clipper2Lib::Paths64 solution = Clipper2Lib::Intersect({canopy_path}, {cell_convex_hull_paths[0]}, Clipper2Lib::FillRule::NonZero);
    // if (solution.size() > 0) {
    //     double final_area = Clipper2Lib::Area(solution[0]);
    //     std::cout << "Area of the overlap: " << final_area << " solution # paths: " << solution.size() << std::endl;
    // }
    // Get overlapping convex hull areas between array cell and canopy
}
