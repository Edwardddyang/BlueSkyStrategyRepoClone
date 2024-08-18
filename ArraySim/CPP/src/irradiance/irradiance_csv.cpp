#include "irradiance_csv.hpp"
#include "QuickHull.hpp"
#include <clipper2/clipper.h>
#include <fstream>
#include <iomanip>

IrradianceCSV::IrradianceCSV(std::shared_ptr<SunPositionLUT> sun_position_lut, std::shared_ptr<Model> car_model,
                            double bearing, std::string direction, bool precise_shadows) {
    sun_position = sun_position_lut;
    car = car_model;
    car->calc_centroid();
    car->center_model();
    car->init_camera();  // Get bounding box characteristics

    size_t num_sun_positions = sun_position_lut->get_num_rows();
    size_t num_solar_cells = (car->get_array_cell_meshes()).size();
    irradiance_csv.resize(num_sun_positions);
    for (size_t i=0; i<num_sun_positions; i++) {
        // Extract information about the sun positions csv
        irradiance_csv[i].resize(num_solar_cells);
        double azimuth = sun_position_lut->get_azimuth_value(i);
        double elevation = sun_position_lut->get_elevation_value(i);
        double irradiance = sun_position_lut->get_irradiance_value(i);

        // Create sun plane
        std::shared_ptr<SunPlane> sun_plane = std::make_shared<SunPlane>(
            azimuth, elevation, direction, bearing, car->get_min_values(),
            car->get_max_values()
        );

        std::cout << "INDEX: " << i << std::endl;
        // Calculate the effective irradiance for all cells given the position of the
        // sun in the sky
        construct_csv_row(sun_plane, i, irradiance, precise_shadows);
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

glm::vec3 IrradianceCSV::compute_triangle_norm(const Triangle& triangle) {
    Point p1 = triangle[0];
    Point p2 = triangle[1];
    Point p3 = triangle[2];
    glm::vec3 u(p2.x() - p1.x(), p2.y() - p1.y(), p3.z() - p3.z());
    glm::vec3 v(p3.x() - p1.x(), p3.y() - p1.y(), p3.z() - p1.z());

    // In the case of a degenerate triangle, we assume its so small that
    // its effective irradiance ~= 0. We return a 0 normal vector to realize
    // this
    if (u == v) {
        return glm::vec3(0,0,0);
    }
    // Ensure that the normal vector has +ve z component
    glm::vec3 cross_product = glm::cross(u,v);
    if (cross_product.z < 0.0f) {
        cross_product.z *= -1.0f;
    }

    return (glm::normalize(cross_product));
}

void IrradianceCSV::construct_csv_row(std::shared_ptr<SunPlane>& sun_plane, size_t row_idx, double irradiance, bool precise_shadows) {
    std::vector<Vertex> canopy_vertices = car->get_canopy_mesh()->get_vertices();
    std::vector<std::vector<Vertex>> array_cells = car->get_array_cell_vertices();
    glm::vec3& sun_plane_normal = sun_plane->normal;

    // Construct AABB tree of the array cells.
    // Note: triangle index refers to the index of a triangle in array_triangles or
    // flattened_array_triangles or triangle_normals. Cell index refers to the index
    // of a cell in array_cells
    std::vector<Triangle> array_triangles;
    std::vector<Triangle2> flattened_array_triangles; // Flattened onto the XY plane
    std::unordered_map<size_t, size_t> triangle_to_cell;  // Map of triangle indices to array cell indices
    std::unordered_map<size_t, std::vector<size_t>> cell_to_triangle;  // Map of cell indices to triangle indices
    std::vector<double> triangle_areas;  // Triangle index -> its area
    std::vector<glm::vec3> triangle_normals; // Normalized unit normal vector of each triangle
    size_t num_array_cells = array_cells.size();
    size_t triangle_idx = 0;
    for (size_t cell_idx=0; cell_idx<num_array_cells; cell_idx++) {
        cell_to_triangle[cell_idx] = {};
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

            Triangle array_triangle = Triangle(point1, point2, point3);
            array_triangles.push_back(array_triangle);
            triangle_areas.push_back(std::sqrt(array_triangle.squared_area()));
            glm::vec3 normal = compute_triangle_norm(array_triangle);
            triangle_normals.push_back(normal);
            triangle_to_cell.insert({triangle_idx, cell_idx});
            cell_to_triangle[cell_idx].push_back(triangle_idx);
            triangle_idx++;
        }
    }
    Tree array_tree(array_triangles.begin(), array_triangles.end());
    size_t num_array_triangles = flattened_array_triangles.size();
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
    std::unordered_set<size_t> non_shaded_triangles;
    std::vector<double> flattened_triangle_shaded_area(num_array_triangles, 0.0);  // Triangle index -> its flattened shaded area (NOT its actual shaded area in 3D space)
    std::vector<double> triangle_shaded_area(num_array_triangles); // Each triangle's shaded area in 3D space
    int num_tri = 0;
    for (size_t tri_idx=0; tri_idx<num_array_triangles; tri_idx++) {
        Triangle2 tri = flattened_array_triangles[tri_idx];
        bool intersects = false;
        for (Face_iterator fit = convex_canopy_dt.finite_faces_begin(); fit != convex_canopy_dt.finite_faces_end(); ++fit) {
            Point2 p1 = fit->vertex(0)->point();
            Point2 p2 = fit->vertex(1)->point();
            Point2 p3 = fit->vertex(2)->point();

            CGAL::Object result = CGAL::intersection(tri, Triangle2(p1, p2, p3));
            if (const Triangle2* int_tri = CGAL::object_cast<Triangle2>(&result)) {
                flattened_triangle_shaded_area[tri_idx] += std::abs(CGAL::area(int_tri->vertex(0),
                                                                     int_tri->vertex(1),
                                                                     int_tri->vertex(2)));
                intersects = true;
            }
        }
        num_tri = intersects ? num_tri + 1 : num_tri;
    }
    int num_completely_shaded_triangles = 0;
    for (size_t tri_idx = 0; tri_idx < num_array_triangles; tri_idx++) {
        // Shaded area on the xy plane. Again, this is NOT representative of actual shaded area
        double flattened_shaded_area = flattened_triangle_shaded_area[tri_idx];
        if (flattened_shaded_area > 0.0) {
            Triangle2 tri = flattened_array_triangles[tri_idx];
            double flattened_tri_area = std::abs(CGAL::area(tri[0], tri[1], tri[2]));
            if (std::abs(flattened_shaded_area - flattened_tri_area) < 1.0 || flattened_shaded_area > flattened_tri_area) {
                num_completely_shaded_triangles++;
                completely_shaded_triangles.insert(tri_idx);
                triangle_shaded_area[tri_idx] = flattened_tri_area;
            } else {
                partially_shaded_triangles.insert(tri_idx);
                triangle_shaded_area[tri_idx] = 0.0;  // If we're not calculating precise shadows, approximate shaded triangles
                                                        // as fully shaded
            }
        } else {
            non_shaded_triangles.insert(tri_idx);
            triangle_shaded_area[tri_idx] = 0.0;
        }
    }

    // For each partially shaded triangle, cast NUM_RAYS from the triangle towards the sun plane. The numerical approximation
    // for shaded area is given by (Num rays intersecting with canopy / NUM_RAYS) * triangle area
    if (precise_shadows) {
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

            triangle_shaded_area[*it] = ((num_rays_intersected /(double) NUM_RAYS)) * triangle_areas[*it];
        }
    }

    // Compute effective irradiances
    for (size_t cell_idx = 0; cell_idx < num_array_cells; cell_idx++) {
        irradiance_csv[row_idx][cell_idx] = 0.0;
    }
    std::vector<double> triangle_powers;
    for (size_t tri_idx=0; tri_idx < num_array_triangles; tri_idx++) {
        double shaded_area = triangle_shaded_area[tri_idx];
        size_t cell_i = triangle_to_cell[tri_idx];
        glm::vec3 normal = triangle_normals[tri_idx];

        double dot_product = glm::dot(normal, sun_plane_normal);
        if (dot_product > 0.0) {
            triangle_powers.push_back(dot_product * (triangle_areas[tri_idx] - shaded_area) * irradiance);
        }
    }
    for (size_t cell_idx=0; cell_idx < num_array_cells; cell_idx++) {
        std::vector<size_t> cell_triangles = cell_to_triangle[cell_idx];

        double cell_power = 0.0;
        double cell_area = 0.0;
        for (const size_t& tri_idx : cell_triangles) {
            cell_power += triangle_powers[tri_idx];
            cell_area += triangle_areas[tri_idx];
        }

        irradiance_csv[row_idx][cell_idx] = (cell_power / cell_area);
    }
}

void IrradianceCSV::write_csv(const std::string& csv_name) {
    std::ofstream csv_file(csv_name);
    csv_file << std::fixed << std::setprecision(8);

    if (!csv_file.is_open()) {
        std::cout << "Could not open csv file for writing" << std::endl;
        return;
    }

    for (const std::vector<double> row : irradiance_csv) {
        size_t num_cells = row.size();
        for (size_t i=0; i<num_cells; i++) {
            csv_file << row[i];
            csv_file << ",";
        }

        csv_file << "\n";
    }

    csv_file.close();
}

IrradianceCSV::IrradianceCSV(std::filesystem::path csv_path) {
    std::ifstream lut(csv_path.string()); // Use ifstream for reading
    assert(lut.is_open() && "File not found...");

    std::string line;
    std::string cell;
    std::vector<double> row;

    while (std::getline(lut, line)) {
        std::stringstream ss(line);
        std::vector<double> row;
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            try {
                row.push_back(std::stod(cell));
            } catch (const std::invalid_argument& e) {
                std::cerr << "Invalid number in file: " << cell << std::endl;
            }
        }
        irradiance_csv.push_back(row);
    }
}
