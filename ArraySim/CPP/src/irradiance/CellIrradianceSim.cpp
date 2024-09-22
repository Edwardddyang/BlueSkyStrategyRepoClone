#include "CellIrradianceSim.hpp"
#include <fstream>
#include <iomanip>
#include <cmath>
#include "Globals.h"

CellIrradianceSim::CellIrradianceSim(std::shared_ptr<Model> model, bool precise_shadows)
                                    : car_model(model), partial_shadows(precise_shadows) {
    RUNTIME_ASSERT(model != nullptr &&
                   model->loaded_array &&
                   model->loaded_canopy,
                   "Car model must be loaded when initializing CellIrradianceSim");
    if (!model->initialized_geometries) {
        car_model->init_geometries();
    }
    canopy_vertices = car_model->get_canopy_mesh()->get_vertices();
    array_cells = car_model->get_array_cell_vertices();
}

void CellIrradianceSim::run_static_sim(const std::unique_ptr<SunPositionLUT>& sun_position_lut,
                                        double bearing, std::string direction) {
    size_t num_sun_positions = sun_position_lut->get_num_rows();

    // Hold the data for the irradiance csv
    std::vector<std::vector<double>> irr_csv(num_sun_positions);
    for (size_t i=0; i<num_sun_positions; i++) {
        // Create sun plane
        double azimuth = sun_position_lut->get_azimuth_value(i);
        double elevation = sun_position_lut->get_elevation_value(i);
        double irradiance = sun_position_lut->get_irradiance_value(i);
        glm::vec3 min_values = car_model->get_min_values();
        glm::vec3 max_values = car_model->get_max_values();
        std::unique_ptr<SunPlane> sun_plane = std::make_unique<SunPlane>(
            azimuth, elevation, direction, bearing, min_values, max_values, irradiance
        );

        std::cout << "INDEX: " << i << std::endl;

        // Calculate the effective irradiance for all cells given the position of the
        // sun in the sky
        irr_csv[i] = construct_csv_row(sun_plane);
    }

    // Create the csv
    cell_irradiance_csv = std::make_unique<CellIrradianceCsv>(irr_csv);
}

void CellIrradianceSim::run_dynamic_sim(const std::unique_ptr<SunPositionLUT>& sun_position_lut, 
                                        const std::unique_ptr<RouteLUT>& route_lut, double speed,
                                        std::string direction, Time start_time, Time end_time) {
    // As the car travels through the route, we don't want to linear search for the right sun position row each time
    // Therefore, we save a cache for the sun position row and check if it's time to move to the next row every time
    // the car moves. The cache is initialized here as the sun position closest to the start time
    size_t upper_bound_cache;
    size_t lower_bound_cache;
    size_t sun_position_cache = 0;
    size_t num_sun_positions = sun_position_lut->get_num_rows();
    // Find the two rows in the sun position lut that "sandwhich" the start time
    for (size_t idx=0; idx<num_sun_positions; idx++) {
        if (start_time < sun_position_lut->get_time_value(idx)) {
            upper_bound_cache = idx;
            lower_bound_cache = idx > 0 ? idx - 1 : idx;
            break;
        }
    }

    // Determine which of the upper or lower row is closest to the start time
    time_t lower_bound = sun_position_lut->get_time_value(lower_bound_cache).get_local_time_point();
    time_t upper_bound = sun_position_lut->get_time_value(upper_bound_cache).get_local_time_point();
    time_t start_time_point = start_time.get_local_time_point();

    if (upper_bound == lower_bound) {
        sun_position_cache = lower_bound_cache;
    } else {
        uint64_t diff_time_from_lower = abs(static_cast<double>(lower_bound - start_time_point));
        uint64_t diff_time_from_upper = abs(static_cast<double>(upper_bound - start_time_point));
        if (diff_time_from_lower <= diff_time_from_upper) {
            sun_position_cache = lower_bound_cache;
        } else {
            sun_position_cache = upper_bound_cache;
        }
    }

    // Store dynamic simulation results
    std::vector<std::vector<double>> irr_csv;
    std::vector<double> bearings;
    std::vector<Coord> coordinates;
    std::vector<Time> times;
    std::vector<size_t> sun_position_caches;
    std::vector<std::string> time_strings;

    // Repeatedly travel the route until the end time is reached
    size_t counter = 0;
    size_t route_idx = 0;
    size_t next_route_idx = 1;
    size_t num_points = route_lut->get_num_points();
    Time curr_time = start_time;
    while (curr_time < end_time) {
        if (next_route_idx >= num_points) next_route_idx = 0;
        if (route_idx >= num_points) route_idx = 0;
        Coord start_point = route_lut->get_coord_value(route_idx);
        Coord dest_point = route_lut->get_coord_value(next_route_idx);

        double bearing = get_bearing(start_point, dest_point);

        // Create sun plane
        double azimuth = sun_position_lut->get_azimuth_value(sun_position_cache);
        double elevation = sun_position_lut->get_elevation_value(sun_position_cache);
        double irradiance = sun_position_lut->get_irradiance_value(sun_position_cache);
        glm::vec3 max_values = car_model->get_max_values();
        glm::vec3 min_values = car_model->get_min_values();
        std::unique_ptr<SunPlane> sun_plane = std::make_unique<SunPlane>(
            azimuth, elevation, direction, bearing, min_values, max_values, irradiance
        );

        std::cout << "INDEX: " << counter << std::endl;

        // Calculate the effective irradiance for all cells given the position of the
        // sun in the sky
        auto a = construct_csv_row(sun_plane);
        std::cout << a.size() << std::endl;
        irr_csv.push_back(a);

        // Move to the next point and update the time
        double distance = get_distance(start_point, dest_point);
        double travel_time = distance / speed;  // seconds
        curr_time.update_time_seconds(travel_time);

        // Update sun position index cache
        if (sun_position_cache < num_sun_positions-1) {
            time_t lower_bound = sun_position_lut->get_time_value(sun_position_cache).get_local_time_point();
            time_t upper_bound = sun_position_lut->get_time_value(sun_position_cache+1).get_local_time_point();
            time_t curr_time_point = curr_time.get_local_time_point();

            uint64_t diff_time_from_lower = abs(static_cast<double>(lower_bound - curr_time_point));
            uint64_t diff_time_from_upper = abs(static_cast<double>(upper_bound - curr_time_point));
            if (diff_time_from_upper < diff_time_from_lower) {
                sun_position_cache += 1;
            }
        }

        // Log all the values for the metadata csv and move to the next point in the route
        bearings.push_back(bearing);
        coordinates.push_back(Coord(start_point));
        times.push_back(curr_time);
        sun_position_caches.push_back(sun_position_cache);
        time_strings.push_back(curr_time.get_local_readable_time());
        counter++;
        route_idx++;
        next_route_idx++;
    }
    cell_irradiance_csv = std::make_shared<CellIrradianceCsv>(irr_csv,
                                                            bearings,
                                                            coordinates,
                                                            times,
                                                            sun_position_caches,
                                                            time_strings);
}

void CellIrradianceSim::write_static_csv(const std::filesystem::path& csv_path) const {
    RUNTIME_ASSERT(cell_irradiance_csv != nullptr, "Static cell irradiance CSV not created. Did you call run_static_sim()?");
    cell_irradiance_csv->write_irr_csv(csv_path);
}

void CellIrradianceSim::write_dynamic_csv(const std::filesystem::path& irr_csv_path,
                                          const std::filesystem::path& metadata_csv_path) const {
    RUNTIME_ASSERT(cell_irradiance_csv != nullptr, "Dynamic cell irradiance CSV not created. Did you call run_dynamic_sim()?");
    cell_irradiance_csv->write_irr_csv(irr_csv_path);
    cell_irradiance_csv->write_metadata_csv(metadata_csv_path);
}

double CellIrradianceSim::get_bearing(Coord src_coord, Coord dst_coord) {
    Coord src_coord_rad = {deg2rad(src_coord.lat), deg2rad(src_coord.lon), src_coord.alt};
    Coord dst_coord_rad = {deg2rad(dst_coord.lat), deg2rad(dst_coord.lon), dst_coord.alt};
    double delta_lon = dst_coord_rad.lon - src_coord_rad.lon;

    double X = cos(dst_coord_rad.lat)*sin(delta_lon);
    double Y = (cos(src_coord_rad.lat)*sin(dst_coord_rad.lat))-(sin(src_coord_rad.lat) * cos(dst_coord_rad.lat)* cos(delta_lon));

    if (delta_lon < 0) {
        return 360 + rad2deg(atan2(X,Y));
    } else {
        return rad2deg(atan2(X,Y));
    }
}

double CellIrradianceSim::get_distance(Coord src_coord, Coord dst_coord) {
	constexpr double R = 6371e3; 

	double phi_1 = src_coord.lat * PI/180; 
	double phi_2 = dst_coord.lat * PI/180;
	double delPhi = (dst_coord.lat-src_coord.lat) * PI/180;
	double delLambda = (dst_coord.lon-src_coord.lon) * PI/180;

	double a =  ( sin(delPhi/2) * sin(delPhi/2) ) + ( cos(phi_1) * cos(phi_2) * sin(delLambda/2) * sin(delLambda/2) );
	double c = 2 * atan2(sqrt(a), sqrt(1-a));

	// Haversine distance in m
	double dist_km = (R * c);

	// calculate altitude difference
	double alt_1 = src_coord.alt;
	double alt_2 = dst_coord.alt;

	double alt_difference = abs(alt_1-alt_2);

	// Calculate true distance with haversine
	double true_distance = sqrt(dist_km * dist_km + alt_difference * alt_difference);
	
    // In meters
	return true_distance;
}

glm::vec3 CellIrradianceSim::compute_triangle_norm(const Triangle& triangle) {
    Point p1 = triangle[0];
    Point p2 = triangle[1];
    Point p3 = triangle[2];
    glm::vec3 u(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());
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
        cross_product.x *= -1.0f;
        cross_product.y *= -1.0f;
    }

    return (glm::normalize(cross_product));
}

std::vector<double> CellIrradianceSim::construct_csv_row(const std::unique_ptr<SunPlane>& sun_plane) {
    glm::vec3& sun_plane_normal = sun_plane->normal;
    Direction sun_plane_direction(sun_plane_normal.x, sun_plane_normal.y, sun_plane_normal.z);

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
    const Tree canopy_tree(canopy_triangles.begin(), canopy_triangles.end());

    // Collect all array triangle vertices and query their intersection with the canopy AABB tree
    // Note: triangle index refers to the index of a triangle in array_triangles or
    // flattened_array_triangles or triangle_normals. Cell index refers to the index
    // of a cell in array_cells
    std::vector<Triangle> array_triangles;
    std::unordered_map<size_t, size_t> triangle_to_cell;  // Map of triangle indices to array cell indices
    std::unordered_map<size_t, std::vector<size_t>> cell_to_triangle;  // Map of cell indices to triangle indices
    std::vector<double> triangle_areas;  // Triangle index -> its area
    std::vector<glm::vec3> triangle_normals; // Normalized unit normal vector of each triangle
    std::vector<double> triangle_shaded_areas;  // Triangle index -> its area
    std::unordered_set<size_t> shaded_triangles; // Includes fully shaded and partially shaded triangles
    const size_t num_array_cells = array_cells.size();
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

            Triangle array_triangle = Triangle(point1, point2, point3);
            array_triangles.push_back(array_triangle);
            triangle_areas.push_back(std::sqrt(array_triangle.squared_area()));
            triangle_normals.push_back(compute_triangle_norm(array_triangle));
            triangle_to_cell.insert({triangle_idx, cell_idx});
            cell_to_triangle[cell_idx].push_back(triangle_idx);

            // Query point intersections with the canopy AABB tree
            triangle_shaded_areas.push_back(0.0);

            Ray point1_ray = Ray(point1, sun_plane_direction);
            bool is_point1_shaded = false;
            bool is_point2_shaded = false;
            bool is_point3_shaded = false;
            bool is_fully_shaded = false;
            bool is_partially_shaded = false;
            boost::optional<Primitive_id> intersection = canopy_tree.first_intersected_primitive(point1_ray);
            if (intersection) is_point1_shaded = true;
            
            Ray point2_ray = Ray(point2, sun_plane_direction);
            intersection = canopy_tree.first_intersected_primitive(point2_ray);
            if (intersection) is_point2_shaded = true;

            Ray point3_ray = Ray(point3, sun_plane_direction);
            intersection = canopy_tree.first_intersected_primitive(point3_ray);
            if (intersection) is_point3_shaded = true;

            if (is_point1_shaded && is_point2_shaded && is_point3_shaded) {
                is_fully_shaded = true;
                triangle_shaded_areas[triangle_idx] = triangle_areas[triangle_idx];
                shaded_triangles.insert(triangle_idx);
            } else if (is_point1_shaded || is_point2_shaded || is_point3_shaded) {
                is_partially_shaded = true;
                triangle_shaded_areas[triangle_idx] = triangle_areas[triangle_idx];
                shaded_triangles.insert(triangle_idx);
            }
            // For each partially shaded triangle, cast NUM_RAYS from the triangle towards the sun plane.
            // The numerical approximation for shaded area is given by
            // (Num rays intersecting with canopy / NUM_RAYS) * triangle area
            if (partial_shadows && is_partially_shaded) {
                auto random_gen = CGAL::Random(0);
                CGAL::Random_points_in_triangle_3<Point> generator(array_triangle, random_gen);

                int num_rays_intersected = 0;
                for (int i=0; i<NUM_RAYS; i++) {
                    Point origin_point = *generator++;
                    Ray triangle_ray(origin_point, sun_plane_direction);

                    if (canopy_tree.any_intersected_primitive(triangle_ray)) {
                        num_rays_intersected++;
                    }
                }

                triangle_shaded_areas[triangle_idx] = ((num_rays_intersected /(double) NUM_RAYS)) * triangle_areas[triangle_idx];
            }

            triangle_idx++;
        }
    }
    const size_t num_array_triangles = array_triangles.size();
    double irradiance = sun_plane->irradiance;
    std::vector<double> triangle_powers;
    for (size_t tri_idx=0; tri_idx < num_array_triangles; tri_idx++) {
        double shaded_area = triangle_shaded_areas[tri_idx];
        size_t cell_i = triangle_to_cell[tri_idx];
        glm::vec3 normal = triangle_normals[tri_idx];

        double dot_product = glm::dot(normal, sun_plane_normal);
        if (dot_product > 0.0) {
            triangle_powers.push_back(dot_product * (triangle_areas[tri_idx] - shaded_area) * irradiance);
        } else {
            triangle_powers.push_back(0.0);
        }
    }

    std::vector<double> irradiance_row(num_array_cells);
    for (size_t cell_idx=0; cell_idx < num_array_cells; cell_idx++) {
        std::vector<size_t> cell_triangles = cell_to_triangle[cell_idx];

        double cell_power = 0.0;
        double cell_area = 0.0;
        for (const size_t& tri_idx : cell_triangles) {
            cell_power += triangle_powers[tri_idx];
            cell_area += triangle_areas[tri_idx];
        }
        double effective_irradiance = cell_power / cell_area;
        irradiance_row[cell_idx] = effective_irradiance;
    }

    return irradiance_row;
}
