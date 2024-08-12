#include "irradiance_csv.hpp"
#include "QuickHull.hpp"
#include <clipper2/clipper.h>

IrradianceCSV::IrradianceCSV(std::shared_ptr<SunPositionLUT> sun_position_lut, std::shared_ptr<Model> car_model,
                            double bearing, std::string direction) {
    sun_position = sun_position_lut;
    car = car_model;

    car_model->calc_centroid();
    car_model->center_model();
    car_model->init_camera();  // Get bounding box characteristics

    size_t num_sun_positions = sun_position_lut->get_num_rows();

    for (size_t i=0; i<num_sun_positions; i++) {
        double azimuth = sun_position_lut->get_azimuth_value(i);
        double elevation = sun_position_lut->get_elevation_value(i);
        double irradiance = sun_position_lut->get_irradiance_value(i);

        // Create sun plane
        std::shared_ptr<SunPlane> sun_plane = std::make_shared<SunPlane>(
            azimuth, elevation, direction, bearing, car_model->get_min_values(),
            car_model->get_max_values()
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

void IrradianceCSV::compute_canopy_shadow(std::shared_ptr<SunPlane>& sun_plane) {
    // Project canopy points onto the plane of the sun
    // https://stackoverflow.com/questions/23472048/projecting-3d-points-to-2d-plane
    std::vector<Vertex> canopy_vertices = car->get_canopy_mesh()->get_vertices();
    std::vector<glm::vec2> projected_canopy_vertices;
    projected_canopy_vertices.reserve(canopy_vertices.size());
    glm::vec3& sun_plane_normal = sun_plane->normal;
    
    // Define an orthonormal basis for the sun plane
    glm::vec3 basis_vec_1 = findOrthogonalVector(sun_plane_normal);
    glm::vec3 basis_vec_2 = glm::normalize(glm::cross(sun_plane_normal, basis_vec_1));

    // Now that I have an orthonormal basis for the 3D plane i.e. Two orthogonal 3D vectors,
    // then I project the vector = (point on plane -> point to project) onto the plane in order
    // to get the coordinates of the projected point with respect to the orthonormal basis.
    // This effectively creates a 2D point
    for (const auto& point : canopy_vertices) {
        glm::vec3 p = point.position - sun_plane->point;
        double x2D = glm::dot(p, basis_vec_1);
        double y2D = glm::dot(p, basis_vec_2);
        projected_canopy_vertices.push_back(glm::vec2(x2D, y2D));
    }
    // Compute convex hull projection of canopy onto plane
    std::vector<quickhull::Vector3<float>> qhPoints;
    for (const glm::vec2& p : projected_canopy_vertices) {
        qhPoints.emplace_back(p.x, p.y, 0.0);  // Attach a 3rd dimension in order to get quickhull to work and recognize that 
                                               // the points are coplanar
    }
    quickhull::QuickHull<float> qh;
    auto hull = qh.getConvexHull(qhPoints, true, false);

    std::vector<glm::vec2> homemade_convex_hull = convexHull2D(projected_canopy_vertices);
    Clipper2Lib::Path64 canopy_path;
    for (const glm::vec2& vertex : homemade_convex_hull) {
        canopy_path.push_back(Clipper2Lib::Point64(vertex.x, vertex.y));
    }

    double area = Clipper2Lib::Area(canopy_path);
    std::cout << "Area of the homemade polygon: " << area << std::endl;

    Clipper2Lib::Path64 path_2;
    for (const auto& vertex : hull.getVertexBuffer()) {
        path_2.push_back(Clipper2Lib::Point64(vertex.x, vertex.y));
    }

    area = Clipper2Lib::Area(path_2);
    std::cout << "Area of the clipper2 polygon: " << area << std::endl;
    std::cout << "NUM VERTICES IN CANOPY CONVEX HULL: " << hull.getVertexBuffer().size() << std::endl;
    std::cout << "NUM VERTICES IN CANOPY CONVEX HULL HOMEMADE: " << homemade_convex_hull.size() << std::endl;
    std::cout << "SUN PLANE NORMAL: " << sun_plane_normal[0] << " " << sun_plane_normal[1] << " " << sun_plane_normal[2] << std::endl;
    std::cout << "SUN PLANE POINT: " << sun_plane->point[0] << " " << sun_plane->point[1] << " " << sun_plane->point[2] << std::endl;

    // Compute convex hull projection of all array cells onto sun plane
    // Project cell meshes onto the plane
    std::vector<std::vector<Vertex>> cell_vertices = car->get_array_cell_vertices();
    std::vector<std::vector<glm::vec2>> projected_array_vertices;
    projected_array_vertices.reserve(cell_vertices.size());

    size_t idx = 0;
    for (const std::vector<Vertex>& cell : cell_vertices) {
        std::vector<glm::vec2> projected_cell;
        projected_cell.reserve(cell.size());
        projected_array_vertices.push_back(projected_cell);
        for (const Vertex& point : cell) {
            glm::vec3 p = point.position - sun_plane->point;
            double x2D = glm::dot(p, basis_vec_1);
            double y2D = glm::dot(p, basis_vec_2);
            projected_array_vertices[idx].push_back(glm::vec2(x2D, y2D));
        }
        idx++;
    }
    Clipper2Lib::Paths64 cell_paths;
    std::vector<std::vector<glm::vec2>> cell_convex_hulls;
    std::vector<Clipper2Lib::Path64> cell_convex_hull_paths;
    std::vector<double> cell_areas;
    cell_convex_hulls.reserve(cell_vertices.size());
    for (size_t i=0; i<cell_vertices.size(); i++) {
        cell_convex_hulls.push_back(convexHull2D(projected_array_vertices[i]));

        Clipper2Lib::Path64 path;
        for (const glm::vec2& vertex : cell_convex_hulls[i]) {
            path.push_back(Clipper2Lib::Point64(vertex.x, vertex.y));
            cell_convex_hull_paths.push_back(path);
        }
    }
    Clipper2Lib::Paths64 solution = Clipper2Lib::Intersect({canopy_path}, {cell_convex_hull_paths[0]}, Clipper2Lib::FillRule::NonZero);
    if (solution.size() > 0) {
        double final_area = Clipper2Lib::Area(solution[0]);
        std::cout << "Area of the overlap: " << final_area << " solution # paths: " << solution.size() << std::endl;
    }
    // Get overlapping convex hull areas between array cell and canopy
}
