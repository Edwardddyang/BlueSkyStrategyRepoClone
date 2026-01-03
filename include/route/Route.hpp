/**
 * Classes used to represent a race route and proposed race plans
 */

#pragma once

#include <stdlib.h>

#include <string>
#include <filesystem>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <limits>
#include <utility>
#include <iostream>

#include "config/ConfigParser.hpp"
#include "SimUtils/Luts.hpp"
#include "SimUtils/Types.hpp"
#include "nlohmann/json.hpp"
#include <filesystem>

using json = nlohmann::json;
using PointsLut = Luts::BaseLut<double>;
using CoordVec = std::vector<util::type::Coord>;

////////////////////// General Utility Functions ///////////////////////
/** @brief Calculate the distance from coords[starting_idx] to coords[ending_idx] in m */
double calculate_segment_distance(const CoordVec& coords,
                                  size_t starting_idx,
                                  size_t ending_idx);

/** @brief Append the character `ch` to the stringstream n times */
void append_char_n_times(char ch, uint64_t n, std::stringstream& output);

/** @brief Truncate `number` to fit a string of n characters.
* If len(number) < n, it does nothing
*/
std::string truncate_number(double number, int n);

///////////////////////////////////////////////////////////////////////
//////////////////////// Race Route Definition ////////////////////////
///////////////////////////////////////////////////////////////////////

struct WSCRouteParameters {
  const double max_route_speed;                // Maximum speed of the route in m/s
  const std::unordered_set<int> control_stops; // Indices of route_points where control stops
                                               // are located

  WSCRouteParameters(double max_route_speed,
                     std::unordered_set<int> control_stops) :
                     max_route_speed(max_route_speed),
                     control_stops(std::move(control_stops)) {}
};

inline WSCRouteParameters get_wsc_route_params(ConfigParser* parser) {
  RUNTIME_EXCEPTION(parser != nullptr, "Parser is null when loading WSC Route parameters");
  return WSCRouteParameters{
    util::constants::kph2mps(parser->get_max_route_speed()),
    parser->get_control_stops()     
  };
}

/** Representation of the WSC Route */
class WSCRoute {
 private:
  /* Points of the route */
  CoordVec route_points;

  const WSCRouteParameters params;

  /* Number of points along the route */
  size_t num_points;

  /* Total length of the route in m */
  double route_length;

 public:
  /** @brief Load coordinates from a CSV file with layout |latitude|longitude|altitude */
  WSCRoute(WSCRouteParameters params, std::filesystem::path route_path);
  WSCRoute() = default;

  /* Getters */
  inline std::unordered_set<int> get_control_stops() const {return params.control_stops;}
  inline const CoordVec& get_route_points() const {return route_points;}
  inline size_t get_num_points() const {return route_points.size();}
  inline double get_route_length() const {return route_length;}
  inline double get_max_route_speed() const {return params.max_route_speed;}
  inline bool is_empty() const {return route_points.size() == 0;}

  /** @brief Segment the route into uniform lengths
  * @param length The length of each segment in m. If the total length of the base route
  * cannot be divided into equal lengths, the last segment will be shorter than the rest
  */
  std::vector<std::pair<size_t, size_t>> segment_route_uniform(double length) const;
};

struct FSGPRouteParameters {
  const double max_route_speed;                // Maximum speed of the route in m/s

  FSGPRouteParameters(double max_route_speed) :
                     max_route_speed(max_route_speed) {}
};

inline FSGPRouteParameters get_fsgp_route_params(ConfigParser* parser) {
  RUNTIME_EXCEPTION(parser != nullptr, "Parser is null when loading FSGP Route parameters");
  return FSGPRouteParameters{
    util::constants::kph2mps(parser->get_max_route_speed())   
  };
}

/** Representation of the FSGP Route */
class FSGPRoute {
 private:
  /* Points of the track */
  CoordVec route_points;
  size_t num_track_points;

  /* Total length of the track in m */
  double route_length;

  /* Lookup table for all distances between any two points. For FSGP, this is feasible
  * since there aren't many coordinates on the track
  */
  PointsLut route_distances;

  /* Longest straight path */
  double longest_straight;

  const FSGPRouteParameters params;

  // Cornering locations. Each pair represents the start and end indices
  // of the corner in the route_points
  std::vector<std::pair<size_t, size_t>> corner_segment_bounds;
  std::unordered_set<size_t> corner_start_indices;
  std::unordered_set<size_t> corner_end_indices;

  /* Map of corner end indices to index in corner_segment_bounds */
  std::unordered_map<size_t, size_t> corner_end_to_corner_idx;

  /* Map of corner start indices to index in corner_segment_bounds */
  std::unordered_map<size_t, size_t> corner_start_to_corner_idx;

  /* Maximum speed of corner segments in m/s */
  std::vector<double> corner_speed_bounds;

 public:
  /** @brief Load track coordinates from a CSV file with layout |latitude|longitude|altitude
  * @param route_path: Absolute path to the base route csv
  * @param precomputed_distances_path: Absolute path to the csv with precomputed distances.
  * If this path doesn't exist, then calculate all distances, and create the file
  * @param corner_bounds_path: Absolute path to the cornering bounds
  */
  FSGPRoute(FSGPRouteParameters params,
            std::filesystem::path route_path,
            std::filesystem::path precomputed_distances_path,
            std::filesystem::path corner_bounds_path = std::filesystem::path());

  /** @brief Read a csv of corner index bounds with layout |start index|end index|max speed (m/s)|
  * @param corner_bounds_path: Path to the csv file
  * @param max_car_speed: Maximum car speed to limit cornering speeds. Defaults to infinity
  */
  void init_cornering_bounds(std::filesystem::path corner_bounds_path,
                             double max_car_speed = std::numeric_limits<double>::infinity());

  /** @brief Calculate distances between all pairs of points on the track route
   * @param csv_path Path to dump the output distances
  */
  void calculate_distances(std::filesystem::path csv_path);

  /** @brief Find the longest straight in the route */
  void init_longest_straight();

  /** @brief Return the corner index that a route index is closest to
   *
   * Examples: Consider cornering_segment_bounds = [{0,5}, {16, 20}, {31,47}]
   * If route index is 2, return 0
   * If route index is 6, return 0
   * If route index is 13, return 1 (closer to {16,20 interval})
   * If route index is 16, return 1
   * If route index is 200, return 2
  */
  size_t get_closest_corner_idx(size_t route_index) const;

  /** @brief Get all corners that a route segment passes through */
  std::vector<size_t> get_overlapping_corners(const std::pair<size_t, size_t>& segment) const;

  /* Getters */
  inline const std::unordered_map<size_t, size_t>& get_corner_end_map() const {
    return corner_end_to_corner_idx;
  }
  inline const std::unordered_map<size_t, size_t>& get_corner_start_map() const {
    return corner_start_to_corner_idx;
  }
  inline const std::vector<std::pair<size_t, size_t>>& get_cornering_segment_bounds() const {
    return corner_segment_bounds;
  }
  inline const std::vector<double>& get_cornering_speed_bounds() const {
    return corner_speed_bounds;
  }
  inline const std::unordered_set<size_t>& get_corner_start_indices() const {
    return corner_start_indices;
  }
  inline const std::unordered_set<size_t>& get_corner_end_indices() const {
    return corner_end_indices;
  }
  inline const CoordVec& get_route_points() const {
    return route_points;
  }
  inline const PointsLut& get_precomputed_distances() const {
    return route_distances;
  }
  inline size_t get_num_points() const {
    return route_points.size();
  }
};

class ASCRoute {
private:
  // Points on the base route
  CoordVec base_route_points;

  // ASC is split into a series of stages. The following
  // maps the i-th stage to its start and end indices in base_route_points
  std::vector<std::pair<size_t, size_t>> stages;

  // {Open time, Close time} time for each stage
  // TODO: Figure out how to fill this
  std::vector<std::pair<util::type::Time, util::type::Time>> stage_time_intervals;

  // Checkpoints as indices into base_route_points
  std::vector<size_t> checkpoints;

  // {Open time, Close time} time for each checkpoint
  // TODO: Figure out how to fill these
  std::vector<std::pair<util::type::Time, util::type::Time>> checkpoint_time_intervals;

  // Loops
  std::vector<CoordVec> loops;

  // Map a loop's first coordinate to the loop coordinates
  std::unordered_map<util::type::Coord, CoordVec*, util::type::CoordHash> coord_to_loop;

  // Map an index in the loops array to a coordinate in base_route_points where the loop
  // begins i.e. the i-th loop begins at base_route_points[loops[i]]
  std::vector<size_t> loop_begin_indices;

  // find the closest base route coordinate to the loop start point
  util::type::Coord find_base_route_start(util::type::Coord loop_start_coord);
public:
  ASCRoute(std::filesystem::path route_path,
           std::filesystem::path loop_config_dir = std::filesystem::path());
  ASCRoute() = default;

  /** @brief Read all csv files from `loop_config_dir` and add them as loops */
  void init_loops(std::filesystem::path loop_config_dir);
  void add_loop(std::filesystem::path loop_file_path);

  /** @brief Check if a coordinate is the beginning of a loop */
  bool is_loop_start(util::type::Coord route_coord) const;
  CoordVec* get_loop_points(util::type::Coord route_coord) const;
};

/* Representation of a car route recorded using telemetry data */
class TelemRoute {
 private:
  // Points of the route
  CoordVec route_points;

  // Timestamps
  std::vector<util::type::Time> timestamps;

  // Speeds
  std::vector<double> speeds;

  // Total length of the route in m
  double route_length;

  public:
   /** @brief Load coordinates from a CSV file with layout |latitude|longitude|altitude|timestamp|speed */
   TelemRoute(std::filesystem::path route_path);
   TelemRoute() = default;

   /* Getters */
   inline const CoordVec& get_route_points() const {return route_points;}
   inline const std::vector<util::type::Time>& get_timestamps() const {return timestamps;}
   inline const std::vector<double>& get_speeds() const {return speeds;}
   inline double get_route_length() const {return route_length;}
   inline size_t get_num_points() const {return route_points.size();}
};

template <typename T>
concept RouteType = std::is_same_v<ASCRoute, T> ||
                    std::is_same_v<WSCRoute, T> ||
                    std::is_same_v<FSGPRoute, T>;

