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

#include "utils/Units.hpp"
#include "utils/Luts.hpp"
#include "utils/CustomTime.hpp"
#include <nlohmann/json.hpp>

double calculate_segment_distance(const std::vector<Coord>& coords,
                                  const size_t starting_idx,
                                  const size_t ending_idx);

/** A class to represent a proposed race plan
 * 
 * Each plan splits the route into segments described by a series of {start index, end index} pairs.
 * Each segment is assigned either a constant speed that the car maintains or it is designated as an
 * acceleration segment. This means that the the car will accelerate from the initial speed to the
 * final speed.
 */
class RacePlan {
 public:
  // Describes a segment between two points, A and B, as follows:
  // A --------------------------- B
  struct SegmentData {
    // {route index of point A (start), route index of point B (end)}
    size_t start_idx;
    size_t end_idx;
    // {Speed in m/s at point A, Speed in m/s at point B}
    double start_speed;
    double end_speed;
    // Acceleration of the segment in m/s^2. Note that
    // |acceleration_value| >= |acceleration to travel from A to B|
    double acceleration_value;
    // Distance from A and B in meters
    double distance;
    // Whether the segment includes a corner of the route
    bool includes_corner;
    // This field is used only for the "raw" loops

    // A crossover segment is one that crosses the start line (route index 0)
    // start_idx | end_idx
    //          ...
    //    568       578
    //    578       6      -> crossover segment
    bool is_crossover_segment() {
      return end_idx < start_idx;
    }

    SegmentData(size_t start_idx = 0, size_t end_idx = 0,
                double start_speed = 0.0, double end_speed = 0.0,
                double acceleration_value = 0.0,
                double distance = -1.0,
                bool includes_corner = false) : start_idx(start_idx),
                end_idx(end_idx), start_speed(start_speed), end_speed(end_speed),
                acceleration_value(acceleration_value), distance(distance),
                includes_corner(includes_corner) {
    }
  };

  using LoopData = std::vector<SegmentData>;
  using PlanData = std::vector<LoopData>;

  RacePlan() : empty(true) {}
  RacePlan(PlanData segments, PlanData orig_segments = {}, int num_repetitions = 1);

  explicit RacePlan(std::string inviability_reason) : empty(true) {
    reason_for_inviability = inviability_reason; viable = false;
  }

  inline PlanData get_segments() const {return segments;}
  inline PlanData get_orig_segments() const {return orig_segments;}
  inline std::string get_inviability_reason() const {return reason_for_inviability;}
  inline double get_accumulated_distance() const {return accumulated_distance;}
  inline double get_driving_time() const {return driving_time;}
  inline double get_average_speed() const {return average_speed;}
  inline double get_score() const {return score;}
  inline int get_num_loops() const {return num_loops;}
  inline int get_num_blocks() const {return num_blocks;}
  inline bool is_viable() const {return viable;}
  inline bool is_empty() const {return empty;}

  inline void set_segments(PlanData new_segments) {segments = new_segments;}
  inline void set_orig_segments(PlanData new_segments) {orig_segments = new_segments;}
  inline void set_time_taken(time_t new_time_taken) {time_taken = new_time_taken;}
  inline void set_viability(bool viability) {viable = viability;}
  inline void set_num_loops(int loops) {num_loops = loops;}
  inline void set_inviability_reason(std::string message) {reason_for_inviability = message;}
  inline void set_accumulated_distance(double distance) {accumulated_distance = distance;}
  inline void set_driving_time(double time) {driving_time = time;}
  inline void set_average_speed(double speed) {average_speed = speed;}
  inline void set_score(double new_score) {score = new_score;}

  /** @brief Validate members of a race plan. Should be called before run_sim()
   *
   * @param route_points Coordinate points of the base route
   * @return True if all members are valid
   */
  bool validate_members(const std::vector<Coord>& route_points) const;

  /** @brief Print the route plan to stdout */
  void print_plan() const;
  std::string get_plan_string() const;
  std::string get_orig_plan_string() const;  // Same as above but with orig_segments

  /** @brief Get a single string displaying a race plan loop in a human readable way */
  static std::string get_loop_string(LoopData loop);

  /** @brief Get a single string displaying a segment in a human readable way */
  static std::string get_segment_string(SegmentData seg);

  void export_json() const; // Exports RacePlan to a json file in the dump directory

 private:
  /* Viability of race plan */
  bool viable = false;

  /* Time taken to complete the race in seconds using this plan */
  time_t time_taken;

  /* Total driving time in seconds */
  double driving_time;

  /* Total distance travelled in m */
  double accumulated_distance;

  /* Average speed of the car ONLY WHEN driving in m/s */
  double average_speed;

  /* Score of the race plan in comparison to other race plans */
  double score;

  /* Hold the entire race plan as a 2D matrix where each row is a single loop */
  PlanData segments;

  /* Same as above before any loop gluing occurred */
  PlanData orig_segments;

  // Number of loops completed by the car i.e. segments.size()
  int num_loops;

  // Number of loop blocks
  int num_blocks;

  // Number of repetitions per loop block
  int num_repetitions;

  // If the plan is not viable, this string holds the reason
  std::string reason_for_inviability = "";

  // Represents an empty race plan
  bool empty;
};

/** A class to hold all points and control stop locations in a race route */
class Route {
 private:
  /* Indices of all control stops */
  std::unordered_set<size_t> control_stops;

  /* Points of the route */
  std::vector<Coord> route_points;

  /* Number of points along the route */
  size_t num_points;

  /* Total length of the route */
  double route_length;

  /* Cornering segments */
  std::vector<std::pair<size_t, size_t>> cornering_segment_bounds;
  std::unordered_set<size_t> corner_start_indices;
  std::unordered_set<size_t> corner_end_indices;

  /* Straight segments - straight_segment_bounds[i] denotes the straight segment between
     corner index i - 1 and corner index i */
  std::vector<std::pair<size_t, size_t>> straight_segment_bounds;

  /* Map of corner ending indices to index in straight_segment_bounds - used for mutation optimizer */
  std::unordered_map<size_t, size_t> corner_end_to_corner_idx;

  /* Map of corner starting indices to index in straight_segment_bounds - used for mutation optimizer*/
  std::unordered_map<size_t, size_t> corner_start_to_corner_idx;

  /* Maximum speed of cornering segments */
  std::vector<double> cornering_speed_bounds;

  /* Lookup table for all distances between any two points. Should be num_points x num_points*/
  BasicLut route_distances;

  /* Longest straight path */
  double longest_straight;

  /* Maximum speed of the route (mps) - comes from regulations */
  double max_route_speed;

  /* ONLY USED FOR TELEMETRY ROUTE */
  std::vector<Time> timestamps;          // Timestamp assigned to each coordinate
  std::vector<double> speeds;             // Speed assigned to each coordinate

 public:
  /** @brief Load information about the race route
   * 
   * @param route_path: Absolute path to the base route csv
   * @param telem_flow: For telemetry simulation flow, there are two extra columns - one for time, the other for speed
   * @param init_control_stops: Initialize control stop indices from config file
   * @param cornering_bounds_path: Absolute path to the cornering bounds. If empty, don't load
   * @param precomputed_distances_path: Absolute path to the precomputed distances csv. If empty or precompute_distances
   * is true, don't load
   * @param precompute_distances: Whether to pre-compute distances. If precomputed_distances_path is not empty, it
   * will save the csv to that path
   */
  Route(const std::filesystem::path route_path, bool telem_flow = false,
        const bool init_control_stops = false,
        const std::filesystem::path cornering_bounds_path = std::filesystem::path(),
        const std::filesystem::path precomputed_distances_path = std::filesystem::path(),
        const bool precompute_distances = false);

  /** @brief initiate the base route csv |latitude|longitude|altitude|
   * @param route_path absolute path to .csv
   * @param telem_flow if there are two extra columns for time and speed respectively
  */
  void init_base_route(const std::filesystem::path route_path, bool telem_flow);

  /** @brief Initialize control stops */
  void init_control_stops();

  /** @brief Read a csv of corner index bounds with columns |starting index|ending index|max speed(mps)|
   * Note: This function also fills the corner_end_to_corner_idx map and the corner_start_to_corner_idx_map
   * @param cornering_bounds_path: Path to the csv file
   * @param max_car_speed: Maximum car speed to limit cornering speeds. Defaults to infinity
  */
  void init_cornering_bounds(const std::filesystem::path cornering_bounds_path,
                             double max_car_speed = std::numeric_limits<double>::infinity());

  /** @brief Read a num_points x num_points csv of pre-computed distances */
  void init_precomputed_distances(const std::filesystem::path precomputed_distances_path);

  /** @brief Find the longest straight in the route */
  void init_longest_straight();

  /** @brief Pre-compute distances between every possible pair of indices inside the route
   * 
   * @param csv_path: Path for the save location of the pre-computed distance csv
   */
  void precompute_distances(const std::filesystem::path csv_path = std::filesystem::path());

  Route() {}

  /** @brief Segment the route into uniform lengths
   * 
   * @param length The length of each segment. If the total length of the base route cannot be divided
   * into equal lengths, the last segment will be shorter than the rest
   */
  std::vector<std::pair<size_t, size_t>> segment_route_uniform(double length);

  /** @brief Find segment length
   *
   * @param start_idx Start index of the segment
   * @param end_idx End index of the segment
   */
  double get_segment_length(const size_t start_idx, const size_t end_idx) const;

  /* Getters */
  inline std::unordered_set<size_t> get_control_stops() const {return control_stops;}
  inline const std::vector<Coord>& get_route_points() {return route_points;}
  inline std::vector<Coord> get_route_points() const {return route_points;}
  inline size_t get_num_points() const {return num_points;}
  inline double get_route_length() const {return route_length;}
  inline const BasicLut& get_precomputed_distances() {return route_distances;}
  inline const double get_max_route_speed() {return max_route_speed;}
  inline std::unordered_map<size_t, size_t> get_corner_end_map() const {
    return corner_end_to_corner_idx;
  }
  inline const std::unordered_map<size_t, size_t> get_corner_start_map() const {
    return corner_start_to_corner_idx;
  }
  inline const std::vector<std::pair<size_t, size_t>> get_cornering_segment_bounds() const {
    return cornering_segment_bounds;
  }
  inline const std::vector<double> get_cornering_speed_bounds() const {
    return cornering_speed_bounds;
  }
  inline const std::vector<Time> get_timestamps() const {
    return timestamps;
  }
  inline const std::vector<double> get_speeds() const {
    return speeds;
  }
  inline const std::unordered_set<size_t> get_corner_start_indices() const {
    return corner_start_indices;
  }
  inline const std::unordered_set<size_t> get_corner_end_indices() const {
    return corner_end_indices;
  }

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

  /** @brief Calculate the distance from starting_idx to ending_idx inclusive in meters */
  double calc_segment_distance(const size_t starting_idx,
                               const size_t ending_idx);
};
