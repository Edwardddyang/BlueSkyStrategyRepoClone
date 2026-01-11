/** Classes used to represent a Race Plan (Speed Profile) for each race */

#pragma once

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "SimUtils/Types.hpp"
#include "route/Route.hpp"

/**
 * The fundamental unit of a route, representing a series of straight line
 * connections between two coordinates A and B: A --------------------------- B
 *
 * Also captures the method by which the car travels the segment - either at one
 * constant speed or accelerating/decelerating
 */
struct BaseSegment {
  // route index of point A (start), route index of point B (end)
  size_t start_idx;
  size_t end_idx;
  // Speed in m/s at point A, and speed in m/s at point
  // BMicrosoft.QuickAction.BatterySaver
  double start_speed;
  double end_speed;
  // Distance from A to B in meters
  double distance;
  // Acceleration of the segment in m/s^2. If this is 0, then
  // the car takes this segment at constant speed
  double acceleration_value;

  explicit BaseSegment(size_t start_idx = 0, size_t end_idx = 0,
                       double start_speed = 0.0, double end_speed = 0.0,
                       double acceleration_value = 0.0, double distance = 0.0)
      : start_idx(start_idx),
        end_idx(end_idx),
        start_speed(start_speed),
        end_speed(end_speed),
        distance(distance),
        acceleration_value(acceleration_value) {}
};

/** FSGP Segments also include corners and other data used during optimization.
 * They can also be dual segments, where the car accelerates at
 * `acceleration_value` m/s^2 until end_speed, and maintains end_speed for the
 * rest of the distance
 */
struct FSGPSegment : BaseSegment {
  // List of corner indices that this segment traverses
  std::vector<size_t> corners;
  // Whether the segment is a dual segment
  bool is_dual_segment;
  // If this is a dual segment, record the acceleration and constant speed
  // distances
  double acceleration_distance;
  double constant_distance;
  // A crossover segment is one that crosses the start line (route index 0)
  // start_idx | end_idx
  //          ...
  //    568       578
  //    578       6      -> crossover segment
  [[nodiscard]] bool is_crossover_segment() const {
    return end_idx < start_idx;
  }
  explicit FSGPSegment(size_t start_idx = 0, size_t end_idx = 0,
                       double start_speed = 0.0, double end_speed = 0.0,
                       double acceleration_value = 0.0, double distance = 0.0,
                       double acceleration_distance = 0.0,
                       double constant_distance = 0.0,
                       std::vector<size_t> corners = {},
                       bool is_dual_segment = false)
      : BaseSegment(start_idx, end_idx, start_speed, end_speed,
                    acceleration_value, distance),
        corners(std::move(corners)),
        is_dual_segment(is_dual_segment),
        acceleration_distance(acceleration_distance),
        constant_distance(constant_distance) {}
};

// ASC Segments are differentiated based on whether they are a segment
// for loops or the base legs
struct ASCSegment : BaseSegment {
  enum class SegmentType : uint8_t {
    LOOP_SEGMENT = 0,
    BASE_LEG_SEGMENT = 1,
  };

  SegmentType segment_type;

  // If this is a segment of a loop, this member indicates
  // the loop index defined in the route class
  size_t loop_idx;

  explicit ASCSegment(size_t start_idx = 0, size_t end_idx = 0,
                      double start_speed = 0.0, double end_speed = 0.0,
                      double acceleration_value = 0.0, double distance = 0.0,
                      SegmentType segment_type = SegmentType::LOOP_SEGMENT,
                      size_t loop_idx = 0)
      : BaseSegment(start_idx, end_idx, start_speed, end_speed,
                    acceleration_value, distance),
        segment_type(segment_type),
        loop_idx(loop_idx) {}
};

/** A class to represent a proposed race plan. It is made of up of a series of
 * segments */
template <typename DerivedRacePlan>
class BaseRacePlan {
 private:
  BaseRacePlan() = default;
  friend DerivedRacePlan;

  explicit BaseRacePlan(std::string inviability_reason)
      : reason_for_inviability(std::move(inviability_reason)) {}

 protected:
  /* Viability of race plan */
  bool viable = false;

  /* Time taken to complete the race in seconds using this plan in seconds */
  double driving_time = 0.0;

  /* Total distance travelled in m */
  double accumulated_distance = 0.0;

  /* Average speed of the car ONLY when driving in m/s */
  double average_speed = 0.0;

  /* Empty race plan */
  bool empty = true;

  /* Start/end time of the race plan */
  util::type::Time start_time;
  util::type::Time end_time;

  /* If the plan is not viable, this string holds the reason */
  std::string reason_for_inviability;

  /* Name of the file name dump */
  std::string dump_file_name;

 public:
  /** @brief Export RacePlan to a json file */
  void export_json(std::filesystem::path dump_path) const;

  /** @brief Transform race plan into a readable format */
  [[nodiscard]] std::string get_plan_string() const;

  void print_plan() const;

  /** @brief Validate race plan */
  template <RouteType R>
  bool validate_members(const R& route) const;

  [[nodiscard]] bool is_viable() const { return viable; }
  [[nodiscard]] double get_driving_time() const { return driving_time; }
  [[nodiscard]] double get_accumulated_distance() const {
    return accumulated_distance;
  }
  [[nodiscard]] double get_average_speed() const { return average_speed; }
  [[nodiscard]] util::type::Time get_start_time() const { return start_time; }
  [[nodiscard]] util::type::Time get_end_time() const { return end_time; }
  [[nodiscard]] std::string get_dump_file_name() const {
    return dump_file_name;
  }
  [[nodiscard]] bool is_empty() const { return empty; }
  [[nodiscard]] std::string get_inviability_reason() const {
    return reason_for_inviability;
  }

  void set_viability(bool viability) { viable = viability; }
  void set_driving_time(double time) { driving_time = time; }
  void set_start_time(util::type::Time new_start_time) {
    start_time = new_start_time;
  }
  void set_end_time(util::type::Time new_end_time) { end_time = new_end_time; }
  void set_accumulated_distance(double distance) {
    accumulated_distance = distance;
  }
  void set_average_speed(double speed) { average_speed = speed; }
  void set_dump_file_name(std::string new_dump_file_name) {
    dump_file_name = std::move(new_dump_file_name);
  }
  void set_empty(bool is_empty) { empty = is_empty; }
  void set_inviability_reason(std::string reason) {
    reason_for_inviability = std::move(reason);
  }
};

/** A class to represent a race plan for WSC */
class WSCRacePlan : public BaseRacePlan<WSCRacePlan> {
 public:
  using SegmentsVec = std::vector<BaseSegment>;

 private:
  // Single flattened array of segments
  SegmentsVec segments;

 public:
  explicit WSCRacePlan(SegmentsVec plan_segments);
  explicit WSCRacePlan(std::string reason) : BaseRacePlan(std::move(reason)) {}
  WSCRacePlan() = default;

  [[nodiscard]] const SegmentsVec& get_segments() const { return segments; }
  void set_segments(SegmentsVec new_segments) {
    segments = std::move(new_segments);
  }

  void export_metadata_json_impl(json& json_obj) const;
  void export_segments_json_impl(json& json_obj) const;

  [[nodiscard]] std::string get_plan_string_impl() const;

  [[nodiscard]] bool validate_members_impl(const WSCRoute& route) const;
};

/** A class to represent a race plan for FSGP
 *
 * The plan is split into a series of blocks, where each block holds
 * n loops of the same speed profile
 */
class FSGPRacePlan : public BaseRacePlan<FSGPRacePlan> {
 public:
  using LoopData = std::vector<FSGPSegment>;
  using PlanData = std::vector<LoopData>;

 private:
  // 2D matrix of segments, where each row is a loop around the track
  PlanData segments;

  // Total number of loops
  size_t num_loops = 0;

  // Number of blocks
  uint32_t num_blocks = 0;

  // Number of loops per block
  uint32_t num_repetitions = 0;

 public:
  FSGPRacePlan(PlanData segments, int num_repetitions);
  explicit FSGPRacePlan(std::string reason) : BaseRacePlan(std::move(reason)) {}

  // Getters/Setters
  [[nodiscard]] const PlanData& get_segments() const { return segments; }
  [[nodiscard]] size_t get_num_loops() const { return segments.size(); }
  [[nodiscard]] uint32_t get_num_blocks() const { return num_blocks; }
  [[nodiscard]] uint32_t get_num_repetitions() const { return num_repetitions; }

  void set_segments(PlanData new_segments) {
    segments = std::move(new_segments);
  }
  void set_num_blocks(uint32_t new_num_blocks) { num_blocks = new_num_blocks; }
  void set_num_repetitions(uint32_t new_num_repetitions) {
    num_repetitions = new_num_repetitions;
  }

  /** Hooks into the base interface */
  void export_metadata_json_impl(json& json_obj) const;
  void export_segments_json_impl(json& json_obj) const;
  [[nodiscard]] std::string get_plan_string_impl() const;

  /** @brief Given segments for a single loop around the track,
   * print them into a human readable table
   */
  [[nodiscard]] static std::string get_loop_string(
      const LoopData& loop_segments);
  [[nodiscard]] bool validate_members_impl(const FSGPRoute& route) const;
};

class ASCRacePlan : public BaseRacePlan<ASCRacePlan> {
 public:
  using LegSegments = std::vector<ASCSegment>;

 private:
  // Segments for the base legs. base_segments[i] holds segments
  // for the i-th base leg
  std::vector<LegSegments> base_segments;

  // Segments for each loop. loop_segments[i] holds segments
  // for the i-th base leg
  std::vector<LegSegments> loop_segments;

  // Number of times each loop is taken - num_loop_iterations[i]
  // is the number of times that loop i is repeated using the segments
  // in loop_segments[i]
  std::vector<size_t> num_loop_iterations;

 public:
  ASCRacePlan(std::vector<LegSegments> base_segments,
              std::vector<LegSegments> loop_segments,
              std::vector<size_t> num_loop_iterations);

  explicit ASCRacePlan(std::string reason) : BaseRacePlan(std::move(reason)) {}

  // Getters/Setters
  [[nodiscard]] const std::vector<LegSegments>& get_base_segments() const {
    return base_segments;
  }
  [[nodiscard]] const std::vector<LegSegments>& get_loop_segments() const {
    return loop_segments;
  }
  [[nodiscard]] const std::vector<size_t>& get_num_loop_iterations() const {
    return num_loop_iterations;
  }

  void set_base_segments(std::vector<LegSegments> new_segments) {
    base_segments = std::move(new_segments);
  }
  void set_loop_segments(std::vector<LegSegments> new_segments) {
    loop_segments = std::move(new_segments);
  }
  void set_num_loop_repetitions(std::vector<size_t> new_iters) {
    num_loop_iterations = std::move(new_iters);
  }

  // Gonna need to change these two functions when we figure out
  // how we wanna define the json layout
  void export_metadata_json_impl(json& json_obj) const;

  // TODO: Implement this
  void export_segments_json_impl(json& json_obj) const;
  // TODO: Implement this
  [[nodiscard]] std::string get_plan_string_impl() const;

  // TODO: Implement this
  [[nodiscard]] bool validate_members_impl(const ASCRoute& route) const;
};

template <typename T>
concept RacePlanType =
    std::is_same_v<ASCRacePlan, T> || std::is_same_v<WSCRacePlan, T> ||
    std::is_same_v<FSGPRacePlan, T>;

#include "route/RacePlan.tpp"  // NOLINT [misc-include-cleaner]
