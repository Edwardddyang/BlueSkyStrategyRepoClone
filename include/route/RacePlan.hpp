/** Classes used to represent a Race Plan (Speed Profile) for each race */

#pragma once

#include <stdlib.h>
#include "route/Route.hpp"

/**
 * The fundamental unit of a route, representing a series of straight line connections
 * between two coordinates A and B:
 * A --------------------------- B
 *
 * Also captures the method by which the car travels the segment - either at one constant
 * speed or accelerating/decelerating
 */
struct BaseSegment {
  // route index of point A (start), route index of point B (end)
  size_t start_idx;
  size_t end_idx;
  // Speed in m/s at point A, and speed in m/s at point BMicrosoft.QuickAction.BatterySaver
  double start_speed;
  double end_speed;
  // Distance from A to B in meters
  double distance;
  // Acceleration of the segment in m/s^2. If this is 0, then
  // the car takes this segment at constant speed
  double acceleration_value;

  BaseSegment(size_t start_idx = 0, size_t end_idx = 0,
              double start_speed = 0.0, double end_speed = 0.0,
              double acceleration_value = 0.0,
              double distance = 0.0) : start_idx(start_idx),
              end_idx(end_idx), start_speed(start_speed),
              end_speed(end_speed), acceleration_value(acceleration_value),
              distance(distance) {}
};

/** FSGP Segments also include corners and other data used during optimization.
* They can also be dual segments, where the car accelerates at `acceleration_value`
* m/s^2 until end_speed, and maintains end_speed for the rest of the distance
*/
struct FSGPSegment : BaseSegment {
  // List of corner indices that this segment traverses
  std::vector<size_t> corners;
  // Whether the segment is a dual segment
  bool is_dual_segment;
  // If this is a dual segment, record the acceleration and constant speed distances
  double acceleration_distance;
  double constant_distance;
  // A crossover segment is one that crosses the start line (route index 0)
  // start_idx | end_idx
  //          ...
  //    568       578
  //    578       6      -> crossover segment
  bool is_crossover_segment() const {
    return end_idx < start_idx;
  }
  FSGPSegment(size_t start_idx = 0, size_t end_idx = 0,
              double start_speed = 0.0, double end_speed = 0.0,
              double acceleration_value = 0.0, double distance = 0.0,
              double acceleration_distance = 0.0, double constant_distance = 0.0,
              std::vector<size_t> corners = {},
              bool is_dual_segment = false) : BaseSegment(
                start_idx, end_idx, start_speed, end_speed, acceleration_value,
                distance), corners(std::move(corners)),
                is_dual_segment(is_dual_segment),
                acceleration_distance(acceleration_distance),
                constant_distance(constant_distance) {}
};

// ASC Segments are differentiated based on whether they are a segment
// for loops or the base legs
struct ASCSegment : BaseSegment {
  enum class SegmentType {
    LOOP_SEGMENT = 0,
    BASE_LEG_SEGMENT = 1,
  };

  SegmentType segment_type;

  // If this is a segment of a loop, this member indicates
  // the loop index defined in the route class
  size_t loop_idx;

  ASCSegment(size_t start_idx = 0, size_t end_idx = 0,
             double start_speed = 0.0, double end_speed = 0.0,
             double acceleration_value = 0.0, double distance = 0.0,
             SegmentType segment_type = SegmentType::LOOP_SEGMENT,
             size_t loop_idx = 0
            ) : BaseSegment(
                start_idx, end_idx, start_speed, end_speed, acceleration_value,
                distance),
              segment_type(segment_type), loop_idx(loop_idx) {}
};

/** A class to represent a proposed race plan. It is made of up of a series of segments */
template <typename DerivedRacePlan>
class BaseRacePlan {
 protected:
  /* Viability of race plan */
  bool viable;

  /* Time taken to complete the race in seconds using this plan in seconds */
  double driving_time;

  /* Total distance travelled in m */
  double accumulated_distance;

  /* Average speed of the car ONLY when driving in m/s */
  double average_speed;

  /* Empty race plan */
  bool empty;

  /* Start/end time of the race plan */
  util::type::Time start_time;
  util::type::Time end_time;

  /* If the plan is not viable, this string holds the reason */
  std::string reason_for_inviability = "";

  /* Name of the file name dump */
  std::string dump_file_name = "";

 public:
  BaseRacePlan() : viable(false) {}
  explicit BaseRacePlan(std::string inviability_reason) :
           empty(true), reason_for_inviability(std::move(inviability_reason)),
           viable(false) {}

  /** @brief Export RacePlan to a json file */
  void export_json(std::filesystem::path dump_file) const;

  /** @brief Transform race plan into a readable format */
  std::string get_plan_string() const;

  void print_plan() const;

  /** @brief Validate race plan */
  template <RouteType R>
  bool validate_members(const R& route) const;

  inline bool is_viable() const {return viable;}
  inline double get_driving_time() const {return driving_time;}
  inline double get_accumulated_distance() const {return accumulated_distance;}
  inline double get_average_speed() const {return average_speed;}
  inline util::type::Time get_start_time() const {return start_time;}
  inline util::type::Time get_end_time() const {return end_time;}
  inline std::string get_dump_file_name() const {return dump_file_name;}
  inline bool is_empty() const {return empty;}
  inline std::string get_inviability_reason() const {return reason_for_inviability;}

  inline void set_viability(bool viability) {viable = viability;}
  inline void set_driving_time(double time) {driving_time = time;}
  inline void set_start_time(util::type::Time new_start_time) {start_time = std::move(new_start_time);}
  inline void set_end_time(util::type::Time new_end_time) {end_time = std::move(new_end_time);}
  inline void set_accumulated_distance(double distance) {accumulated_distance = distance;}
  inline void set_average_speed(double speed) {average_speed = speed;}
  inline void set_dump_file_name(std::string new_dump_file_name) {dump_file_name = std::move(new_dump_file_name);}
  inline void set_empty(bool is_empty) {empty = is_empty;}
  inline void set_inviability_reason(std::string reason) {reason_for_inviability = reason;}
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
  explicit WSCRacePlan(std::string reason) : BaseRacePlan(reason) {}
  WSCRacePlan() = default;

  const SegmentsVec& get_segments() const { return segments; }
  void set_segments(SegmentsVec new_segments) { segments = std::move(new_segments); }

  void export_metadata_json_impl(json& json_obj) const;
  void export_segments_json_impl(json& json_obj) const;

  std::string get_plan_string_impl() const;

  bool validate_members_impl(const WSCRoute& route) const;
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
  size_t num_loops;

  // Number of blocks
  uint32_t num_blocks;

  // Number of loops per block
  uint32_t num_repetitions;

 public:
  FSGPRacePlan(PlanData segments, int num_repetitions);
  explicit FSGPRacePlan(std::string reason) :
           BaseRacePlan(reason), num_loops(0),
           num_repetitions(0), num_blocks(0) {}

  // Getters/Setters
  const PlanData& get_segments() const {return segments;}
  size_t get_num_loops() const {return segments.size();}
  uint32_t get_num_blocks() const {return num_blocks;}
  uint32_t get_num_repetitions() const {return num_repetitions;}

  void set_segments(PlanData new_segments) {segments = std::move(new_segments);}
  void set_num_blocks(uint32_t new_num_blocks) {num_blocks = new_num_blocks;}
  void set_num_repetitions(uint32_t new_num_repetitions) {num_repetitions = new_num_repetitions;}

  /** Hooks into the base interface */
  void export_metadata_json_impl(json& json_obj) const;
  void export_segments_json_impl(json& json_obj) const;
  std::string get_plan_string_impl() const;

  /** @brief Given segments for a single loop around the track,
  * print them into a human readable table
  */
  static std::string get_loop_string(const LoopData& loop_segments);
  bool validate_members_impl(const FSGPRoute& route) const;
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

  explicit ASCRacePlan(std::string reason) : BaseRacePlan(reason) {}

  // Getters/Setters
  const std::vector<LegSegments>& get_base_segments() const {return base_segments;}
  const std::vector<LegSegments>& get_loop_segments() const {return loop_segments;}
  const std::vector<size_t>& get_num_loop_iterations() const {return num_loop_iterations;}

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
  std::string get_plan_string_impl() const;

  // TODO: Implement this
  bool validate_members_impl(const ASCRoute& route) const;
};

template <typename T>
concept RacePlanType = std::is_same_v<ASCRacePlan, T> ||
                       std::is_same_v<WSCRacePlan, T> ||
                       std::is_same_v<FSGPRacePlan, T>;

#include "route/RacePlan.tpp"
