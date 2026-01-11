#include "route/RacePlan.hpp"

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "SimUtils/Constants.hpp"
#include "SimUtils/Defines.hpp"
#include "SimUtils/Utilities.hpp"
#include "nlohmann/json.hpp"  // IWYU pragma: keep
#include "route/Route.hpp"

WSCRacePlan::WSCRacePlan(SegmentsVec plan_segments)
    : segments(std::move(plan_segments)) {
  this->empty = segments.empty();
}

void WSCRacePlan::export_metadata_json_impl(json& json_obj) const {
  json_obj.push_back(
      {{"avg_speed", util::constants::mps2kph(this->average_speed)},
       {"start_time", this->start_time.get_local_string()},
       {"end_time", this->end_time.get_local_string()}});
}

void WSCRacePlan::export_segments_json_impl(json& json_obj) const {
  for (const BaseSegment& seg : segments) {
    json_obj.push_back(
        {{"start_idx", seg.start_idx},
         {"end_idx", seg.end_idx},
         {"start_speed", util::constants::mps2kph(seg.start_speed)},
         {"end_speed", util::constants::mps2kph(seg.end_speed)},
         {"acceleration_value", seg.acceleration_value},
         {"distance", seg.distance}});
  }
}

std::string WSCRacePlan::get_plan_string_impl() const {
  RUNTIME_EXCEPTION(!segments.empty(), "Plan must have >= 1 segments");
  std::stringstream output;

  auto print_header = [&]() {
    output << "+-------+-----------+---------------+-----------------------+---"
              "------------+\n";
    output << ";  Idx  ; Segments  ; Speeds (m/s)  ; Acceleration (m/s^2)  ; "
              "Distances (m) ;\n";
    output << "+-------+-----------+---------------+-----------------------+---"
              "------------+\n";
  };

  print_header();

  const size_t num_segments = segments.size();
  for (size_t seg_idx = 0; seg_idx < num_segments; seg_idx++) {
    // Index (seg_idx)
    output << "|";
    append_char_n_times(' ', 1, output);
    output << std::setw(3) << seg_idx;
    append_char_n_times(' ', 3, output);

    // Segment indices
    output << "| [" << std::setw(3) << segments[seg_idx].start_idx << ","
           << std::setw(3) << segments[seg_idx].end_idx << "] |";

    // Segment speeds
    append_char_n_times(' ', 1, output);
    output << "[" << std::setw(5)
           << truncate_number(segments[seg_idx].start_speed, 5) << ","
           << std::setw(5) << truncate_number(segments[seg_idx].end_speed, 5)
           << "] |";

    // Segment acceleration
    append_char_n_times(' ', 18, output);
    output << std::setw(5)
           << truncate_number(segments[seg_idx].acceleration_value, 5);
    output << "|";

    // Truncate the distance
    append_char_n_times(' ', 10, output);
    output << std::setw(10) << truncate_number(segments[seg_idx].distance, 10);
    output << "|\n";
  }
  append_char_n_times('-', 77, output);
  output << "\n";

  return output.str();
}

bool WSCRacePlan::validate_members_impl(const WSCRoute& route) const {
  RUNTIME_EXCEPTION(!this->empty, "WSCRacePlan is empty");
  RUNTIME_EXCEPTION(!segments.empty(), "No segments in WSCRacePlan");
  const CoordVec& route_points = route.get_route_points();

  const size_t num_segments = segments.size();
  const double tolerance =
      0.0001;  // Tolerance for comparing acceleration values

  for (size_t seg_idx = 0; seg_idx < num_segments; seg_idx++) {
    const size_t start_idx = segments[seg_idx].start_idx;
    const size_t end_idx = segments[seg_idx].end_idx;
    const double start_speed = segments[seg_idx].start_speed;
    const double end_speed = segments[seg_idx].end_speed;
    const double acceleration = segments[seg_idx].acceleration_value;
    const double distance = segments[seg_idx].distance;

    const double segment_distance =
        calculate_segment_distance(route_points, start_idx, end_idx);

    RUNTIME_EXCEPTION(
        std::abs(segment_distance - distance) < tolerance,
        "Segment {} has inconsistent distance. "
        "Calculated segment distance = {}, Distance attribute = {}",
        seg_idx, segment_distance, distance);
    RUNTIME_EXCEPTION(start_speed >= 0.0 && end_speed >= 0.0,
                      "Segment {} has start_speed = {}, end_speed = {}. "
                      "Both must be >= 0.0",
                      seg_idx, start_speed, end_speed);
    RUNTIME_EXCEPTION(start_idx <= end_idx,
                      "Segment {} has start_idx = {}, end_idx = {}", seg_idx,
                      start_idx, end_idx);
    if (seg_idx < num_segments - 1) {
      RUNTIME_EXCEPTION(
          end_idx == segments[seg_idx + 1].start_idx,
          "Segments must be continuous i.e. segment end = next segment start."
          " Segment {} is inconsistent with next segment. Ending index = {}, "
          " Next Starting Index = {}",
          seg_idx, end_idx, segments[seg_idx + 1].start_idx);
      RUNTIME_EXCEPTION(
          end_speed == segments[seg_idx + 1].start_speed,
          "Segments must be continuous i.e. segment end speed = next segment "
          "start speed."
          " Segment {} is inconsistent with next segment. Ending speed = {}, "
          " Next Segment Starting Speed = {}",
          seg_idx, end_speed, segments[seg_idx + 1].start_speed);
    }

    if (acceleration == 0.0) {
      RUNTIME_EXCEPTION(
          start_speed == end_speed,
          "Speed profile for non-acceleration segment {} must have equal and "
          "positive "
          "start and end speeds. Start speed = {}, End Speed = {}",
          seg_idx, start_speed, end_speed);
    } else {
      // Verify acceleration value
      RUNTIME_EXCEPTION(start_speed != end_speed,
                        "Acceleration segment {} must have different start and "
                        "ending speeds.",
                        seg_idx);
      const double calculated_acceleration_distance =
          util::calc_distance_a(start_speed, end_speed, acceleration);
      RUNTIME_EXCEPTION(
          std::abs(calculated_acceleration_distance - segment_distance) <
              tolerance,
          "Acceleration segment {} has inconsistent acceleration values. "
          "Calculated acceleration distance is {}, "
          "acceleration distance attribute is {}",
          seg_idx, calculated_acceleration_distance, segment_distance);
    }
  }
  return true;
}

FSGPRacePlan::FSGPRacePlan(PlanData segments, int num_repetitions)
    : segments(std::move(segments)),
      num_loops(this->segments.size()),
      num_repetitions(num_repetitions) {
  RUNTIME_EXCEPTION(num_repetitions >= 1,
                    "Number of repetitions per loop block be at least 1");
  this->num_blocks = static_cast<int>(std::ceil(num_loops / num_repetitions));
  this->empty = this->segments.empty();
}

void FSGPRacePlan::export_metadata_json_impl(json& json_obj) const {
  json_obj.push_back({{"avg_speed", util::constants::mps2kph(average_speed)},
                      {"num_repetitions", num_repetitions},
                      {"num_loops", num_loops},
                      {"start_time", start_time.get_local_string()},
                      {"end_time", end_time.get_local_string()}});
}

void FSGPRacePlan::export_segments_json_impl(json& json_obj) const {
  for (const auto& loop : segments) {
    json loop_json = json::array();
    // First element is always metadata
    RUNTIME_EXCEPTION(!loop.empty(), "Loop must have >= 1 segment");
    for (const auto& seg : loop) {
      loop_json.push_back(
          {{"start_idx", seg.start_idx},
           {"end_idx", seg.end_idx},
           {"start_speed", util::constants::mps2kph(seg.start_speed)},
           {"end_speed", util::constants::mps2kph(seg.end_speed)},
           {"acceleration_value", seg.acceleration_value},
           {"distance", seg.distance},
           {"corner_indices", json(seg.corners)}});
    }
    json_obj.push_back(loop_json);
  }
}

std::string FSGPRacePlan::get_loop_string(const LoopData& loop_segments) {
  std::stringstream output;

  auto print_header = [&]() {
    output << "+-------+-----------+---------------+-----------------------+---"
              "------------+-------------\n";
    output << ";  Idx  ; Segments  ; Speeds (m/s)  ; Acceleration (m/s^2)  ; "
              "Distances (m) ; Corners    ;\n";
    output << "+-------+-----------+---------------+-----------------------+---"
              "------------+-------------\n";
  };

  print_header();

  const size_t num_segments = loop_segments.size();
  for (size_t seg_idx = 0; seg_idx < num_segments; seg_idx++) {
    // Index (seg_idx)
    output << "|";
    append_char_n_times(' ', 1, output);
    output << std::setw(3) << seg_idx;
    append_char_n_times(' ', 3, output);

    // Segment indices
    output << "| [" << std::setw(3) << loop_segments[seg_idx].start_idx << ","
           << std::setw(3) << loop_segments[seg_idx].end_idx << "] |";

    // Segment speeds
    append_char_n_times(' ', 1, output);
    output << "[" << std::setw(5)
           << truncate_number(loop_segments[seg_idx].start_speed, 5) << ","
           << std::setw(5)
           << truncate_number(loop_segments[seg_idx].end_speed, 5) << "] |";

    // Segment acceleration
    append_char_n_times(' ', 18, output);
    output << std::setw(5)
           << truncate_number(loop_segments[seg_idx].acceleration_value, 5);
    output << "|";

    // Truncate the distance
    append_char_n_times(' ', 10, output);
    output << std::setw(5)
           << truncate_number(loop_segments[seg_idx].distance, 5);
    output << "|";

    // Has Corner
    const std::string corner_str =
        util::vector_to_string<size_t>(loop_segments[seg_idx].corners);

    if (corner_str.empty()) {
      append_char_n_times(' ', 11, output);
      output << " |\n";
    } else {
      output << std::setw(11) << corner_str << " |\n";
    }
  }
  append_char_n_times('-', 90, output);
  output << "\n";

  return output.str();
}

std::string FSGPRacePlan::get_plan_string_impl() const {
  const size_t num_loops = segments.size();
  std::string ret;

  for (size_t loop_idx = 0; loop_idx < num_loops; loop_idx++) {
    ret += "\nLOOP " + std::to_string(loop_idx) + "\n";
    ret += get_loop_string(segments[loop_idx]);
  }
  return ret;
}

bool FSGPRacePlan::validate_members_impl(const FSGPRoute& route) const {
  RUNTIME_EXCEPTION(!empty, "RacePlan is empty");
  RUNTIME_EXCEPTION(!segments.empty(), "No segments in Race Plan!");
  const CoordVec& route_points = route.get_route_points();
  const std::vector<double>& cornering_speed_bounds =
      route.get_cornering_speed_bounds();

  const size_t num_loops = segments.size();
  const double tolerance =
      0.0001;  // Tolerance for comparing acceleration values

  double last_segment_end_speed = 0.0;
  for (size_t loop_idx = 0; loop_idx < num_loops; loop_idx++) {
    const LoopData loop_segments = segments[loop_idx];

    const size_t num_segments = loop_segments.size();
    if (num_loops > 1) {
      RUNTIME_EXCEPTION(
          loop_segments[num_segments - 1].end_idx == 0,
          "Last segment's ending point must be 0 i.e. wrap-around. Loop {}",
          loop_idx);
    }
    if (loop_idx > 0) {
      RUNTIME_EXCEPTION(loop_segments[0].start_speed == last_segment_end_speed,
                        "Last loop's ending speed must be the starting speed "
                        "of the first segment in the next loop. "
                        "Error found in loop {}",
                        loop_idx);
    }

    // Iterate through all segments of the loop
    for (size_t i = 0; i < num_segments; i++) {
      const size_t start_idx = loop_segments[i].start_idx;
      const size_t end_idx = loop_segments[i].end_idx;
      const double start_speed = loop_segments[i].start_speed;
      const double end_speed = loop_segments[i].end_speed;
      const double acceleration = loop_segments[i].acceleration_value;
      const double distance = loop_segments[i].distance;
      const bool is_dual_segment = loop_segments[i].is_dual_segment;
      const bool is_crossover_segment = loop_segments[i].is_crossover_segment();
      const double acceleration_distance =
          loop_segments[i].acceleration_distance;
      const double constant_distance = loop_segments[i].constant_distance;
      const std::vector<size_t>& corners = loop_segments[i].corners;

      const double segment_distance =
          calculate_segment_distance(route_points, start_idx, end_idx);
      RUNTIME_EXCEPTION(std::abs(segment_distance - distance) < tolerance,
                        "Segment distance is not equal");
      RUNTIME_EXCEPTION(start_speed >= 0.0 && end_speed >= 0.0,
                        "Segment speeds in segment {} are not positive", i);
      if (!corners.empty()) {
        RUNTIME_EXCEPTION(
            route.get_overlapping_corners({start_idx, end_idx}) == corners,
            "Segment is missing corners information");
        for (const size_t& c : corners) {
          const double max_c_speed = cornering_speed_bounds[c];
          RUNTIME_EXCEPTION(
              max_c_speed >= start_speed && max_c_speed >= end_speed,
              "Segment speed exceeds maximum cornering speed");
          RUNTIME_EXCEPTION(
              max_c_speed >
                      util::constants::kph2mps(route.get_max_route_speed()) &&
                  acceleration <= 0.0,
              "Cannot accelerate on a corner. Segment {} of loop {}", i,
              loop_idx);
        }
      }

      if (i < num_segments - 1) {
        RUNTIME_EXCEPTION(
            start_idx <= end_idx,
            "Segment start must be <= segment end in loop {} segment {}",
            loop_idx, i);
        RUNTIME_EXCEPTION(
            end_idx == loop_segments[i + 1].start_idx,
            "Segments must be continuous i.e. segment end = next segment start."
            " Invalid on segment {}, loop {}. Ending index = {}, Next Starting "
            "index = {}",
            i, loop_idx, end_idx, loop_segments[i + 1].start_idx);
        RUNTIME_EXCEPTION(end_speed == loop_segments[i + 1].start_speed,
                          "Ending speed of segment {} must equal starting "
                          "speed of next segment in loop {}. {} {}",
                          i, loop_idx, end_speed,
                          loop_segments[i + 1].start_speed);
      } else {
        // Only the last segment of a loop should be a crossover
        RUNTIME_EXCEPTION(is_crossover_segment,
                          "Last segment of loop {} is not a crossover segment",
                          loop_idx);
      }

      if (acceleration == 0.0) {
        RUNTIME_EXCEPTION(
            !is_dual_segment,
            "Segment {} with 0.0 acceleration cannot be a dual segment", i);
        RUNTIME_EXCEPTION(start_speed == end_speed,
                          "Speed profile for non-acceleration segment {} must "
                          "have equal and positive starting "
                          "and ending speeds",
                          i);
      } else {
        // Verify acceleration value
        RUNTIME_EXCEPTION(
            start_speed != end_speed,
            "Acceleration segment must have different start and ending speeds."
            "Segment {} of loop {}",
            i, loop_idx);
        const double calculated_acceleration_distance =
            util::calc_distance_a(start_speed, end_speed, acceleration);
        if (is_dual_segment) {
          RUNTIME_EXCEPTION(std::abs(acceleration_distance + constant_distance -
                                     segment_distance) < tolerance,
                            "Acceleration distance {} and constant speed "
                            "distance {} does not add "
                            "to the dual segment distance {}",
                            acceleration_distance, constant_distance,
                            segment_distance);
          RUNTIME_EXCEPTION(
              calculated_acceleration_distance == acceleration_distance,
              "Calculated acceleraiton distance {} is inconsistent in dual "
              "segment",
              calculated_acceleration_distance);
        } else {
          RUNTIME_EXCEPTION(
              std::abs(calculated_acceleration_distance - segment_distance) <
                  tolerance,
              "Acceleration distance {} is invalid. Must be equal to the "
              "segment distance {} "
              "for loop {} segment {}",
              calculated_acceleration_distance, segment_distance, loop_idx, i);
        }
      }
    }

    last_segment_end_speed = loop_segments[num_segments - 1].end_speed;
  }

  return true;
}

ASCRacePlan::ASCRacePlan(std::vector<LegSegments> base_segments,
                         std::vector<LegSegments> loop_segments,
                         std::vector<size_t> num_loop_iterations)
    : base_segments(std::move(base_segments)),
      loop_segments(std::move(loop_segments)),
      num_loop_iterations(std::move(num_loop_iterations)) {
  this->empty = this->base_segments.empty();
}

// NOLINTBEGIN(cppcoreguidelines-*, modernize-*, misc-*,
// readability-convert-member-functions-to-static)
void ASCRacePlan::export_metadata_json_impl(json& json_obj) const {
  json_obj.push_back(
      {{"avg_speed", util::constants::mps2kph(average_speed)},
       {"loop_iterations", util::vector_to_string(num_loop_iterations)}});
}

void ASCRacePlan::export_segments_json_impl(json& json_obj) const {  // NOLINT
  ;
}  // NOLINT

std::string ASCRacePlan::get_plan_string_impl() const { return ""; }  // NOLINT

bool ASCRacePlan::validate_members_impl(
    const ASCRoute& route) const {  // NOLINT
  return true;
}

// NOLINTEND(cppcoreguidelines-*, modernize-*, misc-*,
// readability-convert-member-functions-to-static)
