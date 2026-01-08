/* Singleton class that contains all parameters for a full scale session */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <filesystem>

#include "SimUtils/Types.hpp"
#include "SimUtils/Constants.hpp"
#include "SimUtils/Luts.hpp"
#include "config/ConfigParam.hpp"
#include "SimUtils/Defines.hpp"
#include <fstream>
#include <sstream>


/* Define all config parameters via. X-Macro
   PARAM(
      <parameter name as it appears exactly in the .yaml configuration file>,
   		<data type>,
		  <default value>
    )

	If the parameter you're introducing is a pointer of some type, please use unique_ptr
	as the <data type> to avoid memory leaks. If you're introducing a new datatype, you'll
  also have to make relevant changes in ConfigParam.hpp. Note that for all variables of
  std::filesystem::path type, $STRAT_ROOT will be pre-pended to the path
 */
#define CONFIG_PARAMETERS                                                                                                                                  \
    PARAM(max_soc,                       double,                  5.2)                                                                                     \
    PARAM(tire_pressure,                 double,                  5.5)                                                                                     \
    PARAM(array_area,                    double,                  3.98)                                                                                    \
    PARAM(car_mass,                      double,                  305)                                                                                     \
    PARAM(air_density,                   double,                  1.17)                                                                                    \
    PARAM(passive_electric_loss,         double,                  20)                                                                                      \
    PARAM(cda,                           double,                  0.16)                                                                                    \
    PARAM(max_motor_power,               double,                  5.0)                                                                                     \
    PARAM(max_car_speed,                 double,                  104)                                                                                     \
    PARAM(max_route_speed,               double,                  104)                                                                                     \
    PARAM(max_acceleration,              double,                  2.0)                                                                                     \
    PARAM(max_deceleration,              double,                  3.0)                                                                                     \
    PARAM(array_power_max,               double,                  950)                                                                                     \
    PARAM(array_efficiency,              double,                  0.252)                                                                                   \
    PARAM(motor_efficiency,              double,                  0.9)                                                                                     \
    PARAM(regen_efficiency,              double,                  0.8)                                                                                     \
    PARAM(battery_efficiency,            double,                  0.95)                                                                                    \
    PARAM(tire_inertia,                  double,                  0.00125)                                                                                 \
    PARAM(tire_radius,                   double,                  0.508)                                                                                   \
    PARAM(num_tires,                      int,                     3)                                                                                       \
    PARAM(max_num_loops,                 int,                     300)                                                                                     \
    PARAM(fix_num_loops,                 bool,                    true)                                                                                    \
    PARAM(control_stops,                 std::unordered_set<int>, util::convert_string_to_int_set("2962,5559,9462,11421,14439,16990,20832,23202,25987"))   \
    PARAM(init_control_stops,            bool,                    false)                                                                                   \
    PARAM(control_stop_charge_time,      int,                     30)                                                                                      \
    PARAM(base_route_path,               std::filesystem::path,   std::filesystem::path("data/luts/fsgp/static/fsgp_base_route.csv"))                       \
    PARAM(telemetry_path,                std::filesystem::path,   std::filesystem::path("data/luts/telemetry/sim.csv"))                                      \
    PARAM(corners_path,                  std::filesystem::path,   std::filesystem::path("data/luts/fsgp/static/fsgp_corners.csv"))                           \
    PARAM(route_has_corners,             bool,                    false)                                                                                   \
    PARAM(power_factor_path,             std::filesystem::path,   std::filesystem::path("data/luts/fsgp/static/powerfactor.csv"))                           \
    PARAM(roll_res_slope_path,           std::filesystem::path,   std::filesystem::path("data/luts/fsgp/static/rr2.csv"))                                   \
    PARAM(roll_res_yint_path,            std::filesystem::path,   std::filesystem::path("data/luts/fsgp/static/rr1.csv"))                                   \
    PARAM(dni_path,                      std::filesystem::path,   std::filesystem::path("data/luts/fsgp/dynamic/dni.csv"))                                   \
    PARAM(dhi_path,                      std::filesystem::path,   std::filesystem::path("data/luts/fsgp/dynamic/dhi.csv"))                                   \
    PARAM(ghi_path,                      std::filesystem::path,   std::filesystem::path("data/luts/fsgp/dynamic/shortwave_radiation.csv"))                   \
    PARAM(wind_direction_path,           std::filesystem::path,   std::filesystem::path("data/luts/fsgp/dynamic/wind_direction_10m.csv"))                    \
    PARAM(wind_speed_path,               std::filesystem::path,   std::filesystem::path("data/luts/fsgp/dynamic/wind_speed_10m.csv"))                        \
    PARAM(precomputed_distances_path,    std::filesystem::path,   std::filesystem::path("data/luts/fsgp/static/precomputed_distances.csv"))                  \
    PARAM(results_dir,                   std::filesystem::path,   std::filesystem::path("Results"))                  \
    PARAM(use_ghi,                       bool,                    false)                                                                                   \
    PARAM(calculate_distances,           bool,                    false)                                                                                   \
    PARAM(overnight_charging_location,   util::type::Coord,       util::type::Coord(37.000823, -86.367676, 0.157135))                                      \
    PARAM(is_route_track,                bool,                    true)                                                                                    \
    PARAM(current_soc,                   double,                  5.2)                                                                                     \
    PARAM(gps_coordinates,               util::type::Coord,       util::type::Coord())                                                                     \
    PARAM(utc_adjustment,                double,                  6.0)                                                                                     \
    PARAM(model,                         std::string,             "Gen 12")                                                                                \
    PARAM(optimizer,                     std::string,             "Acceleration")                                                                          \
    PARAM(min_speed,                     int,                  10)                                                                                      \
    PARAM(max_speed,                     int,                  80)                                                                                      \
    PARAM(save_csv,                      bool,                    true)                                                                                    \
    PARAM(simulator,                     std::string,             "FSGP")                                                                                  \
    PARAM(threads,                       double,                  0.5)                                                                                     \
    PARAM(acceleration_power_budget,     double,                  0.7)                                                                                     \
    PARAM(min_acceleration,              double,                  0.1)                                                                                     \
    PARAM(average_speed,                 double,                  45.0)                                                                                    \
    PARAM(corner_speed_min,              double,                  0.2)                                                                                     \
    PARAM(corner_speed_max,              double,                  0.9)                                                                                     \
    PARAM(aggressive_straight_threshold, double,                  200.0)                                                                                   \
    PARAM(num_repetitions,               int,                     5)                                                                                       \
    PARAM(log_optimization,              bool,                    false)                                                                                   \
    PARAM(print_population,              bool,                    false)                                                                                   \
    PARAM(fix_seeds,                     bool,                    false)                                                                                   \
    PARAM(loop_seed,                     unsigned int,            1)                                                                                       \
    PARAM(speed_seed,                    unsigned int,            1)                                                                                       \
    PARAM(aggressive_seed,               unsigned int,            1)                                                                                       \
    PARAM(acceleration_seed,             unsigned int,            1)                                                                                       \
    PARAM(idx_seed,                      unsigned int,            1)                                                                                       \
    PARAM(skip_seed,                     unsigned int,            1)                                                                                       \
    PARAM(num_generations,               int,                     100)                                                                                     \
    PARAM(population_size,               int,                     1000000)                                                                                 \
    PARAM(survival_percentage,           double,                  40)                                                                                      \
    PARAM(crossover_percentage,          double,                  30)                                                                                      \
    PARAM(mutation_percentage,           double,                  50)                                                                                      \
    PARAM(mutation_strategy,             std::string,             "PreferConstantSpeed")                                                                   \
    PARAM(dump_dir,                      std::filesystem::path,   std::filesystem::path("exports"))                                                        \
    PARAM(crossover_strategy,            std::string,             "LoopCross")                                                                             \
    PARAM(max_braking_force,             double,                  5000)                                                                                    \
    PARAM(day_one_start_time,            util::type::Time,        util::type::Time("2024-08-16T10:00:00+00:00"))                                           \
    PARAM(day_one_end_time,              util::type::Time,        util::type::Time("2024-08-16T18:00:00+00:00"))                                           \
    PARAM(day_start_time,                util::type::Time,        util::type::Time("2024-08-17T09:00:00+00:00"))                                           \
    PARAM(day_end_time,                  util::type::Time,        util::type::Time("2024-08-17T17:00:00+00:00"))                                           \
    PARAM(race_end_time,                 util::type::Time,        util::type::Time("2024-08-26T17:00:00+00:00"))                                           \
    PARAM(impounding_release_time,       util::type::Time,        util::type::Time("2024-08-26T07:00:00+00:00"))                                           \
    PARAM(impounding_start_time,         util::type::Time,        util::type::Time("2024-08-26T07:20:00+00:00"))                                           \
    PARAM(current_date_time,             util::type::Time,        util::type::Time("2024-08-26T07:20:00+00:00"))                                           \
    PARAM(checkpoint_hold_time,           int,                     30)        

/** Parse the config .yaml file
* the ::load_config(file) method should be called immediately after construction
*/
class ConfigParser {
 private:
  /* Map with all parsed key-value pairs from the yaml config file */
  std::unordered_map<std::string, YAML::Node> key_values;

  /* Store the absolute path to the configuration file */
  std::filesystem::path config_file_path;

  /* Store the path to gen12_strategy/RaceSim */
  char* STRAT_ROOT;

  const int MAX_RECURSION_DEPTH = 10;

  /* Declare all parameters */
  #define PARAM(name, type, default_value) \
    ConfigParam<type> name;

  CONFIG_PARAMETERS

  #undef PARAM

  /* Extract all values from the config */
  void get_config_leaf_nodes(YAML::Node node, int current_depth = 0);

 public:
  /* Parse the config file */
  ConfigParser(std::filesystem::path file_path);
  ConfigParser() = delete;

  /* Getters */
  #define PARAM(name, type, default_value) \
    inline ParamReturnType<type> get_##name() { return name.get_value(); }

  CONFIG_PARAMETERS
  #undef PARAM

  inline std::string get_strat_root() { return STRAT_ROOT; }

  /* No setters since config parameters should never change after initialization */
};
