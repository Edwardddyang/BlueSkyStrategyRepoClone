/* Singleton class that contains all parameters for a full scale session */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <filesystem>

#include "utils/Units.hpp"
#include "utils/Luts.hpp"
#include "config/ConfigParam.hpp"
#include "utils/Defines.hpp"
#include <fstream>
#include <sstream>


/* Define all config parameters
   PARAM(<parameter name as it appears exactly in the .yaml configuration file>,
   		<data type>,
		<return type to accessors>,
		<default value>)

	If the parameter you're introducing is a pointer of some type, please use unique_ptr
	as the <data type> to avoid memory leaks. You'll also have to make relevant changes in
	config_param.h. Note that for all variables of type std::filesystem::path, they will be
  pre-pended by STRAT_ROOT
 */
#define CONFIG_PARAMETERS                                                                   \
  PARAM(max_soc, double, double, 5.2)                                                       \
  PARAM(tire_pressure, double, double, 5.5)                                                 \
  PARAM(array_area, double, double, 3.98)                                                   \
  PARAM(car_mass, double, double, 305)                                                      \
  PARAM(air_density, double, double, 1.17)                                                  \
  PARAM(passive_electric_loss, double, double, 20)                                          \
  PARAM(cda, double, double, 0.16)                                                          \
  PARAM(max_motor_power, double, double, 5.0)                                               \
  PARAM(max_car_speed, double, double, 104)                                                 \
  PARAM(max_route_speed, double, double, 104)                                               \
  PARAM(max_acceleration, double, double, 2.0)                                              \
  PARAM(max_deceleration, double, double, 3.0)                                              \
  PARAM(array_power_max, double, double, 950)                                               \
  PARAM(array_efficiency, double, double, 0.252)                                            \
  PARAM(motor_efficiency, double, double, 0.9)                                              \
  PARAM(regen_efficiency, double, double, 0.8)                                              \
  PARAM(battery_efficiency, double, double, 0.95)                                           \
  PARAM(tire_inertia, double, double, 0.00125)                                              \
  PARAM(tire_radius, double, double, 0.508)                                                 \
  PARAM(num_tires, int, int, 3)                                                             \
  PARAM(max_num_loops, int, int, 300)                                                       \
  PARAM(fix_num_loops, bool, bool, true)                                                    \
  PARAM(control_stops, std::unordered_set<size_t>,                                          \
        std::unordered_set<size_t>,                                                         \
        convert_string_to_int_set("2962,5559,9462,11421,14439,16990,20832,23202,25987"))    \
  PARAM(init_control_stops, bool, bool, false)                                              \
  PARAM(control_stop_charge_time, int, int, 30)                                             \
  PARAM(base_route_path, std::filesystem::path, std::filesystem::path,                      \
        std::filesystem::path("data/luts/fsgp/static/fsgp_base_route.csv"))                 \
  PARAM(telemetry_path, std::filesystem::path, std::filesystem::path,                       \
        std::filesystem::path("data/luts/telemetry/sim.csv"))                               \
  PARAM(corners_path, std::filesystem::path, std::filesystem::path,                         \
        std::filesystem::path("data/luts/fsgp/static/fsgp_corners.csv"))                    \
  PARAM(power_factor_path, std::filesystem::path, std::filesystem::path,                    \
        std::filesystem::path("data/luts/fsgp/static/powerfactor.csv"))                     \
  PARAM(roll_res_slope_path, std::filesystem::path, std::filesystem::path,                  \
        std::filesystem::path("data/luts/fsgp/static/rr2.csv"))                             \
  PARAM(roll_res_yint_path, std::filesystem::path, std::filesystem::path,                   \
        std::filesystem::path("data/luts/fsgp/static/rr1.csv"))                             \
  PARAM(dni_path, std::filesystem::path, std::filesystem::path,                             \
        std::filesystem::path("data/luts/fsgp/dynamic/dni.csv"))                            \
  PARAM(dhi_path, std::filesystem::path, std::filesystem::path,                             \
        std::filesystem::path("data/luts/fsgp/dynamic/dhi.csv"))                            \
  PARAM(wind_direction_path, std::filesystem::path, std::filesystem::path,                  \
        std::filesystem::path("data/luts/fsgp/dynamic/wind_direction_10m.csv"))             \
  PARAM(wind_speed_path, std::filesystem::path, std::filesystem::path,                      \
        std::filesystem::path("data/luts/fsgp/dynamic/wind_speed_10m.csv"))                 \
  PARAM(precomputed_distances_path, std::filesystem::path, std::filesystem::path,           \
        std::filesystem::path("data/luts/fsgp/static/precomputed_distances.csv"))           \
  PARAM(calculate_distances, bool, bool, false)                                             \
  PARAM(day_one_start_time, std::unique_ptr<Time>, Time,                                    \
        std::make_unique<Time>("2024-08-16 10:00:00", 6.0))                                 \
  PARAM(day_one_end_time, std::unique_ptr<Time>, Time,                                      \
        std::make_unique<Time>("2024-08-16 18:00:00", 6.0))                                 \
  PARAM(day_start_time, std::unique_ptr<Time>, Time, std::make_unique<Time>("09:00:00"))    \
  PARAM(day_end_time, std::unique_ptr<Time>, Time, std::make_unique<Time>("17:00:00"))      \
  PARAM(race_end_time, std::unique_ptr<Time>, Time,                                         \
        std::make_unique<Time>("2024-08-18 17:00:00", 6.0))                                 \
  PARAM(impounding_release_time, std::unique_ptr<Time>, Time,                               \
        std::make_unique<Time>("07:00:00"))                                                 \
  PARAM(impounding_start_time, std::unique_ptr<Time>, Time,                                 \
        std::make_unique<Time>("20:00:00"))                                                 \
  PARAM(overnight_charging_location, Coord, Coord, Coord(37.000823, -86.367676, 0.157135))  \
  PARAM(is_route_track, bool, bool, true)                                                   \
  PARAM(current_soc, double, double, 5.2)                                                   \
  PARAM(gps_coordinates, Coord, Coord, Coord())                                             \
  PARAM(current_date_time, std::unique_ptr<Time>, Time,                                     \
        std::make_unique<Time>("2023-10-28 17:00:00", 6.0))                                 \
  PARAM(utc_adjustment, double, double, 6.0)                                                \
  PARAM(model, std::string, std::string, "Gen 12")                                          \
  PARAM(optimizer, std::string, std::string, "Acceleration")                                \
  PARAM(min_speed, double, double, 10)                                                      \
  PARAM(max_speed, double, double, 80)                                                      \
  PARAM(save_csv, bool, bool, true)                                                         \
  PARAM(simulator, std::string, std::string, "FSGP")                                        \
  PARAM(threads, double, double, 0.5)                                                       \
  PARAM(acceleration_power_budget, double, double, 0.7)                                     \
  PARAM(min_acceleration, double, double, 0.1)                                              \
  PARAM(average_speed, double, double, 45.0)                                                \
  PARAM(corner_speed_min, double, double, 0.2)                                              \
  PARAM(corner_speed_max, double, double, 0.9)                                              \
  PARAM(aggressive_straight_threshold, double, double, 200.0)                               \
  PARAM(num_repetitions, int, int, 5)                                                       \
  PARAM(log_optimization, bool, bool, false)                                                \
  PARAM(print_population, bool, bool, false)                                                \
  PARAM(fix_seeds, bool, bool, false)                                                       \
  PARAM(loop_seed, unsigned int, unsigned int, 1)                                           \
  PARAM(speed_seed, unsigned int, unsigned int, 1)                                          \
  PARAM(aggressive_seed, unsigned int, unsigned int, 1)                                     \
  PARAM(acceleration_seed, unsigned int, unsigned int, 1)                                   \
  PARAM(idx_seed, unsigned int, unsigned int, 1)                                            \
  PARAM(skip_seed, unsigned int, unsigned int, 1)                                           \
  PARAM(num_generations, int, int, 100)                                                     \
  PARAM(population_size, int, int, 1000000)                                                 \
  PARAM(survival_percentage, double, double, 40)                                            \
  PARAM(crossover_percentage, double, double, 30)                                           \
  PARAM(mutation_percentage, double, double, 50)                                            \
  PARAM(mutation_strategy, std::string, std::string, "PreferConstantSpeed")                 \


/* Class that holds all information from a .yaml file storing configuration parameters for
 * a race simulation
*/
class Config {
 private:
  /* Parsed config */
  static YAML::Node config;

  /* Map with all parsed key-value pairs from the yaml config file */
  static std::unordered_map<std::string, YAML::Node> key_values;

  /* Singleton pointer */
  static std::unique_ptr<Config> instance_ptr;

  /* Whether STRAT_ROOT and config_file_path have been initialized */
  static bool initialized;

  /* Store the absolute path to the configuration file */
  static std::filesystem::path config_file_path;

  /* Store the path to gen12_strategy/RaceSim */
  static char* STRAT_ROOT;

  static const int MAX_RECURSION_DEPTH = 10;

  std::vector<Coord> telem_coords;
  std::vector<Time> telem_times;


  // Dummy variable to satisfy macro syntax
  int b;

  /* Declare all parameters */
  #define PARAM(name, type, return_type, default_value) \
    Config_Param<type, return_type> name;

  CONFIG_PARAMETERS

  #undef PARAM

  /* Initialize all parameters */
  #define PARAM(name, type, return_type, default_value)\
    name(Config_Param<type, return_type>(#name, default_value, key_values, STRAT_ROOT)),

  /* Load all parameters from yaml file. Should only be called from get_instance() */
  Config() : CONFIG_PARAMETERS b(45) {}

  #undef PARAM

  /* Extract all values from the config */
  static void get_config_leaf_nodes(YAML::Node node, int current_depth = 0);

 public:
  /* Public singleton constructor */
  static Config* get_instance();

  /** Set the config file. Can only be called once in an executable's life cycle
  * @param file_path path to the config file relative to executable
  * @param strat_root_path full path to gen12_strategy/RaceSim
  */
  static void initialize(std::filesystem::path file_path, char* strat_root_path);

  /* Getters */
  #define PARAM(name, type, return_type, default_value) \
    inline return_type get_##name() { return name.get_value(); }

  CONFIG_PARAMETERS
  #undef PARAM

  static inline std::string get_strat_root() { return STRAT_ROOT; }

  /* No setters since config parameters should never change after initialization */
};
