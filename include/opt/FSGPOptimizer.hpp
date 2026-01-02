/** Optimizer that searches for the best acceleration profile using a genetic algorithm */
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "opt/Optimizer.hpp"
#include "route/Route.hpp"
#include "sim/Simulator.hpp"
#include "opt/PopulationGenerator.hpp"

class FSGPOptimizer : public Optimizer<FSGPOptimizer, FSGPRoute, FSPRacePlan, FSGPSimulator> {
 private:
  // Race plan population and their results
  std::vector<FSGPRacePlan> population;
  std::vector<FSGPRacePlan> new_population;
  std::vector<Luts::DataSet> result_luts;

  // Initial population generator
  RacePlanCreator generator;
  // rng variables
  unsigned int idx_seed;
  unsigned int speed_seed;
  unsigned int acceleration_seed;
  unsigned int aggressive_seed;
  unsigned int loop_seed;
  unsigned int skip_seed;
  RacePlanCreator::Gen rng_collection;

  // Genetic algorithm parameters
  const int population_size;
  const int num_generations;
  const double survival_percentage;
  const double crossover_percentage;
  const double mutation_percentage;
  const std::string mutation_strategy;
  double survival_num;
  const bool fix_num_loops;
  double crossover_num;
  double mutation_num;
  unsigned int gen_seed;
  const int num_loops_in_block;

  // Population status
  double population_average_speed;
  int num_viable_plans;

  // Log optimization
  bool log_optimization;

  // Mutation logger
  FileLogger mutation_logger;

  // Crossover logger
  FileLogger crossover_logger;

  // These parameters are required for mutating RacePlans and crossing over parents
  const double max_motor_power;
  const double car_mass;  // kg
  const double acceleration_power_budget;  // Units of Watts, must be < max_motor_power
  const double max_acceleration;
  const double max_deceleration;
  PointsLut route_distances;

  // Thread management
  std::vector<std::thread> threads;

  /** @brief Create initial population */
  void create_initial_population();

  /** @brief Simulate the population */
  void simulate_population();

  /** @brief Evaluate population and gather fitness levels */
  void evaluate_population();

  /** @brief Initialize parameters and thread manager */
  void init_params();

  ////////////////////////////////////////////////////////////
  ///////////////////////// Crossover ////////////////////////
  ////////////////////////////////////////////////////////////
  std::unordered_set<std::string> CROSSOVER_STRATEGIES = {
    "LoopCross"
  };

  /** @brief Crossover best parents of the population */
  void crossover_population();

  /** @brief Perform crossover by using loops from parents */
  RacePlan loop_cross(RacePlan* parent_a, RacePlan* parent_b, RacePlanCreator::Gen* gen);

  /** @brief Crossover two race plans */
  RacePlan crossover_parents(RacePlan parent_a, RacePlan parent_b);

  ////////////////////////////////////////////////////////////
  ///////////////////////// Mutation /////////////////////////
  ////////////////////////////////////////////////////////////
  /** @brief Sample and mutate members of the population */
  void mutate_population();

  /** @brief Print summary of population status
   * @param generation Generation number. If provided, print banner
   */
  void print_population_status(int generation = -1);

  /** @brief Mutate a race plan according to some strategy chosen from configuration */
  void mutate_plan(RacePlan* plan);

  std::unordered_set<std::string> MUTATION_STRATEGIES = {
    "ConstantForDeceleration",
    "AccelerationNoise",
    "ConstantNoise",
    "Mix",
  };

  // All mutation functions below have the following signature:
  // @param plan: Plan to mutate
  // @param rng: Random number generator seeds
  // @param loop_idx: Loop to mutate. If -1, mutate all loops
  // Note: All functions operate on the "raw" loops prior to glueing

  /** @brief Replace deceleration segment with constant speed in a specific loop */
  void constant_for_deceleration(FSGPRacePlan* plan, RacePlanCreator::Gen* rng);
  /** @brief Mutate a loop of the plan using the method described above
   * @note new_raw_plan is modified in place 
  */
  void constant_for_deceleration_loop(FSGPRacePlan::PlanData* new_raw_plan,
                                      size_t loop_idx,
                                      RacePlanCreator::Gen* rng);

  /** @brief Add either +1 or -1 m/s to the end of an acceleration/deceleration segment */
  void acceleration_noise(FSGPRacePlan* plan, RacePlanCreator::Gen* rng);
  /** @brief Mutate a loop of the plan using the method described above
   * @note new_raw_plan is modified in place
   */
  void acceleration_noise_loop(FSGPRacePlan::PlanData* new_raw_plan, size_t loop_idx, RacePlanCreator::Gen* rng);

  /** @brief Add either +1 or -1 m/s to a constant speed */
  void constant_noise(FSGPRacePlan* plan, RacePlanCreator::Gen* rng);
  /** @brief Mutate a loop of the plan using the method described above
   * @note new_raw_plan is modified in place
   */
  void constant_noise_loop(FSGPRacePlan::PlanData* new_raw_plan, size_t loop_idx,
                           RacePlanCreator::Gen* rng);

  /** @brief Mutate each loop using one of the techniques above */
  void mix_mutation(RacePlan* plan, RacePlanCreator::Gen* rng);

  /** @brief Attempt to legalize a loop starting from some segment. Modification will be done in place
   *
   * @param plan The raw loop plan
   * @param loop_idx Loop index to start examination
   * @param seg_idx Segment index to start examination 
   * @note: plan is MODIFIED IN PLACE
   *
   * Note: This assumes that there is some index/speed discontinuity e.g.
   * Loop Indices: [0,5],[5,14],[16,26] -> Discontinuity going from segment 1 to segment 2
   * Loop Speeds: [14,14],[14,15],[13,13] -> Discontinuity going from segment 1 to segment 2
   * Discontinuities are rectified by attemping to propagate "truth" from replacement_idx
   *
   * @return True if legalization was successful, false if unsuccessful
  */
  bool legalize_loop(FSGPRacePlan::PlanData* plan,
                     size_t loop_idx,
                     size_t seg_idx,
                     FileLogger* logger = nullptr);

 public:
  FSGPOptimizer(Simulator simulator, FSGPRoute route);
  FSGPRacePlan optimize() override;
};
