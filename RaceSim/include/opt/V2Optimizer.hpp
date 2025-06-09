/** Optimizer that searches for the best acceleration profile using a genetic algorithm */
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "opt/Optimizer.hpp"
#include "route/Route.hpp"
#include "sim/Simulator.hpp"
#include "opt/PopulationGenerator.hpp"

enum class MutationStrategy {
  PreferConstantSpeed = 0
};

class V2Optimizer : public Optimizer {
 private:
  // Race plan population and their results
  std::vector<RacePlan> population;
  std::vector<RacePlan> new_population;
  std::vector<std::shared_ptr<ResultsLut>> result_luts;

  // Initial population generator
  std::shared_ptr<RacePlanCreator> generator;

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

  // Log optimization
  bool log_optimization;

  // Mutation logger
  FileLogger mutation_logger;

  // These parameters are required for mutating RacePlans and crossing over parents
  const double max_motor_power;
  const double car_mass;  // kg
  const double acceleration_power_budget;  // Units of Watts, must be < max_motor_power
  const double max_acceleration;
  const double max_deceleration;
  BasicLut route_distances;

  // Thread management
  ThreadManager thread_manager;
  const unsigned int num_threads;
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
  /** @brief Crossover best parents of the population */
  void crossover_population();

  /** @brief Crossover two race plans */
  RacePlan crossover_parents(RacePlan parent_a, RacePlan parent_b);

  ////////////////////////////////////////////////////////////
  ///////////////////////// Mutation /////////////////////////
  ////////////////////////////////////////////////////////////
  /** @brief Sample and mutate members of the population */
  void mutate_population();

  /** @brief Mutate a race plan according to some strategy chosen from configuration */
  void mutate_plan(RacePlan* plan);

  enum MutationStrategy {
    ConstantForDeceleration,
  };

  /** @brief Perform constant speed mutation by replacing a deceleration segment with constant speed */
  RacePlan constant_for_deceleration(RacePlan* plan, RacePlanCreator::Gen* gen);

  /** @brief Attempt to legalize a loop starting from some segment. Modification will be done in place
   *
   * @param plan The raw loop plan
   * @param loop_idx Loop index to start examination
   * @param seg_idx Segment index to start examination 
   *
   * Note: This assumes that there is some index/speed discontinuity e.g.
   * Loop Indices: [0,5],[5,14],[16,26] -> Discontinuity going from segment 1 to segment 2
   * Loop Speeds: [14,14],[14,15],[13,13] -> Discontinuity going from segment 1 to segment 2
   * Discontinuities are rectified by attemping to propagate "truth" from replacement_idx
   *
   * @return True if legalization was successful, false if unsuccessful
  */
  bool legalize_loop(RacePlan::PlanData& plan,
                     size_t loop_idx,
                     size_t seg_idx,
                     FileLogger* logger = nullptr);

 public:
  V2Optimizer(std::shared_ptr<Simulator> simulator, std::shared_ptr<Route> route);
  RacePlan optimize() override;
};
