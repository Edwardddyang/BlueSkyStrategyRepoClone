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

  // These parameters are required for mutating RacePlans and crossing over parents
  const double max_motor_power;
  const double acceleration_power_budget;
  const double max_acceleration;
  const double max_deceleration;

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
  void constant_for_deceleration(RacePlan* plan, RacePlanCreator::Gen* gen);

 public:
  V2Optimizer(std::shared_ptr<Simulator> simulator, std::shared_ptr<Route> route);
  RacePlan optimize() override;
};
