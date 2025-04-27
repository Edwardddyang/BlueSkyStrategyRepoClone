/** Optimizer that searches for the best acceleration profile using a genetic algorithm */
#pragma once

#include <memory>
#include <vector>

#include "opt/Optimizer.hpp"
#include "route/Route.hpp"
#include "sim/Simulator.hpp"

class V2Optimizer : public Optimizer {
 private:
  // Race plan population and their results
  std::vector<RacePlan> population;
  std::vector<std::shared_ptr<ResultsLut>> result_luts;

  // Genetic algorithm parameters
  const int initial_population_size;
  const int num_generations;
  const double survival_percentage;
  const bool fix_num_loops;
  const double parents_percentage;
  unsigned int gen_seed;

  // Creation overhead tracking
  std::vector<double> race_plan_creation;

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

  /** @brief Crossover best parents of the population */
  void crossover_population();

  /** @brief Crossover two race plans */
  RacePlan crossover_parents(RacePlan parent_a, RacePlan parent_b);

 public:
  V2Optimizer(std::shared_ptr<Simulator> simulator, std::shared_ptr<Route> route);
  RacePlan optimize() override;
};
