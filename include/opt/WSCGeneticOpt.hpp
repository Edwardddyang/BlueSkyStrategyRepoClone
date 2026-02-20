#pragma once 

#include <algorithm>
#include <memory>
#include <random> 
#include <filesystem>
#include <thread>
#include <utility>
#include <vector>

#include "SimUtils/Defines.hpp"
#include "SimUtils/Luts.hpp"
#include "config/ConfigParser.hpp"

#include "opt/Optimizer.hpp"
#include "route/RacePlan.hpp"
#include "route/Route.hpp"
#include "sim/WSCSimulator.hpp"

struct WSCOptimizerParams {
  unsigned int max_num_threads;
  int min_speed;                      // Minimum speed to consider in m/s
  int max_speed;                      // Maximum speed to consider in m/s
  bool save_csv;                      // Whether to save the metric log
  std::filesystem::path results_dir;  // Directory to store metric logs

  size_t population_size = 100;
  size_t num_generations = 100; 
  size_t survival_num = 10;
  size_t crossover_num = 60;
  size_t mutation_num = 30;
};

class WSCGeneticOpt : public Optimizer<WSCGeneticOpt, WSCRoute, WSCRacePlan, WSCSimulator> {
    public: 
        //Uses WSCOptimizerParams because FSGP uses global parameters which this does not 
        WSCGeneticOpt(WSCOptimizerParams params, WSCSimulator simulator, WSCRoute route);

    private: 
        //Main execution. Returns most optimal viable race strategy 
        WSCRacePlan optimize_impl();

        // Configuration
        WSCOptimizerParams params
        
        // Concurrency & Data Storage
        std::vector<std::thread> threads;
        std::vector<WSCRacePlan> population;
        std::vector<std::shared_ptr<ResultsLut>> result_luts;
        
        // Random Number Generator
        std::mt19937 gen;
        
        // GA Pipeline Functions
        void init_params();
        void create_initial_population();
        void simulate_population();
        void evaluate_population();
        void crossover_population();
        void mutate_population();
        
        // Genetic Operators
        WSCRacePlan crossover_parents(const WSCRacePlan& parent_a, const WSCRacePlan& parent_b);
        void mutate_constant_noise(WSCRacePlan* plan);
        void mutate_plan(WSCRacePlan* plan);
        
        // Utility for building smooth, contiguous initial arrays
        std::vector<BaseSegment> create_baseline_segments(double target_speed);
}
