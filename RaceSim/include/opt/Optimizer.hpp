/* 
Base class for optimization algorithms
*/

#pragma once

#include "Car.hpp"
#include "Route.hpp"
#include "Simulator.hpp"

class Optimizer {
protected:
    /* Assess viability of proposed speed profiles */
    std::unique_ptr<Simulator> simulator;

    /* Route to optimize on */
    std::unique_ptr<Route> route;

    /* Output speed profile */
    std::vector<double> speed_profile_kph;
public:
    Optimizer(std::unique_ptr<Simulator> sim, std::unique_ptr<Route> path);

    /* Apply the optimization algorithm, output a speed profile over
       the segments of the race
    */
    virtual std::vector<double> optimize() = 0;
};
