#include "opt/WSCGeneticOpt.hpp"

#include <algorithm>
#include <filesystem>
#include <chrono>

#include "SimUtils/Constants.hpp"

void WSCGenericOpt::create_initial_population(){

    int index = 0;
    double speed_limit = this->route.get_max_route_speed(); 

    //20 race plans are made using constant speed 
    for(int i = 0; i < 20; i++){
        double speed = util::constants::kph2mps(60 + (i * 2));
        population[index++] = WSCRacePlan(create_baseline_segments(speed));
    }

    //Normal Distribution creates random distribution, (mean, standard deviation is 3) 
    std::normal_distribution<double> noise_dist(0.0, util::constants::kph2mps(3.0));
    for(int i = 0; i < 60; i++){
        //base_speed is cycles through (75 + 0-15) and the speed limit of the segment 
        double base_speed = std::min(util::constants::kph2mps(75 + (i % 15)), speed_limit);
        auto segments = create_baseline_segments(base_speed); //Create a route plan using a constant speed 
        
        //Loops through each segment of the base_speed 
        for(auto& seg : segments) {
            // Apply noise but clamp to legal route speed limits
            double offset = noise_dist(gen); //Get a number from the random noise 
            //Change the start and end speed of this segment 
            seg.start_speed = std::clamp(seg.start_speed + offset, 0.0, speed_limit); 
            seg.end_speed = std::clamp(seg.end_speed + offset, 0.0, speed_limit);
            seg.acceleration_value = (seg.end_speed * seg.end_speed - seg.start_speed * seg.start_speed) / (2 * seg.distance);
        }
        population[index++] = WSCRacePlan(segments);
    }

    //Mutation for final race plans. population size = 100 
    for(; index < params.population_size; index++) {
        population[index] = WSCRacePlan(create_baseline_segments(util::constants::kph2mps(80)));
        mutate_constant_noise(&population[index]); 
        mutate_constant_noise(&population[index]); 
    }
}

//ASK: Should I split the track into its control segments or into equal distance segments??? 

//BELOW IS 3 DIFFERENT IMPLEMENTATIONS OF THE CREATE_BASELINE_SEGMENTS
//Splits the route into 50 equal distance segments 
std::vector<BaseSegment> WSCGeneticOpt::create_baseline_segments(double target_speed) {
    std::vector<BaseSegment> segments;

    //Gets # of points for the WSC route 
    size_t total_points = this->route.get_num_points();
    
    // Break the 3020km route into ~50 logical simulation segments
    size_t chunk_size = total_points / 50; 
    
    for(size_t i = 0; i < total_points - chunk_size; i += chunk_size) {
        double dist = this->route.get_precomputed_distances().get_value(i, i + chunk_size);
        segments.emplace_back(i, i + chunk_size, target_speed, target_speed, 0.0, dist);
    }
    return segments;
}

//Seperate into control stops, and for each one 
std::vector<BaseSegment> WSCGeneticOpt::create_baseline_segments(double target_speed) {
    std::vector<BaseSegment> segments;
    
    //Get the control stops and sort them in order 
    //Control stops in sort vector 
    auto stops_set = this->route.get_control_stops();
    std::vector<size_t> stops(stops_set.begin(), stops_set.end());
    std::sort(stops.begin(), stops.end());
    
    // Ensure the Starting Line (Darwin) and Finish Line (Adelaide) are included
    if (stops.empty() || stops.front() != 0) {
        stops.insert(stops.begin(), 0);
    }
    if (stops.back() != this->route.get_num_points() - 1) {
        stops.push_back(this->route.get_num_points() - 1);
    }

    // 2. Define standard, safe acceleration rates for a solar car (m/s^2)
    const double standard_accel = 0.5;
    const double standard_decel = -0.5;
    
    // Calculate the distance (in meters) required to reach the target speed: d = v^2 / 2a
    double req_accel_dist = (target_speed * target_speed) / (2.0 * standard_accel);
    double req_decel_dist = -(target_speed * target_speed) / (2.0 * standard_decel);

    auto& distances = this->route.get_precomputed_distances();

    //Create a 3-part physical profile between every pair of towns
    for(size_t i = 0; i < stops.size() - 1; i++) {
        size_t start_idx = stops[i];
        size_t end_idx = stops[i + 1];
        
        // Find the exact map index where the car finishes accelerating
        size_t accel_end_idx = start_idx;
        while (accel_end_idx < end_idx && 
               distances.get_value(start_idx, accel_end_idx) < req_accel_dist) {
            accel_end_idx++;
        }

        // Find the exact map index where the car must start braking
        size_t decel_start_idx = end_idx;
        while (decel_start_idx > accel_end_idx && 
               distances.get_value(decel_start_idx, end_idx) < req_decel_dist) {
            decel_start_idx--;
        }

        // --- Build the 3 Segments ---
        
        // Segment A: Acceleration (0 -> target_speed)
        double actual_accel_dist = distances.get_value(start_idx, accel_end_idx);
        double actual_accel = (actual_accel_dist > 0) ? (target_speed * target_speed) / (2.0 * actual_accel_dist) : 0.0;
        if (actual_accel_dist > 0) {
            segments.emplace_back(start_idx, accel_end_idx, 0.0, target_speed, actual_accel, actual_accel_dist);
        }

        // Segment B: Highway Cruise (target_speed -> target_speed)
        double cruise_dist = distances.get_value(accel_end_idx, decel_start_idx);
        if (cruise_dist > 0) {
            segments.emplace_back(accel_end_idx, decel_start_idx, target_speed, target_speed, 0.0, cruise_dist);
        }

        // Segment C: Deceleration (target_speed -> 0)
        double actual_decel_dist = distances.get_value(decel_start_idx, end_idx);
        double actual_decel = (actual_decel_dist > 0) ? -(target_speed * target_speed) / (2.0 * actual_decel_dist) : 0.0;
        if (actual_decel_dist > 0) {
            segments.emplace_back(decel_start_idx, end_idx, target_speed, 0.0, actual_decel, actual_decel_dist);
        }
    }
    
    return segments;
}

//Seperate the race into control stops 
std::vector<BaseSegment> WSCGeneticOpt::create_baseline_segments(double target_speed) {
    std::vector<BaseSegment> segments;
    
    // Convert the unordered_set to a sorted vector of indices
    auto stops_set = this->route.get_control_stops(); // Assuming this getter exists
    std::vector<size_t> stops(stops_set.begin(), stops_set.end());
    std::sort(stops.begin(), stops.end());
    
    // Add the starting line (index 0) and finish line (get_num_points - 1)
    stops.insert(stops.begin(), 0);
    stops.push_back(this->route.get_num_points() - 1);
    
    // Create segments exactly between the control stops
    for(size_t i = 0; i < stops.size() - 1; i++) {
        double dist = this->route.get_precomputed_distances().get_value(stops[i], stops[i+1]);
        segments.emplace_back(stops[i], stops[i+1], target_speed, target_speed, 0.0, dist);
    }
    return segments;
}