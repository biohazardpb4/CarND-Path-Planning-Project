#ifndef COST_H
#define COST_H

#include "vehicle.h"
#include "trajectory.h"
#include <string>

using std::map;
using std::string;
using std::vector;

float calculate_cost(const Vehicle &ego, 
                     const map<int, Trajectory<Vehicle>> &predictions, 
                     Trajectory<Vehicle> &trajectory);

float off_lane_center_cost(const Vehicle &ego, 
                        const Trajectory<Vehicle> &trajectory, 
                        const map<int, Trajectory<Vehicle>> &predictions);
						
float inefficiency_cost(const Vehicle &ego, 
                        const Trajectory<Vehicle> &trajectory, 
                        const map<int, Trajectory<Vehicle>> &predictions);

float max_jerk_cost(const Vehicle &ego,
		const Trajectory<Vehicle> &trajectory,
		const map<int, Trajectory<Vehicle>> &predictions);

float max_accel_cost(const Vehicle &ego,
		     const Trajectory<Vehicle> &trajectory,
		     const map<int, Trajectory<Vehicle>> &predictions);

float max_velocity_cost(const Vehicle &ego,
		     const Trajectory<Vehicle> &trajectory,
		     const map<int, Trajectory<Vehicle>> &predictions);

float collision_cost(const Vehicle &ego,
		     const Trajectory<Vehicle> &trajectory,
		     const map<int, Trajectory<Vehicle>> &predictions);

float stay_on_road_cost(const Vehicle &ego,
		     const Trajectory<Vehicle> &trajectory,
		     const map<int, Trajectory<Vehicle>> &predictions);

float lane_speed(const Vehicle &ego,
	         const map<int, Trajectory<Vehicle>> &predictions,
		 const int lane);

float nearest_vehicle(
	const Trajectory<Vehicle> &trajectory,
	const map<int, Trajectory<Vehicle>> &predictions);

#endif  // COST_H
