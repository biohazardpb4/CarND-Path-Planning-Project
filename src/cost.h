#ifndef COST_H
#define COST_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

float calculate_cost(const Vehicle &ego, 
                     const map<int, vector<Vehicle>> &predictions, 
                     const vector<Vehicle> &trajectory);

float inefficiency_cost(const Vehicle &ego, 
                        const vector<Vehicle> &trajectory, 
                        const map<int, vector<Vehicle>> &predictions);

float max_accel_cost(const Vehicle &ego,
		     const vector<Vehicle> &trajectory,
		     const map<int, vector<Vehicle>> &predictions);

float max_jerk_cost(const Vehicle &ego,
		const vector<Vehicle> &trajectory,
		const map<int, vector<Vehicle>> &predictions);


float collision_cost(const Vehicle &ego,
		     const vector<Vehicle> &trajectory,
		     const map<int, vector<Vehicle>> &predictions);

float lane_speed(const Vehicle &ego,
	         const map<int, vector<Vehicle>> &predictions,
		 const int lane);

float nearest_vehicle(
	const vector<Vehicle> &trajectory,
	const map<int, vector<Vehicle>> &predictions);

#endif  // COST_H
