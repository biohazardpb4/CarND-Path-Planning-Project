#include "cost.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"

using std::string;
using std::vector;

const float EFFICIENCY = 5.0;

float inefficiency_cost(const Vehicle &ego, 
                        const vector<Vehicle> &trajectory, 
                        const map<int, vector<Vehicle>> &predictions) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than ego's target speed.
  float proposed_speed_final = lane_speed(ego, predictions, trajectory[1].lane);
  std::cout << "lane " << trajectory[1].lane << " speed: " << proposed_speed_final << std::endl;
  float cost = (2.0*ego.target_speed - proposed_speed_final)/ego.target_speed;

  return cost;
}

float lane_speed(const Vehicle &ego, const map<int, vector<Vehicle>> &predictions, const int lane) {
  // Find the speed of the next car ahead of us in the lane.
  double min_same_lane_distance = 1000; // large number
  double speed = 1000; // large number
  for (const auto& kv : predictions) {
	if (kv.second[0].lane == lane) {
		double d = kv.second[0].s - ego.s;
		if (d > 0 && d < min_same_lane_distance) {
			min_same_lane_distance = d;
			speed = kv.second[0].v;
		}
	}
  }
  return speed;
}

float calculate_cost(const Vehicle &ego, 
                     const map<int, vector<Vehicle>> &predictions, 
                     const vector<Vehicle> &trajectory) {
  float cost = 0.0;

  // Add additional cost functions here.
  vector<std::function<float(const Vehicle &, const vector<Vehicle> &, 
                             const map<int, vector<Vehicle>> &)
    >> cf_list = {inefficiency_cost};
  vector<float> weight_list = {EFFICIENCY};
    
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i]*cf_list[i](ego, trajectory, predictions);
    cost += new_cost;
  }

  return cost;
}

