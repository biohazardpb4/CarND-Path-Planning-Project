#include "cost.h"
#include <iostream>
#include <math.h>
#include <functional>
#include <iostream>
#include <sstream>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"

using std::string;
using std::vector;

// Nice to have
const float LANE_CENTER = 0.1;
const float EFFICIENCY = 1.4;
const float SLOW = 0.25;
const float SHORT = 0.3;

// Need to have
const float MAX_JERK = 0.9;
const float MAX_ACCELERATION = 0.85;
const float MAX_VELOCITY = 0.8;
const float STAY_ON_ROAD = 0.95;

// Critical
const float COLLISION = 10.0;

float off_lane_center_cost(const Vehicle &ego, 
                        Trajectory<Vehicle> &trajectory, 
                        const map<int, Trajectory<Vehicle>> &predictions) {
  // Cost becomes higher for trajectories which stray farther away from the center.
  double d = 0;
  for (const auto& step : trajectory.path) {
    d += distance(step.d, 0, step.lane()*4.0+2.0, 0);
  }

  return 1.0 - exp(-d/50.0);
}


float inefficiency_cost(const Vehicle &ego, 
                        Trajectory<Vehicle> &trajectory, 
                        const map<int, Trajectory<Vehicle>> &predictions) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than ego's target speed.
  float proposed_speed_final = lane_speed(ego, predictions, trajectory.path[trajectory.path.size()-1].lane());
  float diff = ego.target_speed - proposed_speed_final;

  return 1.0 - exp(-diff/10.0);
}

float slow_cost(const Vehicle &ego,
		Trajectory<Vehicle> &trajectory,
		const map<int, Trajectory<Vehicle>> &predictions) {
      // Get the speed at the 2s mark (typical planning horizon)
  float speed = trajectory.path[trajectory.path.size()-1].vs;
  int horizon_index = int(2.0 / 0.02)-1;
  if (trajectory.path.size() > horizon_index) {
    speed = trajectory.path[horizon_index].vs;
  }
  // Consider all paths which go over target speed as 0 cost
  if (speed > ego.target_speed) {
    return 0;
  }
  return (ego.target_speed - speed)/ego.target_speed;
}

float short_cost(const Vehicle &ego,
		Trajectory<Vehicle> &trajectory,
		const map<int, Trajectory<Vehicle>> &predictions) {
  double max_s = 6945.554;
      // Get the s at the 2s mark (typical planning horizon)
  float s = trajectory.path[trajectory.path.size()-1].vs;
  int horizon_index = int(2.0 / 0.02)-1;
  if (trajectory.path.size() > horizon_index) {
    s = trajectory.path[horizon_index].s;
  }
  float s_distance = s - ego.s;
  return exp(-s_distance);
}

float max_jerk_cost(const Vehicle &ego, 
                        Trajectory<Vehicle> &trajectory, 
                        const map<int, Trajectory<Vehicle>> &predictions) {
  // Cost becomes higher for trajectories with > 10 m/s^3 jerk
  if (trajectory.path.size() < 2) {
    return 0;
  }
  for (int i = 0; i < trajectory.path.size()-1; i++) {
    double jerk = distance(trajectory.path[i].as, trajectory.path[i].ad,
      trajectory.path[i+1].as, trajectory.path[i+1].ad);
    if (jerk > 10) {
      return 1;
    }
  }
  return 0;
}

float max_accel_cost(const Vehicle &ego, 
                        Trajectory<Vehicle> &trajectory, 
                        const map<int, Trajectory<Vehicle>> &predictions) {
  // Cost becomes higher for trajectories with > 10 m/s^2 acceleration
  for (const auto& step : trajectory.path) {
    if (distance(0, 0, step.ad, step.as) > 10) {
      return 1;
    }
  }
  return 0;
}

float max_velocity_cost(const Vehicle &ego, 
                        Trajectory<Vehicle> &trajectory, 
                        const map<int, Trajectory<Vehicle>> &predictions) {
  // Cost becomes higher for trajectories with > 50 mph velocity
  for (const auto& step : trajectory.path) {
    if (distance(0, 0, step.vd, step.vs) > step.target_speed) {
      return 1;
    }
  }
  return 0;
}

float collision_cost(const Vehicle &ego, 
                        Trajectory<Vehicle> &trajectory, 
                        const map<int, Trajectory<Vehicle>> &predictions) {
  // Cost becomes higher when collisions occur
  float nearest = nearest_vehicle(trajectory, predictions);
  // std::cout << "nearest vehicle: " << nearest << std::endl;
  return nearest < 3 ? 1 : 0;
}

float lane_speed(const Vehicle &ego, const map<int, Trajectory<Vehicle>> &predictions, const int lane) {
  // Find the speed of the next car ahead of us in the lane.
  const float HORIZON = 50; // Only consider cars within 50 meters.
  double min_same_lane_distance = 1000; // large number
  double speed = ego.target_speed;
  for (const auto& kv : predictions) {
    if (kv.second.path[0].lane() == lane) {
      double d = kv.second.path[0].s - ego.s;
      if (d > 0 && d < min_same_lane_distance && d < HORIZON) {
        min_same_lane_distance = d;
        speed = kv.second.path[0].vs;
      }
    }
  }
  return std::min(speed, ego.target_speed*1.1);
}

float stay_on_road_cost(const Vehicle &ego,
		     Trajectory<Vehicle> &trajectory,
		     const map<int, Trajectory<Vehicle>> &predictions) {
  // Cost becomes higher if the car goes off of the road
  for (const auto& step : trajectory.path) {
    if (step.d < 1.5 || step.d > step.lanes_available*4-1.5) {
      return 1;
    }
  }
  return 0;
}

float calculate_cost(const Vehicle &ego, 
                     const map<int, Trajectory<Vehicle>> &predictions, 
                     Trajectory<Vehicle> &trajectory) {
  float cost = 0.0;

  // Add additional cost functions here.
  vector<string> label_list{"inefficiency", "slow", "short", "max_jerk", "max_accel", "max_velocity",
    "collision", "stay_on_road", "off_lane_center"};
  vector<std::function<float(const Vehicle &, Trajectory<Vehicle> &, 
    const map<int, Trajectory<Vehicle>> &)>> cf_list =
    {inefficiency_cost, slow_cost, short_cost, max_jerk_cost, max_accel_cost, max_velocity_cost,
    collision_cost, stay_on_road_cost, off_lane_center_cost};
  vector<float> weight_list = {EFFICIENCY, SLOW, SHORT, MAX_JERK, MAX_ACCELERATION, MAX_VELOCITY,
    COLLISION, STAY_ON_ROAD, LANE_CENTER};
    
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i]*cf_list[i](ego, trajectory, predictions);
    trajectory.individual_costs[label_list[i]] = new_cost;
    cost += new_cost;
  }
  trajectory.total_cost = cost;

  return cost;
}

float nearest_vehicle(Trajectory<Vehicle> &trajectory, const map<int, Trajectory<Vehicle>> &predictions) {
  float nearest = 1000000; // big number
  int nearest_i = -1;
  Vehicle nearest_vehicle;
  double DT = 0.02; // WARNING -- THIS MUST LINE UP WITH WHAT WAS USED TO GENERATE THE TRAJECTORY!
  double t = 0;
  for (const auto& ego : trajectory.path) {
    for (auto& kv : predictions) {
      auto other = kv.second.path[0];
      // Assume a bit of negative acceleration to protect against slow downs
      other.as = -4.0;
      const Vehicle& predicted = other.at(t); // Assume constant acceleration of other vehicles.
      float d = distance(ego.s, ego.d, predicted.s, predicted.d);
      if (d < nearest) {
        nearest = d;
        nearest_i = kv.first;
        nearest_vehicle = predicted;
      }
    }
    t += DT;
  }

  std::ostringstream debug_info;
  debug_info << "nearest(car:" << nearest_i << " s<" << nearest_vehicle.s << "," <<
    nearest_vehicle.vs << "," << nearest_vehicle.as << "> d<" << nearest_vehicle.d <<
    "," << nearest_vehicle.vd << "," << nearest_vehicle.ad << ">): " << nearest;
  trajectory.debug = debug_info.str();

  return nearest;
}
