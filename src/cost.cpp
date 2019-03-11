#include "cost.h"
#include <iostream>
#include <math.h>
#include <functional>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"

using std::string;
using std::vector;

const float LANE_CENTER = 0.1;
const float EFFICIENCY = 0.2;
const float MAX_JERK = 0.9;
const float MAX_ACCELERATION = 0.85;
const float MAX_VELOCITY = 0.8;
const float STAY_ON_ROAD = 0.95;
const float COLLISION = 1.0;

float off_lane_center_cost(const Vehicle &ego, 
                        const Trajectory<Vehicle> &trajectory, 
                        const map<int, Trajectory<Vehicle>> &predictions) {
  // Cost becomes higher for trajectories which stray farther away from the center.
  double d = 0;
  for (const auto& step : trajectory.path) {
    d += distance(step.d, 0, step.lane()*4.0-2.0, 0);
  }

  return 1.0 - exp(-d);
}


float inefficiency_cost(const Vehicle &ego, 
                        const Trajectory<Vehicle> &trajectory, 
                        const map<int, Trajectory<Vehicle>> &predictions) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than ego's target speed.
  float proposed_speed_final = lane_speed(ego, predictions, trajectory.path[trajectory.path.size()-1].lane());
  float cost = (2.0*ego.target_speed - proposed_speed_final)/ego.target_speed;

  return cost;
}

float max_jerk_cost(const Vehicle &ego, 
                        const Trajectory<Vehicle> &trajectory, 
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
                        const Trajectory<Vehicle> &trajectory, 
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
                        const Trajectory<Vehicle> &trajectory, 
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
                        const Trajectory<Vehicle> &trajectory, 
                        const map<int, Trajectory<Vehicle>> &predictions) {
  // Cost becomes higher when collisions occur
  float nearest = nearest_vehicle(trajectory, predictions);
  std::cout << "nearest vehicle: " << nearest << std::endl;
  return nearest < 4 ? 1 : 0;
}

float lane_speed(const Vehicle &ego, const map<int, Trajectory<Vehicle>> &predictions, const int lane) {
  // Find the speed of the next car ahead of us in the lane.
  const float HORIZON = 50; // Only consider cars within 50 meters.
  double min_same_lane_distance = 1000; // large number
  double speed = 1000; // large number
  for (const auto& kv : predictions) {
    if (kv.second.path[0].lane() == lane) {
      double d = kv.second.path[0].s - ego.s;
      if (d > 0 && d < min_same_lane_distance && d < HORIZON) {
        min_same_lane_distance = d;
        speed = kv.second.path[0].vs;
      }
    }
  }
  return speed;
}

float stay_on_road_cost(const Vehicle &ego,
		     const Trajectory<Vehicle> &trajectory,
		     const map<int, Trajectory<Vehicle>> &predictions) {
  // Cost becomes higher if the car goes off of the road
  for (const auto& step : trajectory.path) {
    if (step.d < 0 || step.d > step.lanes_available*4) {
      return 1;
    }
  }
  return 0;
}

float calculate_cost(const Vehicle &ego, 
                     const map<int, Trajectory<Vehicle>> &predictions, 
                     const Trajectory<Vehicle> &trajectory) {
  float cost = 0.0;

  // Add additional cost functions here.
  vector<std::function<float(const Vehicle &, const Trajectory<Vehicle> &, 
                             const map<int, Trajectory<Vehicle>> &)
    >> cf_list = {inefficiency_cost, max_jerk_cost, max_accel_cost, max_velocity_cost,
      collision_cost, stay_on_road_cost, off_lane_center_cost};
  vector<float> weight_list = {EFFICIENCY, MAX_JERK, MAX_ACCELERATION, MAX_VELOCITY,
    COLLISION, STAY_ON_ROAD, LANE_CENTER};
    
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i]*cf_list[i](ego, trajectory, predictions);
    cost += new_cost;
  }

  return cost;
}

float nearest_vehicle(const Trajectory<Vehicle> &trajectory, const map<int, Trajectory<Vehicle>> &predictions) {
  float nearest = 1000000; // big number
  double DT = 0.02; // WARNING -- THIS MUST LINE UP WITH WHAT WAS USED TO GENERATE THE TRAJECTORY!
  double t = 0;
  for (const auto& ego : trajectory.path) {
    for (const auto& kv : predictions) {
      const Vehicle& predicted = kv.second.path[0].at(t); // Assume constant acceleration of other vehicles.
      float d = distance(ego.s, ego.d, predicted.s, predicted.d);
      nearest = std::min(nearest, d);
    }
    t += DT;
  }
  return nearest;
}
