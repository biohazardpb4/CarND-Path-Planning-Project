#include "vehicle.h"
#include "helpers.h"
#include "cost.h"
#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <math.h>

using std::string;
using std::vector;

vector<double> Vehicle::map_waypoints_x = vector<double>{};
vector<double> Vehicle::map_waypoints_y = vector<double>{};
vector<double> Vehicle::map_waypoints_s = vector<double>{};

// Initializes Vehicle
Vehicle::Vehicle() {}

Vehicle::Vehicle(double s, double s_dot, double s_ddot, double d, double d_dot, double d_ddot, string state) : s(d), s_dot(s_dot), s_ddot(s_ddot), d(d), d_dot(d_dot), d_ddot(d_ddot), state(state) {}

Vehicle::~Vehicle() {}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> &predictions, double dt)
{
  /**
   *
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego 
   *   vehicle state.
   */
  bool min_cost_init = false;
  double min_cost = 0;
  vector<Vehicle> min_trajectory;

  for (auto const &next : this->successor_states())
  {
    auto const &potential_trajectory = generate_trajectory(next, predictions, dt);
    auto const &potential_cost = calculate_cost(*this, predictions, potential_trajectory);
    std::cout << "state: " << next << ", cost: " << potential_cost << std::endl;
    if (!min_cost_init || potential_cost < min_cost)
    {
      min_cost_init = true;
      min_cost = potential_cost;
      min_trajectory = potential_trajectory;
    }
  }

  if (!min_cost_init)
  {
    return generate_trajectory("KL", predictions, dt);
  }
  return min_trajectory;
}

vector<string> Vehicle::successor_states()
{
  vector<string> states;
  states.push_back("KL");
  if (lane > 0)
  {
    states.push_back("LCL");
  }
  if (lane < lanes_available - 1)
  {
    states.push_back("LCR");
  }
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state,
                                             map<int, vector<Vehicle>> &predictions,
                                             double dt)
{
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  vector<Vehicle> trajectory;
  if (state.compare("CS") == 0)
  {
    trajectory = constant_speed_trajectory(dt);
  }
  else if (state.compare("KL") == 0)
  {
    trajectory = keep_lane_trajectory(predictions, dt);
  }
  else if (state.compare("LCL") == 0 || state.compare("LCR") == 0)
  {
    trajectory = change_lane_trajectory(state, predictions, dt);
  }
  return trajectory;
}

vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions,
                                       int lane, double dt, bool debug)
{
  // Gets next timestep kinematics (position, velocity, acceleration)
  //   for a given lane. Tries to choose the maximum velocity and acceleration,
  //   given other vehicle positions and accel/velocity constraints.
  double max_velocity_accel_limit = this->max_acceleration * dt + this->s_dot;
  double scaled_target_speed = projectOnWaypointPath(this->target_speed, this->s, this->d, Vehicle::map_waypoints_s, Vehicle::map_waypoints_x, Vehicle::map_waypoints_y);
  double new_position;
  double new_velocity;
  double new_accel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;
  if (get_vehicle_ahead(predictions, lane, vehicle_ahead))
  {
    /*if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
      // must travel at the speed of traffic, regardless of preferred buffer
      new_velocity = std::min(std::min(vehicle_ahead.s_dot, max_velocity_accel_limit), this->target_speed);
      if(debug) std::cout << "vehicle ahead and behind. new velocity: " << new_velocity << std::endl;
    } else {*/
    double max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.s_dot - 0.5 * (this->s_ddot);
    new_velocity = std::min(std::min(max_velocity_in_front,
                                     max_velocity_accel_limit),
                            scaled_target_speed);
    if (debug)
      std::cout << "vehicle ahead. new velocity: " << new_velocity << std::endl;
    //}
  }
  else
  {
    new_velocity = std::min(max_velocity_accel_limit, scaled_target_speed);
    if (debug)
      std::cout << "clear path. new velocity: " << new_velocity << std::endl;
  }
  new_velocity = std::max(new_velocity, 0.0);
  new_accel = (new_velocity - this->s_dot) / dt;                    // Equation: (v_1 - v_0)/t = acceleration
  new_accel = std::max(new_accel, -this->max_acceleration);         // cap the acceleration on the low end
  new_position = this->s + 0.5 * (this->s_dot + new_velocity) * dt; // d' = d + (vi + vf) / 2 * t
  //if(debug) std::cout << "old s: " << this->s << ", v: " << this->s_dot << ", a: " << this->s_ddot << std::endl;
  //if(debug) std::cout << "new s: " << new_position << ", v: " << new_velocity << ", a: " <<new_accel << std::endl;
  return {new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::constant_speed_trajectory(double dt)
{
  // Generate a constant speed trajectory.
  double next_pos = position_at(dt);
  vector<Vehicle> trajectory = {Vehicle(this->s, this->s_dot, this->s_ddot, this->d, this->d_dot, this->d_ddot, this->state),
                                Vehicle(next_pos, this->s_dot, 0, this->d, this->d_dot, this->d_ddot, this->state)};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> &predictions, double dt)
{
  // Generate a keep lane trajectory.
  vector<Vehicle> trajectory = {Vehicle(this->s, this->s_dot, this->s_ddot, this->d, this->d_dot, this->d_ddot, this->state)};
  vector<double> kinematics = get_kinematics(predictions, this->lane, dt, false);
  double new_s = kinematics[0];
  double new_s_dot = kinematics[1];
  double new_s_ddot = kinematics[2];
  //std::cout << "keep lane s: " << new_s << ", v: " << new_v << ", a: " << new_a << std::endl;
  auto next = Vehicle(new_s, new_s_dot, new_s_ddot, this->d, this->d_dot, this->d_ddot, "KL");
  next.target_speed = this->target_speed;
  next.lanes_available = this->lanes_available;
  next.max_acceleration = this->max_acceleration;
  trajectory.push_back(next);
  return trajectory;
}

vector<Vehicle> Vehicle::change_lane_trajectory(string state,
                                                map<int, vector<Vehicle>> &predictions,
                                                double dt)
{
  // Generate a lane change trajectory.
  int new_lane = this->lane + this->lane_direction[state];
  double new_d = (new_lane + 1) * 4 - 2;
  vector<Vehicle> trajectory;
  // TODO: Check if a lane change is possible (check if another vehicle occupies
  //   that spot).
  trajectory.push_back(*this);

  const double TIME_HORIZON = 2;
  auto start = vector<double>{this->d, 0, 0};
  auto end = vector<double>{new_d, 0, 0};
  auto coeffs = jerk_min_trajectory(start, end, TIME_HORIZON);

  double d_i = this->d, a_3(coeffs[3]), a_4(coeffs[4]), a_5(coeffs[5]);
  const int STEP_HORIZON = int(TIME_HORIZON * 50);
  for (int i = 1; i <= STEP_HORIZON; i++)
  {
    double t = dt * i;
    double d = d_i + a_3 * pow(t, 3) + a_4 * pow(t, 4) + a_5 * pow(t, 5);

    vector<double> kinematics = trajectory[i - 1].get_kinematics(predictions, new_lane, dt);
    double s = kinematics[0];
    double s_dot = kinematics[1];
    double s_ddot = 0;
    auto next = Vehicle(s, s_dot, s_ddot, d, 0, 0, state);
    next.target_speed = this->target_speed;
    next.lanes_available = this->lanes_available;
    next.max_acceleration = this->max_acceleration;
    trajectory.push_back(next);
  }
  return trajectory;
}

void Vehicle::increment(double dt = 1)
{
  this->s = position_at(dt);
}

double Vehicle::position_at(double t)
{
  return this->s + this->s_dot * t + this->s_ddot * t * t / 2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> &predictions,
                                 int lane, Vehicle &rVehicle)
{
  // Returns a true if a vehicle is found behind the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
       it != predictions.end(); ++it)
  {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s)
    {
      max_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }

  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> &predictions,
                                int lane, Vehicle &rVehicle)
{
  // Returns a true if a vehicle is found ahead of the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  double min_s = 100000.0; // big number
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
       it != predictions.end(); ++it)
  {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s)
    {
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }

  return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(double horizon, double dt)
{
  // Generates predictions for non-ego vehicles to be used in trajectory
  //   generation for the ego vehicle.
  vector<Vehicle> predictions;
  double next_v = 0;
  double horizon_s = this->s + horizon;
  double next_s = 0;
  int i = 1;
  while (next_s < horizon_s)
  {
    next_s = position_at(i * dt);
    next_v = position_at((i + 1) * dt) - next_s;
    predictions.push_back(Vehicle(next_s, next_v, 0, this->d, 0, 0));
    i++;
    if (i > 10)
    {
      break;
    }
  }

  return predictions;
}

void Vehicle::realize_next_state(Vehicle &next_state)
{
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->d = next_state.d;
  this->s = next_state.s;
  this->s_dot = next_state.s_dot;
  this->s_ddot = next_state.s_ddot;
  //std::cout << "realizing new state -- s: " << this->s << ", v: " << this->s_dot << ", a: " << this->s_ddot << std::endl;
}
