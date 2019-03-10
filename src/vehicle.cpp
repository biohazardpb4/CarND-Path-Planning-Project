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

Vehicle::Vehicle(double s, double vs, double as, double d, double vd, double ad, string state) :
  s(s), vs(vs), as(as), d(d), vd(vd), ad(ad), state(state) {
    this->lane = int(d/4);
  }

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
  double max_velocity_accel_limit = this->max_acceleration * dt + this->vs;
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
      new_velocity = std::min(std::min(vehicle_ahead.vs, max_velocity_accel_limit), this->target_speed);
      if(debug) std::cout << "vehicle ahead and behind. new velocity: " << new_velocity << std::endl;
    } else {*/
    double max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.vs - 0.5 * (this->as);
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
  new_accel = (new_velocity - this->vs) / dt;                    // Equation: (v_1 - v_0)/t = acceleration
  new_accel = std::max(new_accel, -this->max_acceleration);         // cap the acceleration on the low end
  new_position = this->s + 0.5 * (this->vs + new_velocity) * dt; // d' = d + (vi + vf) / 2 * t
  //if(debug) std::cout << "old s: " << this->s << ", v: " << this->vs << ", a: " << this->as << std::endl;
  //if(debug) std::cout << "new s: " << new_position << ", v: " << new_velocity << ", a: " <<new_accel << std::endl;
  return {new_position, new_velocity, new_accel};
}

// at returns a vehicle with position and velocity updated based on a supplied time delta.
Vehicle Vehicle::at(double dt) {
  double new_s = this->s + this->vs*dt + 0.5*this->as*pow(dt, 2);
  double new_vs = this->vs + this->as*dt;
  double new_as = this->as;
  double new_d = this->d + this->vd*dt + 0.5*this->ad*pow(dt, 2);
  double new_vd = this->vd + this->ad*dt;
  double new_ad = this->ad;
  return Vehicle(new_s, new_vs, new_as, new_d, new_vd, new_ad, this->state);
}

vector<Vehicle> Vehicle::constant_speed_trajectory(double dt)
{
  // Generate a constant speed trajectory.
  vector<Vehicle> trajectory = {*this, this->at(dt)};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> &predictions, double dt)
{
  // Generate a keep lane trajectory.
  vector<Vehicle> trajectory = {*this};
  vector<double> kinematics = get_kinematics(predictions, this->lane, dt, false);
  double new_s = kinematics[0];
  double new_vs = kinematics[1];
  double new_as = kinematics[2];
  //std::cout << "keep lane s: " << new_s << ", v: " << new_v << ", a: " << new_a << std::endl;
  auto next = Vehicle(new_s, new_vs, new_as, this->d, this->vd, this->ad, "KL");
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
  double goal_d = (new_lane + 1) * 4 - 2;
  vector<Vehicle> trajectory;
  // TODO: Check if a lane change is possible (check if another vehicle occupies
  //   that spot).
  trajectory.push_back(*this);

  const double TIME_HORIZON = 2; // TODO: perturb this
  auto start = vector<double>{this->d, this->vd, this->ad};
  auto end = vector<double>{goal_d, 0, 0};
  auto coeffs = jerk_min_trajectory(start, end, TIME_HORIZON);

  double d_0 = this->d, vd_0 = this->vd, ad_0 = this->ad, jd_0(coeffs[3]), sd_0(coeffs[4]), cd_0(coeffs[5]);
  const int STEP_HORIZON = int(TIME_HORIZON * 50);
  for (int i = 1; i <= STEP_HORIZON; i++)
  {
    // https://en.wikipedia.org/wiki/Pop_(physics)
    double t = dt * i;
    double d = d_0 + vd_0*t + (1.0/2.0)*ad_0*pow(t, 2) + (1.0/6.0)*jd_0*pow(t, 3) +
      (1.0/24.0)*sd_0*pow(t, 4) + (1.0/120.0)*cd_0*pow(t, 5);
    double vd = vd_0 + ad_0*t + (1.0/2.0)*jd_0*pow(t, 2) + (1.0/6.0)*sd_0*pow(t, 3) +
      (1.0/24.0)*cd_0*pow(t, 4);
    double ad = ad_0 + jd_0*t + (1.0/2.0)*sd_0*pow(t, 2) + (1.0/6.0)*cd_0*pow(t, 3);

    vector<double> kinematics = trajectory[i - 1].get_kinematics(predictions, new_lane, dt);
    double s = kinematics[0];
    double vs = kinematics[1];
    double as = 0;
    auto next = Vehicle(s, vs, as, d, vd, ad, state);
    trajectory.push_back(next);
  }
  return trajectory;
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
    predictions.push_back(this->at(i * dt));
    i++;
    if (i > 10)
    {
      break;
    }
  }

  return predictions;
}
