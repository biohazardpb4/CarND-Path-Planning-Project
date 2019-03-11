#include "vehicle.h"
#include "helpers.h"
#include "trajectory.h"
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

Vehicle::Vehicle(double s, double vs, double as, double d, double vd, double ad) :
  s(s), vs(vs), as(as), d(d), vd(vd), ad(ad) {}

Vehicle::~Vehicle() {}

int Vehicle::lane() const {
  return this->d/4;
}

Trajectory<Vehicle> Vehicle::choose_next_trajectory(map<int, Trajectory<Vehicle>> &predictions, double dt)
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
  vector<Trajectory<Vehicle>> potential_trajectories;
  for (auto const &potential_trajectory : slow_down_for_ahead_trajectories(predictions, dt)) {
    potential_trajectories.push_back(potential_trajectory);
  }
  for (auto const &potential_trajectory : change_lane_trajectories(predictions, dt)) {
    potential_trajectories.push_back(potential_trajectory);
  }
  for (auto const &potential_trajectory : target_speed_trajectories(dt)) {
    potential_trajectories.push_back(potential_trajectory);
  }
  // TODO: add other potential trajectories

  double min_cost = calculate_cost(*this, predictions, potential_trajectories[0]);
  Trajectory<Vehicle> min_trajectory = potential_trajectories[0];
  for (auto const &potential_trajectory : potential_trajectories) {
    auto const &potential_cost = calculate_cost(*this, predictions, potential_trajectory);
    if (potential_cost < min_cost) {
      min_trajectory = potential_trajectory;
    }
  }
  return min_trajectory;
}

// vector<double> Vehicle::get_kinematics(map<int, Trajectory<Vehicle>> &predictions,
//                                        int lane, double dt, bool debug)
// {
//   // Gets next timestep kinematics (position, velocity, acceleration)
//   //   for a given lane. Tries to choose the maximum velocity and acceleration,
//   //   given other vehicle positions and accel/velocity constraints.
//   double max_velocity_accel_limit = this->max_acceleration * dt + this->vs;
//   double scaled_target_speed = projectOnWaypointPath(this->target_speed, this->s, this->d, Vehicle::map_waypoints_s, Vehicle::map_waypoints_x, Vehicle::map_waypoints_y);
//   double new_position;
//   double new_velocity;
//   double new_accel;
//   Vehicle vehicle_ahead(0, 0, 0, 0, 0, 0);
//   if (get_vehicle_ahead(predictions, lane, vehicle_ahead))
//   {
//     /*if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
//       // must travel at the speed of traffic, regardless of preferred buffer
//       new_velocity = std::min(std::min(vehicle_ahead.vs, max_velocity_accel_limit), this->target_speed);
//       if(debug) std::cout << "vehicle ahead and behind. new velocity: " << new_velocity << std::endl;
//     } else {*/
//     double max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.vs - 0.5 * (this->as);
//     new_velocity = std::min(std::min(max_velocity_in_front,
//                                      max_velocity_accel_limit),
//                             scaled_target_speed);
//     if (debug)
//       std::cout << "vehicle ahead. new velocity: " << new_velocity << std::endl;
//     //}
//   }
//   else
//   {
//     new_velocity = std::min(max_velocity_accel_limit, scaled_target_speed);
//     if (debug)
//       std::cout << "clear path. new velocity: " << new_velocity << std::endl;
//   }
//   new_velocity = std::max(new_velocity, 0.0);
//   new_accel = (new_velocity - this->vs) / dt;                    // Equation: (v_1 - v_0)/t = acceleration
//   new_accel = std::max(new_accel, -this->max_acceleration);         // cap the acceleration on the low end
//   new_position = this->s + 0.5 * (this->vs + new_velocity) * dt; // d' = d + (vi + vf) / 2 * t
//   //if(debug) std::cout << "old s: " << this->s << ", v: " << this->vs << ", a: " << this->as << std::endl;
//   //if(debug) std::cout << "new s: " << new_position << ", v: " << new_velocity << ", a: " <<new_accel << std::endl;
//   return {new_position, new_velocity, new_accel};
// }

// at returns a vehicle with position and velocity updated based on a supplied time delta.
Vehicle Vehicle::at(double dt) const {
  double new_s = this->s + this->vs*dt + 0.5*this->as*pow(dt, 2);
  double new_vs = this->vs + this->as*dt;
  double new_as = this->as;
  double new_d = this->d + this->vd*dt + 0.5*this->ad*pow(dt, 2);
  double new_vd = this->vd + this->ad*dt;
  double new_ad = this->ad;
  return Vehicle(new_s, new_vs, new_as, new_d, new_vd, new_ad);
}

vector<Trajectory<Vehicle>> Vehicle::target_speed_trajectories(double dt)
{
  vector<Trajectory<Vehicle>> trajectories;
  // Generates options ranging from 1 to 5 seconds to get up to speed.
  for (int i = 1; i < 5; i++) {
    auto start_s = vector<double>{this->s, this->vs, this->as};
    auto end_s = vector<double>{this->s+this->target_speed*i, this->target_speed, 0};
    auto start_d = vector<double>{this->d, this->vd, this->ad};
    auto end_d = vector<double>{this->lane()*4.0+2.0, 0, 0};
    trajectories.push_back(this->generate_trajectory(start_s, end_s, start_d, end_d, i));
  }
  return trajectories;
}

vector<Trajectory<Vehicle>> Vehicle::slow_down_for_ahead_trajectories(
    map<int, Trajectory<Vehicle>> &predictions, double dt) {
  vector<Trajectory<Vehicle>> trajectories;
  Vehicle ahead;
  if (this->get_vehicle_ahead(predictions, this->lane(), ahead)) {
    const double TIME_HORIZON = 1; // TODO: perturb this
    // Min jerk trajectory to reach vehicle ahead less buffer.
    auto start_s = vector<double>{this->s, this->vs, this->as};
    // Predict where the vehicle in front of us will be in 2s.
    Vehicle predicted_ahead = ahead.at(TIME_HORIZON);
    auto end_s = vector<double>{predicted_ahead.s - this->preferred_buffer, predicted_ahead.vs, predicted_ahead.as};
    auto start_d = vector<double>{this->d, this->vd, this->ad};
    auto end_d = vector<double>{this->lane()*4.0+2.0, 0, 0};
    
    trajectories.push_back(this->generate_trajectory(start_s, end_s, start_d, end_d, TIME_HORIZON));
  }
  return trajectories;
}

vector<Trajectory<Vehicle>> Vehicle::change_lane_trajectories(
  map<int, Trajectory<Vehicle>> &predictions, double dt)
{
  vector<Trajectory<Vehicle>> trajectories;
  // TODO: use get_vehicle_ahead to limit goal parameters
  // Vehicle ahead;
  // if (this->get_vehicle_ahead(predictions, this->lane(), ahead)) {

  const double TIME_HORIZON = 1.5; // TODO: perturb this
  // Min jerk trajectory to reach target vs.
  auto start_s = vector<double>{this->s, this->vs, this->as};
  auto end_s = vector<double>{this->s+this->target_speed*TIME_HORIZON*0.95, this->target_speed, 0};

  // Min jerk left lane change.
  if (this->lane() > 0) {
    auto start_d = vector<double>{this->d, this->vd, this->ad};
    auto end_d = vector<double>{(this->lane()-1)*4.0 + 2.0, 0, 0};
    trajectories.push_back(this->generate_trajectory(start_s, end_s, start_d, end_d, TIME_HORIZON));
  }

  // Min jerk right lane change.
  if (this->lane() < this->lanes_available-1) {
    auto start_d = vector<double>{this->d, this->vd, this->ad};
    auto end_d = vector<double>{(this->lane()+1)*4.0 + 2.0, 0, 0};
    trajectories.push_back(this->generate_trajectory(start_s, end_s, start_d, end_d, TIME_HORIZON));
  }
  
  return trajectories;
}

// bool Vehicle::get_vehicle_behind(map<int, Trajectory<Vehicle>> &predictions,
//                                  int lane, Vehicle &rVehicle)
// {
//   // Returns a true if a vehicle is found behind the current vehicle, false
//   //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
//   int max_s = -1;
//   bool found_vehicle = false;
//   for (map<int, Trajectory<Vehicle>>::iterator it = predictions.begin();
//        it != predictions.end(); ++it)
//   {
//     Vehicle temp_vehicle = it->second[0];
//     if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s)
//     {
//       max_s = temp_vehicle.s;
//       rVehicle = temp_vehicle;
//       found_vehicle = true;
//     }
//   }

//   return found_vehicle;
// }

bool Vehicle::get_vehicle_ahead(map<int, Trajectory<Vehicle>> &predictions,
                                int lane, Vehicle &rVehicle)
{
  // Returns a true if a vehicle is found ahead of the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  bool found_vehicle = false;
  double min_s = this->s + this->target_speed*2.0; // Don't return vehicles we'll never reach in 2s.
  for (map<int, Trajectory<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
  {
    Vehicle other = it->second.path[0];
    if (other.lane() == this->lane() && other.s > this->s && other.s < min_s)
    {
      found_vehicle = true;
      min_s = other.s;
      rVehicle = other;
    }
  }

  return found_vehicle;
}

Trajectory<Vehicle> Vehicle::generate_trajectory(
  vector<double> start_s, vector<double> end_s, vector<double> start_d, vector<double> end_d,
  double time_horizon) {
  double DT = 0.02;
  // Min jerk trajectory to reach target.
  auto s_coeffs = jerk_min_trajectory(start_s, end_s, time_horizon);
  double s_0 = this->s, vs_0 = this->vs, as_0 = this->as, js_0(s_coeffs[3]), ss_0(s_coeffs[4]), cs_0(s_coeffs[5]);

  auto d_coeffs = jerk_min_trajectory(start_d, end_d, time_horizon);
  double d_0 = this->d, vd_0 = this->vd, ad_0 = this->ad, jd_0(d_coeffs[3]), sd_0(d_coeffs[4]), cd_0(d_coeffs[5]);

  vector<Vehicle> trajectory;
  trajectory.push_back(*this);
  for (double t=0; t < time_horizon; t+=DT)
  {
    // https://en.wikipedia.org/wiki/Pop_(physics)
    // Note: our j, s, and c coefficients use a different scale (not sure why).
    double s = s_0 + vs_0*t + 0.5*as_0*pow(t, 2) + js_0*pow(t, 3) + ss_0*pow(t, 4) + cs_0*pow(t, 5);
    double vs = vs_0 + as_0*t + 3.0*js_0*pow(t, 2) + 4.0*ss_0*pow(t, 3) + 5.0*cs_0*pow(t, 4);
    double as = as_0 + 6.0*js_0*t + 12.0*ss_0*pow(t, 2) + 20.0*cs_0*pow(t, 3);

    double d = d_0 + vd_0*t + 0.5*ad_0*pow(t, 2) + jd_0*pow(t, 3) + sd_0*pow(t, 4) + cd_0*pow(t, 5);
    double vd = vd_0 + ad_0*t + 3.0*jd_0*pow(t, 2) + 4.0*sd_0*pow(t, 3) + 5.0*cd_0*pow(t, 4);
    double ad = ad_0 + 6.0*jd_0*t + 12.0*sd_0*pow(t, 2) + 20.0*cd_0*pow(t, 3);

    auto next = Vehicle(s, vs, as, d, vd, ad);
    trajectory.push_back(next);
  }
  return trajectory;
}

Trajectory<Vehicle> Vehicle::generate_predictions(double horizon, double dt)
{
  // Generates predictions for non-ego vehicles to be used in trajectory
  //   generation for the ego vehicle.
  vector<Vehicle> predictions;
  for (int i = 0; i < int(horizon/dt)+1; i++) {
    predictions.push_back(this->at(i*dt));
  }
  return Trajectory<Vehicle>(predictions);
}
