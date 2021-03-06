#include "vehicle.h"
#include "helpers.h"
#include "trajectory.h"
#include "cost.h"
#include <algorithm>
#include <iostream>
#include <sstream>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <random>

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
  Vehicle ahead;
  bool is_vehicle_ahead = this->get_vehicle_ahead(predictions, ahead);
  if (is_vehicle_ahead && ahead.s - this->s < 50) {
    if(this->lane() > 0 && this->lane_available(predictions, this->lane()-1)) {
      // change lane left
      std::cout << "change lane left" << std::endl;
      for (auto const &potential_trajectory : this->change_lane_left_trajectories(predictions, dt)) {
        potential_trajectories.push_back(potential_trajectory);
      }
    } else if (this->lane() < this->lanes_available-1 && this->lane_available(predictions, this->lane()+1)) {
      // change lane right
      std::cout << "change lane right" << std::endl;
      for (auto const &potential_trajectory : this->change_lane_right_trajectories(predictions, dt)) {
        potential_trajectories.push_back(potential_trajectory);
      }
    } else {
      // slow down
      std::cout << "slow down" << std::endl;
      for (auto const &potential_trajectory : this->slow_down_for_ahead_trajectories(predictions, dt)) {
        potential_trajectories.push_back(potential_trajectory);
      }
    }
  } else {
    int center_lane = this->lanes_available/2;
    if (this->lane() != center_lane) {
      if (this->lane() > center_lane && this->lane_available(predictions, this->lane()-1)) {
        std::cout << "not center, change lane left" << std::endl;
        // change lane left
        for (auto const &potential_trajectory : this->change_lane_left_trajectories(predictions, dt)) {
          potential_trajectories.push_back(potential_trajectory);
        }
      }
      else if (this->lane() < center_lane && this->lane_available(predictions, this->lane()+1)) {
        std::cout << "not center, change lane right" << std::endl;
        // change lane right
        for (auto const &potential_trajectory : this->change_lane_right_trajectories(predictions, dt)) {
          potential_trajectories.push_back(potential_trajectory);
        }
      }
    } else {
      // get up to speed
      std::cout << "get up to speed" << std::endl;
      for (auto const &potential_trajectory : target_speed_trajectories(dt)) {
        potential_trajectories.push_back(potential_trajectory);
      }
      if (this->vs < this->target_speed/2.0) {
        for (auto const &potential_trajectory : kinematic_trajectories(dt)) {
          potential_trajectories.push_back(potential_trajectory);
        }
      }
    }
  }

  if (potential_trajectories.size() == 0) {
    // generate some fallback trajectories
    std::cout << "Generating fallback trajectories" << std::endl;
    for (auto const &potential_trajectory : this->slow_down_for_ahead_trajectories(predictions, dt)) {
      potential_trajectories.push_back(potential_trajectory);
    }
    // get up to speed
    for (auto const &potential_trajectory : target_speed_trajectories(dt)) {
      potential_trajectories.push_back(potential_trajectory);
    }
  }

  double min_cost = calculate_cost(*this, predictions, potential_trajectories[0]);
  Trajectory<Vehicle> min_trajectory = potential_trajectories[0];
  for (auto &potential_trajectory : potential_trajectories) {
    auto const &potential_cost = calculate_cost(*this, predictions, potential_trajectory);
    // std::cout << "potential: " << potential_trajectory << std::endl;
    if (potential_cost < min_cost) {
      min_cost = potential_cost;
      min_trajectory = potential_trajectory;
    }
  }

  std::cout << "chose: " << min_trajectory << std::endl << std::endl;
  this->wrap_s(min_trajectory);

  return min_trajectory;
}

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

bool Vehicle::get_vehicle_ahead(map<int, Trajectory<Vehicle>> &predictions,
                                Vehicle &rVehicle)
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

bool Vehicle::lane_available(map<int, Trajectory<Vehicle>> &predictions, int lane) {
  // Returns a true if there is no vehicle close in the desired lane.
  bool close_vehicle = false;
  double min_s = this->s - 10;
  double max_s = this->s + 35;
  for (map<int, Trajectory<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
  {
    Vehicle other = it->second.path[0];
    if (other.lane() == lane && other.s > min_s && other.s < max_s)
    {
      close_vehicle = true;
    }
  }

  return !close_vehicle;
}

vector<Trajectory<Vehicle>> Vehicle::target_speed_trajectories(double dt)
{
  vector<Trajectory<Vehicle>> trajectories;
  const double TIME_HORIZON = 2;
  // Generates options of several scales to eventually get up to speed.
  for (double scale = 0.4; scale <= 1.0; scale += 0.2) {
    auto start_s = vector<double>{this->s, this->vs, this->as};
    double goal_vs = scale*this->target_speed;
    auto end_s = vector<double>{this->s+(this->vs+goal_vs)/2.0*TIME_HORIZON, goal_vs, 0};
    auto start_d = vector<double>{this->d, this->vd, this->ad};
    auto end_d = vector<double>{this->lane()*4.0+2.0, 0, 0};
    std::ostringstream label;
    label << "target_speed (" << scale*100 << "%%)";
    const auto& subtrajectories = this->generate_trajectories(label.str(), start_s, end_s, start_d, end_d, TIME_HORIZON);
    trajectories.insert(trajectories.end(), subtrajectories.begin(), subtrajectories.end());
  }
    // Generates longer range options ranging from 6 to 9 seconds for getting into the lane center smoothly.
  for (int i = 6; i <= 13; i+=2) {
    auto start_s = vector<double>{this->s, this->vs, this->as};
    auto end_s = vector<double>{this->s+(this->vs+this->target_speed)/2.0*i, this->target_speed, 0};
    auto start_d = vector<double>{this->d, this->vd, this->ad};
    auto end_d = vector<double>{this->lane()*4.0+2.0, 0, 0};
    std::ostringstream label;
    label << "target_speed (" << i << "s)";
    const auto& subtrajectories = this->generate_trajectories(label.str(), start_s, end_s, start_d, end_d, i);
    trajectories.insert(trajectories.end(), subtrajectories.begin(), subtrajectories.end());
  }
  return trajectories;
}

vector<Trajectory<Vehicle>> Vehicle::kinematic_trajectories(double dt)
{
  // Kinematic is meant for use in slow situations.
  Trajectory<Vehicle> trajectory;
  trajectory.generated_by = "kinematic";
  const double TIME_HORIZON = 2;
  auto v = *this;
  double goal_as = 0.7*this->max_acceleration_slow;
  double goal_ad = 0, goal_vd = 0;
  for (double t = 0; t <= TIME_HORIZON; t+=dt) {
    v = v.at(dt);
    if (v.as < goal_as) {
      v.as += 0.7*v.max_jerk;
      if (v.as > v.max_acceleration_slow) {
        v.as = v.max_acceleration_slow;
      }
    }
    if (v.vs >= this->target_speed) {
      v.as = 0;
    }
    if (v.ad > goal_ad) {
      v.ad -= 0.1*v.max_jerk;
      v.ad = std::max(goal_ad, v.ad);
    }
    if (v.vd > goal_vd) {
      v.vd -= 0.1*v.max_acceleration_slow;
      v.vd = std::max(goal_vd, v.vd);
    }
    trajectory.path.push_back(v);
  }
  return vector<Trajectory<Vehicle>>{trajectory};
}

vector<Trajectory<Vehicle>> Vehicle::slow_down_for_ahead_trajectories(
    map<int, Trajectory<Vehicle>> &predictions, double dt) {
  vector<Trajectory<Vehicle>> trajectories;
  Vehicle ahead;
  if (this->get_vehicle_ahead(predictions, ahead)) {
    const double TIME_HORIZON = 1.5; // TODO: perturb this
    // Min jerk trajectory to reach vehicle ahead less buffer.
    auto start_s = vector<double>{this->s, this->vs, this->as};
    // Predict where the vehicle in front of us will be in 2s.
    Vehicle predicted_ahead = ahead.at(TIME_HORIZON);
    auto end_s = vector<double>{predicted_ahead.s - this->preferred_buffer, predicted_ahead.vs*0.8, predicted_ahead.as};
    auto start_d = vector<double>{this->d, this->vd, this->ad};
    auto end_d = vector<double>{this->lane()*4.0+2.0, 0, 0};
    
    return this->generate_trajectories("slow_down_for_ahead", start_s, end_s, start_d, end_d, TIME_HORIZON);
  }
  return trajectories;
}

vector<Trajectory<Vehicle>> Vehicle::change_lane_left_trajectories(
  map<int, Trajectory<Vehicle>> &predictions, double dt)
{
  vector<Trajectory<Vehicle>> trajectories;
  for (int i = 6; i <= 13; i+=3) {
    // Min jerk trajectory to reach target vs.
    auto start_s = vector<double>{this->s, this->vs, this->as};
    double goal_vs = 0.9*this->vs;
    auto end_s = vector<double>{this->s+(this->vs+goal_vs)/2.0*i, goal_vs, 0};

    auto start_d = vector<double>{this->d, this->vd, this->ad};
    auto end_d = vector<double>{(this->lane()-1)*4.0 + 2.0, 0, 0};
    std::ostringstream label;
    label << "change_lane_left (" << i << "s)";
    const auto& subtrajectories = this->generate_trajectories(label.str(), start_s, end_s, start_d, end_d, i);
    trajectories.insert(trajectories.end(), subtrajectories.begin(), subtrajectories.end());
  }
  
  return trajectories;
}

vector<Trajectory<Vehicle>> Vehicle::change_lane_right_trajectories(
  map<int, Trajectory<Vehicle>> &predictions, double dt)
{
  vector<Trajectory<Vehicle>> trajectories;
  for (int i = 6; i <= 13; i+=3) {
    // Min jerk trajectory to reach target vs.
    auto start_s = vector<double>{this->s, this->vs, this->as};
    double goal_vs = 0.9*this->vs;
    auto end_s = vector<double>{this->s+(this->vs+goal_vs)/2.0*i, goal_vs, 0};

    auto start_d = vector<double>{this->d, this->vd, this->ad};
    auto end_d = vector<double>{(this->lane()+1)*4.0 + 2.0, 0, 0};
    std::ostringstream label;
    label << "change_lane_right (" << i << "s)";
    const auto& subtrajectories = this->generate_trajectories(label.str(), start_s, end_s, start_d, end_d, i);
    trajectories.insert(trajectories.end(), subtrajectories.begin(), subtrajectories.end());
  }
  
  return trajectories;
}

Trajectory<Vehicle> Vehicle::generate_trajectory(
  string generated_by,
  vector<double> start_s, vector<double> end_s, vector<double> start_d, vector<double> end_d,
  double time_horizon) {
  double DT = 0.02;
  // Min jerk trajectory to reach target.
  auto s_coeffs = jerk_min_trajectory(start_s, end_s, time_horizon);
  double s_0 = this->s, vs_0 = this->vs, as_0 = this->as, js_0(s_coeffs[3]), ss_0(s_coeffs[4]), cs_0(s_coeffs[5]);

  auto d_coeffs = jerk_min_trajectory(start_d, end_d, time_horizon);
  double d_0 = this->d, vd_0 = this->vd, ad_0 = this->ad, jd_0(d_coeffs[3]), sd_0(d_coeffs[4]), cd_0(d_coeffs[5]);

  vector<Vehicle> path;
  path.push_back(*this);
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
    path.push_back(next);
  }
  auto trajectory = Trajectory<Vehicle>(generated_by, path);
  this->trim_trajectory(trajectory);
  return trajectory;
}

vector<Trajectory<Vehicle>> Vehicle::generate_trajectories(
  string generated_by,
  vector<double> start_s, vector<double> end_s, vector<double> start_d, vector<double> end_d,
  double time_horizon) {
    vector<Trajectory<Vehicle>> trajectories;
    double SIGMA_S = 5.0, SIGMA_VS = 2.0, SIGMA_AS = 1.0,
      SIGMA_D = 0.5, SIGMA_VD = 0.25, SIGMA_AD = 0.1, SIGMA_T = 0.5;

  std::default_random_engine g;
  std::normal_distribution<double> s(end_s[0], SIGMA_S), vs(end_s[1], SIGMA_VS), as(end_s[2], SIGMA_AS),
    d(end_d[0], SIGMA_D), vd(end_d[1], SIGMA_VD), ad(end_d[2], SIGMA_AD), t(time_horizon, SIGMA_T);

  for (int i = 0; i < 10; i++) {
    vector<double> normal_end_s{s(g), vs(g), as(g)};
    vector<double> normal_end_d{d(g), vd(g), ad(g)};
    double h = t(g);
    trajectories.push_back(this->generate_trajectory(generated_by, start_s, normal_end_s, start_d, normal_end_d, h));
  }
  trajectories.push_back(this->generate_trajectory(generated_by, start_s, end_s, start_d, end_d, time_horizon));
  return trajectories;
}

void Vehicle::trim_trajectory(Trajectory<Vehicle>& trajectory) {
  int horizon_index = int(2.5 / 0.02)-1;
  vector<Vehicle> trimmed_path;
  for (int i = 0; i < horizon_index && i < trajectory.path.size(); i++) {
    trimmed_path.push_back(trajectory.path[i]);
  }
  trajectory.path = trimmed_path;
}

void Vehicle::wrap_s(Trajectory<Vehicle>& trajectory) {
  vector<Vehicle> wrapped;
  for (int i = 0; i < trajectory.path.size(); i++) {
    auto v = trajectory.path[i];
    if (v.s > this->max_s) {
      v.s -= this->max_s;
    }
    wrapped.push_back(v);
  }
  trajectory.path = wrapped;
}

Trajectory<Vehicle> Vehicle::generate_predictions(double horizon, double dt)
{
  // Generates predictions for non-ego vehicles to be used in trajectory
  //   generation for the ego vehicle.
  vector<Vehicle> predictions;
  for (int i = 0; i < int(horizon/dt)+1; i++) {
    predictions.push_back(this->at(i*dt));
  }
  return Trajectory<Vehicle>("predictions", predictions);
}
