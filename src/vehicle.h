#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>
#include "helpers.h"
#include "trajectory.h"

using std::map;
using std::string;
using std::vector;

class Vehicle
{
public:
  // Constructors
  Vehicle();
  Vehicle(double s, double vs, double as, double d, double vd, double ad);

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  Trajectory<Vehicle> choose_next_trajectory(map<int, Trajectory<Vehicle>> &predictions, double dt);

  //vector<double> get_kinematics(map<int, Trajectory<Vehicle>> &predictions, int lane, double dt, bool debug = false);

  Vehicle at(double dt) const;

  // Trajectories
  vector<Trajectory<Vehicle>> target_speed_trajectories(double dt);
  vector<Trajectory<Vehicle>> slow_down_for_ahead_trajectories(map<int, Trajectory<Vehicle>> &predictions, double dt);
  vector<Trajectory<Vehicle>> change_lane_trajectories(map<int, Trajectory<Vehicle>> &predictions, double dt);

  Trajectory<Vehicle> generate_trajectory(
    string generated_by,
    vector<double> start_s, vector<double> end_s, vector<double> start_d, vector<double> end_d,
    double time_horizon);

  // bool get_vehicle_behind(map<int, Trajectory<Vehicle>> &predictions, int lane,
  //                         Vehicle &rVehicle);

  bool get_vehicle_ahead(map<int, Trajectory<Vehicle>> &predictions, int lane, Vehicle &rVehicle);

  Trajectory<Vehicle> generate_predictions(double horizon, double dt);

  int lane() const;

  double target_speed = mph2mps(50);
  double max_acceleration = 10;
  int lanes_available = 3;
  int preferred_buffer = 12;

  // Kinematic coefficients for s and d coord.
  double s, vs, as;
  double d, vd, ad;

  static vector<double> map_waypoints_x;
  static vector<double> map_waypoints_y;
  static vector<double> map_waypoints_s;
};

#endif // VEHICLE_H
