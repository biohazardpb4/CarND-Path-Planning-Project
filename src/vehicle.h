#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>
#include "helpers.h"

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
  vector<Vehicle> choose_next_trajectory(map<int, vector<Vehicle>> &predictions, double dt);

  //vector<double> get_kinematics(map<int, vector<Vehicle>> &predictions, int lane, double dt, bool debug = false);

  Vehicle at(double dt);

  // Trajectories
  vector<vector<Vehicle>> constant_speed_trajectories(double dt);
  // vector<vector<Vehicle>> keep_lane_trajectories(map<int, vector<Vehicle>> &predictions, double dt);
  // vector<vector<Vehicle>> change_lane_trajectories(string state, map<int, vector<Vehicle>> &predictions, double dt);

  // bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane,
  //                         Vehicle &rVehicle);

  // bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane,
  //                        Vehicle &rVehicle);

  vector<Vehicle> generate_predictions(double horizon, double dt);

  int lane() const;

  double target_speed = mph2mps(50);
  double max_acceleration = 10;
  int lanes_available = 3;
  int preferred_buffer = 6;

  // Kinematic coefficients for s and d coord.
  double s, vs, as;
  double d, vd, ad;

  static vector<double> map_waypoints_x;
  static vector<double> map_waypoints_y;
  static vector<double> map_waypoints_s;
};

#endif // VEHICLE_H
