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
  Vehicle(double s, double vs, double as, double d, double vd, double ad, string state = "CS");

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &predictions, double dt);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state,
                                      map<int, vector<Vehicle>> &predictions, double dt);

  vector<double> get_kinematics(map<int, vector<Vehicle>> &predictions, int lane, double dt, bool debug = false);

  Vehicle at(double dt);

  // Trajectories
  vector<Vehicle> constant_speed_trajectory(double dt);
  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &predictions, double dt);
  vector<Vehicle> change_lane_trajectory(string state, map<int, vector<Vehicle>> &predictions, double dt);

  bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane,
                          Vehicle &rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane,
                         Vehicle &rVehicle);

  vector<Vehicle> generate_predictions(double horizon, double dt);

// TODO: constify these
  map<string, int> lane_direction = {{"LCL", -1}, {"LCR", 1}};
  double target_speed = mph2mps(50);
  double max_acceleration = 10;
  int lanes_available = 3;

  int preferred_buffer = 6;
  int lane;
  string state;

  // Kinematic coefficients for s and d coord.
  double s, vs, as;
  double d, vd, ad;

  static vector<double> map_waypoints_x;
  static vector<double> map_waypoints_y;
  static vector<double> map_waypoints_s;
};

#endif // VEHICLE_H
