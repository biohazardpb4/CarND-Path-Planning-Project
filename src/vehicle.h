#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(double d, double s, double v, double a, string state="CS");

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &predictions, double dt);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, 
                                      map<int, vector<Vehicle>> &predictions, double dt);

  vector<double> get_kinematics(map<int, vector<Vehicle>> &predictions, int lane, double dt, bool debug=false);

  vector<Vehicle> constant_speed_trajectory(double dt);

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &predictions, double dt);

  vector<Vehicle> change_lane_trajectory(string state, 
                                         map<int, vector<Vehicle>> &predictions, double dt);

  void increment(double dt);

  double position_at(double t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane, 
                          Vehicle &rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane, 
                         Vehicle &rVehicle);

  vector<Vehicle> generate_predictions(double horizon, double dt);

  void realize_next_state(Vehicle &next_state);

  map<string, int> lane_direction = {{"LCL", -1}, {"LCR", 1}};

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane, lanes_available;//,goal_lane, goal_s, ;

  double s, d, v, target_speed, a, max_acceleration;

  string state;

  static vector<double> map_waypoints_x;
  static vector<double> map_waypoints_y;
  static vector<double> map_waypoints_s;
};

#endif  // VEHICLE_H
