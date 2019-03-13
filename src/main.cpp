#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "vehicle.h"
#include "trajectory.h"
#include "cost.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> raw_map_waypoints_iteration;
  vector<double> raw_map_waypoints_x;
  vector<double> raw_map_waypoints_y;
  vector<double> raw_map_waypoints_s;
  vector<double> raw_map_waypoints_dx;
  vector<double> raw_map_waypoints_dy;
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  double iteration = 0;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    raw_map_waypoints_iteration.push_back(iteration);
    iteration += 1;
    raw_map_waypoints_x.push_back(x);
    raw_map_waypoints_y.push_back(y);
    raw_map_waypoints_s.push_back(s);
    raw_map_waypoints_dx.push_back(d_x);
    raw_map_waypoints_dy.push_back(d_y);
  }
  // create splines in the XY space, since I don't trust the transformation from SD to be very smooth.
  tk::spline waypoints_x_spline, waypoints_y_spline, waypoints_s_spline, waypoints_dx_spline, waypoints_dy_spline;
  waypoints_x_spline.set_points(raw_map_waypoints_iteration, raw_map_waypoints_x);
  waypoints_y_spline.set_points(raw_map_waypoints_iteration, raw_map_waypoints_y);
  waypoints_s_spline.set_points(raw_map_waypoints_iteration, raw_map_waypoints_s);
  waypoints_dx_spline.set_points(raw_map_waypoints_iteration, raw_map_waypoints_dx);
  waypoints_dy_spline.set_points(raw_map_waypoints_iteration, raw_map_waypoints_dy);

  vector<double>& map_waypoints_x = Vehicle::map_waypoints_x;
  vector<double>& map_waypoints_y = Vehicle::map_waypoints_y;
  vector<double>& map_waypoints_s = Vehicle::map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  const int WAYPOINT_INTERPOLATION_FACTOR = 10;
  double d_i = 1.0 / WAYPOINT_INTERPOLATION_FACTOR;
  for (int i = 0; i < raw_map_waypoints_x.size(); i++) {
    double iteration = i;
    for (int j = 0; j < WAYPOINT_INTERPOLATION_FACTOR; j++) {
      map_waypoints_x.push_back(waypoints_x_spline(iteration));
      map_waypoints_y.push_back(waypoints_y_spline(iteration));
      map_waypoints_s.push_back(waypoints_s_spline(iteration));
      map_waypoints_dx.push_back(waypoints_dx_spline(iteration));
      map_waypoints_dy.push_back(waypoints_dy_spline(iteration));
      iteration += d_i;
    }
  }

  bool sensors_initiliazed = false;
  double d = 6.0;
  Vehicle ego(0, 0, 0, d, 0, 0); // Also initialized in onConnected.
  vector<Vehicle> ego_history;

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
      &map_waypoints_dx, &map_waypoints_dy, &ego, &sensors_initiliazed, &max_s, &ego_history]
      (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
       uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
      auto j = json::parse(s);

      string event = j[0].get<string>();

      if (event == "telemetry") {
      auto previous_path_x = j[1]["previous_path_x"];
      auto previous_path_y = j[1]["previous_path_y"];

      // ego's localization Data
      double car_x = j[1]["x"];
      double car_y = j[1]["y"];
      double car_s = j[1]["s"];
      double car_d = j[1]["d"];
      double car_yaw = j[1]["yaw"];
      if (!sensors_initiliazed) {
        sensors_initiliazed = true;
        ego.s = car_s;
        ego.d = car_d;
      }

      // rewind state to the first unprocessed state
      const int first_unprocessed_i = ego_history.size() - previous_path_x.size();
      if (first_unprocessed_i > 0 && first_unprocessed_i < ego_history.size()) {
        // revert the ego state back to the first unprocessed state
        ego = ego_history[first_unprocessed_i];
        ego_history.clear();
      }

      // Sensor Fusion Data, a list of all other cars on the same side 
      //   of the road.
      auto sensor_fusion = j[1]["sensor_fusion"];
      const double DT = 0.02;
      const double TIME_HORIZON = 2;
      const int STEP_HORIZON = TIME_HORIZON / DT;
      map<int, Vehicle> sensed_vehicles;
      for (auto& sensed_vehicle : sensor_fusion) {
        // format is [car ID, x(map), y(map), vx (m/s), vy (m/s), s, d]
        int id = sensed_vehicle[0];
        double vx = sensed_vehicle[3];
        double vy = sensed_vehicle[4];
        double s = sensed_vehicle[5];
        double vs = distance(0, 0, vx, vy); // TODO: maybe turn this into s and d components
        double as = 0; // TODO: fill in
        double d = sensed_vehicle[6];
        double vd = 0, ad = 0;
        sensed_vehicles[id] = Vehicle(s, vs, as, d, vd, ad); 
      }
      vector<double> next_x_vals;
      vector<double> next_y_vals;

      // generate trajectory
      map<int, Trajectory<Vehicle>> predictions;
      for (int i = 0; i < STEP_HORIZON;) {
        for (auto& kv : sensed_vehicles) {
          predictions[kv.first] = kv.second.at(i*DT).generate_predictions(TIME_HORIZON, DT);
        }
        Trajectory<Vehicle> trajectory = ego.choose_next_trajectory(predictions, DT);
        for (int j = 1; j < trajectory.path.size() && i < STEP_HORIZON; j++) {
          i++;
          ego = trajectory.path[j];
          ego_history.push_back(ego);
          //std::cout << "i: " << i << ", j: " << j << ", lane: " << ego.lane() << ", s: " << ego.s << ", vs: " << ego.vs << ", as: " << ego.as << ", d: " << ego.d << ", vd: " << ego.vd << ", ad: " << ego.ad << std::endl;

          auto xy = getXY(ego.s, ego.d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          next_x_vals.push_back(xy[0]);
          next_y_vals.push_back(xy[1]);
        }
      }
      json msgJson;
      msgJson["next_x"] = next_x_vals;
      msgJson["next_y"] = next_y_vals;

      auto msg = "42[\"control\","+ msgJson.dump()+"]";

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
      }  // end websocket if
      }); // end h.onMessage

  h.onConnection([&h, &ego, &ego_history, &sensors_initiliazed](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      std::cout << "Connected!!!" << std::endl;
      ego.s = 0;
      ego.vs = 0;
      ego.as = 0;
      ego.d = 6.0;
      ego.vd = 0;
      ego.ad = 0;
      sensors_initiliazed = false;
      ego_history.clear();
      });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
        char *message, size_t length) {
      ws.close();
      std::cout << "Disconnected" << std::endl;
      });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
