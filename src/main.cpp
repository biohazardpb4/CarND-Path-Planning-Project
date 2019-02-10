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
#include "cost.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  Vehicle ego(1, 0, 50, 0, "KL");
  ego.target_speed = mph2mps(50);
  ego.lanes_available = 3;
  ego.goal_s = max_s;
  ego.goal_lane = 1;
  ego.max_acceleration = 10;

  bool initialized = false;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&initialized, &ego]
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
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = mph2mps(j[1]["speed"]);

	  int lane = car_d/4;
	  float acceleration = 0; // TODO: fill this in
	  ego.lane = lane;
	  ego.s = car_s;
	  ego.v = car_speed;
	  ego.a = acceleration;

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

	  // TODO: create a vehicle for each vehicle in sensor_fusion
	  map<int, vector<Vehicle>> predictions;
	  for (auto& sensed_vehicle : sensor_fusion) {
	    // format is [car ID, x(map), y(map), vx (m/s), vy (m/s), s, d]
	    int id = sensed_vehicle[0];
	    double vx = sensed_vehicle[3];
	    double vy = sensed_vehicle[4];
	    double v = distance(0, 0, vx, vy);
	    double a = 0; // TODO: fill in
	    double s = sensed_vehicle[5];
	    double d = sensed_vehicle[6];
	    Vehicle vehicle(int(d/4), s, v, a);
	    predictions[id] = vehicle.generate_predictions(); 
	  }
	  // generate trajectory
	  vector<Vehicle> trajectory = ego.choose_next_state(predictions);
	  ego.realize_next_state(trajectory);
	  // TODO: generate 50 trajectory points and store into next_x_vals and next_y_vals
	  vector<double> next_x_vals;
          vector<double> next_y_vals;
// TODO: this is just a hack to only generate trajectories once for debugging. remove later
if(!initialized) {
	initialized = true;

	  for (int i = 0; i < 50; i++) {
            // one trajectory point is generated for every 0.02 second
            double next_s = ego.position_at(i*0.02);
	    double next_d = ego.lane *4 - 2;
	    auto xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    next_x_vals.push_back(xy[0]);
	    next_y_vals.push_back(xy[1]); 
          }
}

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
	  /*
	  if (!initialized) {
		  initialized = true;
		  //next_x_vals = map_waypoints_x;
		  //next_y_vals = map_waypoints_y;
		  // Interpolate some points between here and the next waypoint.
	  for (int i = 1; i < map_waypoints_x.size(); i++) {
                double y1 = map_waypoints_y[i-1], y2 = map_waypoints_y[i],
		       x1 = map_waypoints_x[i-1], x2 = map_waypoints_x[i];
		// shift to first lane
		x1 += map_waypoints_dx[i-1]*2;
		x2 += map_waypoints_dx[i]*2;
		y1 += map_waypoints_dy[i-1]*2;
		y2 += map_waypoints_dy[i]*2;

		if (x2 - x1 != 0) {
		double m = (y2 - y1)/(x2 - x1);
		for (int j = 0; j < 10; j++) {
			double x = x1 + j*(x2 - x1)/10.0;
			double y = y1 + m*(x - x1);
			next_x_vals.push_back(x);
			next_y_vals.push_back(y);
		}
		} else {
		for (int j = 0; j < 10; j++) {
			double x = x1;
			double y = y1 + j*(y2 - y1)/10.0;
			next_x_vals.push_back(x);
			next_y_vals.push_back(y);
		}
		}
	  }
          }*/
	  
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

  h.onConnection([&h, &initialized](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    initialized = false;
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
