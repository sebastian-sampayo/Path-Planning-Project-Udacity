#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "logger.h"
#include "path_planner.h"
#include "sensor_data.h"
#include "trajectory.h"
#include "utils.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // SET_LOG_LEVEL(logDEBUG2);
  // SET_LOG_LEVEL(logWARNING);
  // SET_LOG_LEVEL(logINFO);
  SET_LOG_LEVEL(logERROR);
  PathPlanner path_planner;
  
  Timer timer;

  h.onMessage([&path_planner, &timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    LOG(logINFO) << "main() - Elapsed time since last message end: " << timer.GetElapsedMiliSeconds() << "ms";
    timer.Reset();
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          EgoSensorData ego_data;
          ego_data.x = car_x;
          ego_data.y = car_y;
          ego_data.s = car_s;
          ego_data.d = car_d;
          ego_data.speed = car_speed;
          ego_data.yaw = car_yaw; // degrees
          path_planner.SetEgoData(ego_data);
          
          // Convert previous path from json to Trajectory class
          path_planner.SetPreviousPath(previous_path_x, previous_path_y);
          // path_planner.SetPreviousEndPoint(end_path_s, end_path_d);
          // path_planner.SetPointsAlreadyPassed()

          // Convert Sensor fusion data from json to SensorData class:
          EnvironmentSensorData environment_data;
          for (const auto& sensed_vehicle : sensor_fusion)
          {
            EnvironmentSensorData::SensedVehicleData data;
            data.id = sensed_vehicle[0];
            data.x = sensed_vehicle[1];
            data.y= sensed_vehicle[2];
            data.vx = sensed_vehicle[3];
            data.vy = sensed_vehicle[4];
            data.s = sensed_vehicle[5];
            data.d = sensed_vehicle[6];
            environment_data.sensed_vehicle_list.push_back(data);
          }
          path_planner.SetEnvironmentData(environment_data);

          Trajectory next_path = path_planner.Generate();

          // JSON message
          json msgJson;
          msgJson["next_x"] = next_path.GetXvalues();
          msgJson["next_y"] = next_path.GetYvalues();

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
    
    LOG(logINFO) << "main() - Elapsed time of the PathPlanner, h.onMessage(): " << timer.GetElapsedMiliSeconds() << "ms";
    timer.Reset();
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
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
















































































