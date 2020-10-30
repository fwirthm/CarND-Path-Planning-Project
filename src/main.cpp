#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <algorithm>
#include <limits>
#include <ctime>


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
  
  int lane = 1;
  //double ref_vel = 49.5;
  double ref_vel = 0.5;
  double accel = 0.224;
  double max_vel = 43.5;
  bool prepare_lcl = false;
  bool prepare_lcr = false;
  clock_t startTime = std::clock();


  h.onMessage([&lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &accel, &max_vel, &prepare_lcl, &prepare_lcr, &startTime]
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
          double car_vel = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          
          int previous_size = previous_path_x.size();

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * Defining a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          //ref_vel = 49.5;
          //look at all surrounding vehicles and prevent us from colliding with them
          
          //determine the closest preceding vehicle
          vector<double> closest_prec_obj;
          double closest_prec_s = std::numeric_limits<double>::max();
          double closest_prec_d;
          double closest_prec_vel = std::numeric_limits<double>::max();
          bool closest_prec_exists = false;
          
          bool lcl = true;
          double left_vel;
          bool lcr = true;
          double right_vel;
          
          bool closest_left_exists = false;
          double closest_left_s = std::numeric_limits<double>::max();
          double closest_left_vel = std::numeric_limits<double>::max();
          
          bool closest_right_exists = false;
          double closest_right_s = std::numeric_limits<double>::max();
          double closest_right_vel = std::numeric_limits<double>::max();

          //this for loop analyzes all surrounding objects detected by the sensors
          for (int i=0; i<sensor_fusion.size(); i++){
            vector<double> obj = sensor_fusion[i];
            double obj_s = obj[5];
            double obj_d = obj[6];
            
            double obj_vx = obj[3];
            double obj_vy = obj[4];

            double obj_vel = sqrt((obj_vx*obj_vx) + (obj_vy*obj_vy));
            
            //if this condition is satisfied, the object is in our lane
            //if ((obj_d > ((6*lane)-2)) && (obj_d < ((6*lane)+2))){
            if ((obj_d > (4*lane)) && (obj_d < ((4*lane)+4))){
              //this condition checks if the car is driving in front of us and if it is closer than other cars on that lane
              if ((obj_s>car_s) && (obj_s < closest_prec_s)){
                closest_prec_obj = obj;
                closest_prec_s = obj_s;
                closest_prec_d = obj_d;
                closest_prec_exists = true;
                closest_prec_vel = obj_vel;
              }
            }
            else{
              //if this condition is satisfied, the object is on our left lane
              if ((lane>0) && (obj_d > ((4*lane)-4)) && (obj_d < (4*lane))){
                //if the object is in a certain s range around around our vehicle a lane cghance to the left is not safe
                if ((obj_s<(car_s+30)) && (obj_s>(car_s-30))){
                  lcl = false;
                }
                //this condition checks if the car is driving in front of us and if it is closer than other cars on that lane
                if ((obj_s>car_s) && (obj_s < closest_left_s)){
                  closest_left_s = obj_s;
                  closest_left_vel = obj_vel;
                  closest_left_exists = true;
                }
              }
              else{
                //if this condition is satisfied, the object is on our right lane
                if ((lane<2) && (obj_d > ((4*lane)+4)) && (obj_d < ((4*lane)+8))){
                  //if the object is in a certain s range around around our vehicle a lane cghance to the left is not safe
                  if ((obj_s<(car_s+30)) && (obj_s>(car_s-30))){
                    lcr = false;
                  }
                  //this condition checks if the car is driving in front of us and if it is closer than other cars on that lane
                  if ((obj_s>car_s) && (obj_s < closest_right_s)){
                    closest_right_s = obj_s;
                    closest_right_vel = obj_vel;
                    closest_right_exists = false;
                  }
                }
              }
            }
          }

          if (closest_prec_exists){
            //if the vehicle is driving in front of us and closer than the recommended safety distance 
            // and slower, we need to take action
            if (((closest_prec_s-car_s)<30.) && (closest_prec_vel<car_vel)){
              accel = -0.3;
              if ((closest_prec_s-car_s)<10.){
                accel = -1.;
              }
            }
            else{
              //are we already about to prepare a lane change? if not check if we should do so
              if (!prepare_lcl && !prepare_lcr){
                if (((closest_prec_s-car_s)<70.) && (closest_prec_vel<41.)){
                if (lane==1){
                  if (closest_prec_vel>=closest_right_vel && closest_prec_vel>=closest_left_vel){
                    accel = 0.1;
                  }
                  else{
                    if(closest_left_vel>closest_right_vel){
                      prepare_lcl = true;
                    }
                    else{
                      prepare_lcr = true;
                    }
                  }
                }
                else{
                  if ((lane==0)&&(closest_prec_vel<closest_right_vel)){
                    prepare_lcr = true;
                  }
                  else{
                    if ((lane==2)&&(closest_prec_vel<closest_left_vel)){
                      prepare_lcl = true;
                    }
                  }
                }
                }
                else{
                  accel = 0.2;          
                }
              }
              // prepare the lane change
              else{
                if (car_vel>35){
                  accel = -0.2;
                }
                else{
                  if(car_vel<25.){
                    accel = +0.2;
                  }
                  else{
                    if (prepare_lcl){
                      if (lcl){
                        clock_t endTime = std::clock();
                        clock_t clockTicksTaken = endTime - startTime;
                        double timeInSeconds = clockTicksTaken / (double) CLOCKS_PER_SEC;
                        if (timeInSeconds>0.05){
                          prepare_lcl = false;
                          lane -=1;
                          accel = 0.;
                          startTime = std::clock();
                        }
                      }
                      else{
                        prepare_lcl = false;
                        accel = 0.2;
                      }
                  }
                    else{
                      if (lcr){
                        clock_t endTime = std::clock();
                        clock_t clockTicksTaken = endTime - startTime;
                        double timeInSeconds = clockTicksTaken / (double) CLOCKS_PER_SEC;
                        if (timeInSeconds>0.05){
                          prepare_lcr = false;
                          lane += 1;
                          accel = 0.;
                          startTime = std::clock();
                        }
                      }
                      else{
                        prepare_lcr = false;
                        accel = 0.2;
                      }
                    }
                  }
                }
              }
            }
          }
          else{
            accel = 0.224;
          }
          
          //on startup or low speeds the following check is not neccessary
          if (ref_vel>15.){
            //if a lane change maneuver was started less than 2 seconds ago, the acceleration is hold at a avlue of 0 no matter what other conditions tell the vehicle
            clock_t endTime = std::clock();
            clock_t clockTicksTaken = endTime - startTime;
            double timeInSeconds = clockTicksTaken / (double) CLOCKS_PER_SEC;
            if (timeInSeconds<0.05){
              accel =0.;
            }
          }
          
          
          //here we check that the new velocity does not exceed the maximum velocity
          double new_vel = ref_vel + accel;
          ref_vel = std::min(new_vel, max_vel);
          
          
          closest_prec_exists = false;
          closest_prec_s = std::numeric_limits<double>::max();
          lcl = true;
          lcr = true;
          closest_left_exists = false;
          closest_right_exists = false;
          
          
          // generate a list of previosuly visited x and y coordinates and calculate the last yaw angle
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          //if there are two few points stored in previous_car_x and previous_car_y we need to extrapolate 
          if (previous_size < 2){
            
            double previous_car_x = car_x - cos(car_yaw);
            double previous_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(previous_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(previous_car_y);
            ptsy.push_back(car_y);
            
          }
          //otherwise we are just pushing the two last points to ptsx and ptsy
          else{
            
            ref_x = previous_path_x[previous_size-1];
            ref_y = previous_path_y[previous_size-1];
            
            double prev_ref_x = previous_path_x[previous_size-2];
            double prev_ref_y = previous_path_y[previous_size-2];
            ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);

            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);
            
          }
          
          // with the following code a spline gooing through the last position, the current position and 3 points far in front of our vehicle is generated 
          // first the 3 points are generated - they are 30, 60 and 90m in front of us and on our current lane
          vector<double> next_wp0 = getXY(car_s+30., (2.+4.*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60., (2.+4.*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90., (2.+4.*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsy.push_back(next_wp0[1]);
          
          ptsx.push_back(next_wp1[0]);
          ptsy.push_back(next_wp1[1]);
          
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp2[1]);
          
          //the points have to converted to ensure, that the spline starts in the origin of the vehicle
          double shift_x, shift_y;
          for (int i=0; i<ptsx.size(); i++){
            shift_x = ptsx[i] - ref_x;
            shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
            
          }

          //here the spline is initialized with the 5 points
          tk::spline sp;
          sp.set_points(ptsx, ptsy);
          
          //if the previously planned path is not fully exacuted, the remaining points will be pushed to the next x and y values
          for (int i = 0; i < previous_size; ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // target x and y are two points which are on the defined spline - the distance is the euclidean 
          double target_x = 30.;
          double target_y = sp(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          double x_add_on = 0.;
          
          //all points which are necessery to fill the next x and y values with 50 values are generated using the spline
          for (int i = 0; i < 50-previous_size; ++i) {
            
            // N is calculated as normalizer to ensure, that the vehicle roughly holds the reference velocity
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = sp(x_point);
            
            x_add_on = x_point;
            
            // as we transformed the coordinates to vehcile coordinates, we have to invert the conversion before 
            // we return the values, as the output is expected in global coordinates
            double x_ref = x_point;
            double y_ref = y_point;
            
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
            
            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
          }

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
