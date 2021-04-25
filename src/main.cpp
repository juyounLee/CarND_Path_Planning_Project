#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include <map>
#include "spline.h"
#include <sstream>
#include <utility>

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
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  string file = "../data/highway_map.csv";
  string line;
  std::ifstream in_map_(file);
  if (in_map_.is_open())
  {
	while (getline(in_map_,line))
    {
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
    in_map_.close();
  }

  else std::cout << "Unable to open file"<<std::endl; 
  // start in lane 1
  int lane = 1;
  double ref_vel = 0; // mph
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel]
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
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          int prev_size = previous_path_x.size();
          
          // assume car is at end position of previous path
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }
          
          //  for guiding my car to change lane and keep a desired velocity.
          bool car_front = false;
          bool car_left = false;
          bool car_right = false;         
          
          // go through all cars
          for(int i =0;i<sensor_fusion.size();i++)
          {
            //get the other car's s , d , present lane 
            double o_s = sensor_fusion[i][5];
            double o_d = sensor_fusion[i][6];
            double o_vx = sensor_fusion[i][3];
            double o_vy = sensor_fusion[i][4];
            double o_speed = sqrt(o_vx*o_vx + o_vy*o_vy);
            int o_lane  = ((int)floor(o_d/4));
             if(o_lane < 0) {
               continue;
             }
            // Estimate the other car s when using previous points.
            o_s+=((double)o_speed * 0.02 * prev_size); 
            
             // ther other car in my lane.                        
            if(o_lane == lane)
            {
              // check whether the other front car is too close
              if ((o_s - car_s > 0) and (o_s - car_s <=30.0))    
              {              
                  car_front = true;          
              }
            }
            else
            {
              //if the other car is in the left lane
              if(o_lane  == lane-1)
              {                
                // Secure a safe distance for changing left line
                if(car_s-30.0<o_s and car_s+30.0>o_s)
                {
                  //change the left lane
                	car_left = true;
                }
                
              //if the other car is in the right lane  
              }else if(o_lane == lane+1)
              {
                
                // Secure a safe distance for changing right line
                if(car_s-30.0<o_s && car_s+30.0>o_s)
                {
                  //change the right lane
                	car_right = true;
                }
              }
            }            
          }

          //  take safty action on above boole statements
          
          // there is the other car in my lane ahead of me
          if(car_front)
          {
            // and there is no car in left change. And then, switch the left lane
            if(!car_left && lane>0)
            {
              lane-=1;              
            }
            // and there is no car in right change. And then, switch the right lane
            else if(!car_right && lane < 2)
            {
              lane+=1;
            } 
                
            // change lane slowly"
            ref_vel-=0.4; 
          }
          // spped up, if my car is slow.
          else if(ref_vel < 49.5)
          {
            ref_vel+=0.35;             
          }
          // Generate the best trajectory (keep smooth the trajectory).
          
          // create a list widely spaced (x, y) waypoints
          vector<double> point_x;
          vector<double> point_y;
          
          double ref_yaw = deg2rad(car_yaw);
          double ref_x = car_x;
          double ref_y = car_y;
          
          //including two previous points for smooth tansition
          
          // use the last two points.
          if(prev_size >= 2)
          {           
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];

            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            point_x.push_back(ref_x_prev);
            point_x.push_back(ref_x);

            point_y.push_back(ref_y_prev);
            point_y.push_back(ref_y);
          }
          
           // if previous size is almost empty, use the car as starting ref
          else
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            point_x.push_back(prev_car_x);
            point_x.push_back(car_x);

            point_y.push_back(prev_car_y);
            point_y.push_back(car_y);             
          }

          // In Frenet add evenly 30m points ahead of the staring referenc
          for(int i =30;i<=90;i+=30)
          {
            vector<double> wp = getXY(car_s+i, 2+4*lane, map_waypoints_s,map_waypoints_x,map_waypoints_y);
          point_x.push_back(wp[0]);

          point_y.push_back(wp[1]);
          }
          
          // Shift the points to local car coordinates make sure that the car or that
          // last point of previous path at zero and its angle at zero degree.
           
          for(int i =0;i<point_x.size();i++)
           {
			 double transl_x = point_x[i] - ref_x;
             double transl_y = point_y[i] - ref_y;
             point_x[i] = transl_x*cos(0-ref_yaw) - transl_y*sin(0-ref_yaw);
             point_y[i] = transl_x*sin(0-ref_yaw) + transl_y*cos(0-ref_yaw);
           }
          
          // Create a spline
          tk::spline spl;
          // Set (x, y) points to the spline
          spl.set_points(point_x,point_y);
          
          
          // Start with all of the previous path point from last time        
          for(int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
                      
          // Calculate with which distance to interpolate along the spline 
          // so that the car will travel at a desired reference velocity
          // path will extend 30m ahead in x direction

           double x = 30;
           double y = spl(x);
           double dist = sqrt(x*x+y*y); //becuase we have shifted the points to make the car position as origin 
           double dummy_x = 0;
           double N = dist / (0.02 * ref_vel/2.24);
           for(int i=0;i<50 -prev_size;i++)
           {
            double x_point = dummy_x + x / N;
            double y_point = spl(x_point);

            dummy_x = x_point;

            double x_dum = x_point;
            double y_dum = y_point;

            // Rotating back to normal after rotating it earlier.
            x_point = x_dum * cos(ref_yaw) - y_dum * sin(ref_yaw);
            y_point = x_dum * sin(ref_yaw) + y_dum * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;   
             next_x_vals.push_back(x_point);
             next_y_vals.push_back(y_point);
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