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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;


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
  
  // car starts in lane 1 (lane numbers: 0,1,2. So middle lane would be 1)
  int lane = 1; 

  double ref_vel = 0; //mph
  
 
   
  //Note: After declaring these variables, we've added them in h.onMessage (notice &ref_vel, &lane)
  
  
  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
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

          // ----DEFINING VARIABLE
          
          //Last path the car was following. Simulator will tell us what the previous path was. 
          int prev_size = previous_path_x.size();
          
          bool lane_change_last_time = false;
          
          // ----END OF DECLARATIONS  
          
          
          json msgJson;

          // Vectors that contains actual (x,y) coordinates in map frame which we'll send to simulator
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          
          //////////////////////////////////////////////// START OF CODE TO AVOID COLLISION 

          double car_s_present = car_s; //will be used to feed in function as parameter
          
          if(prev_size > 0)
          {
            car_s = end_path_s; 
          }
          
          bool too_close = false; 
          
          //find ref_vel to use 
          for(int i =0; i< sensor_fusion.size(); i++)
          {
            //car is in my lane
            float d = sensor_fusion[i][6];
            if(d < (2+4*lane+2) && d > (2+4*lane-2) )
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              check_car_s+=((double)prev_size*0.02*check_speed); //if using previous points can preoject s value out
              // check s values greater than mine and s gap 
              if((check_car_s > car_s) && ((check_car_s-car_s) <30) )
              {
                //Bracket A
                
                // Do some logic here, lower reference velocity so we don't crash into the car infront of us 
                // could also flag to try to change lanes
                //ref_vel = 29.5; //mph
                too_close = true; 
                
                cout<<"Hi we're in the collision test loop\n";

                int new_lane = -1; //randomly intiializing new lane (as 0 corresponds to left lane)

                // behaviour_planner function is defined in helper.h (at the bottom) 
                if (lane_change_last_time == false){
                new_lane = behaviour_planner( sensor_fusion, prev_size, lane, car_s_present, end_path_s);
                
                  if (new_lane != lane){
                   lane = new_lane;
                   lane_change_last_time = true;   
                  }

                }
                
                cout<<"Lane's value"<<lane<<endl<<endl;
                
              }
              
            }// End of Bracket A
            
            else {
            //Bracket B opens up  
         
            // Even if any vehicle is not too close to ego vehicle, it would be nice to come to the middle lane

            //If you're NOT in the middle lane, try to see if you can move to the middle lane:
              
              if(lane != 1){  //Bracket C opens
                
                int new_lane = -1; //randomly intiializing new lane (as 0 corresponds to left lane)
                
                if (lane_change_last_time == false){
                new_lane = behaviour_planner( sensor_fusion, prev_size, lane, car_s_present, end_path_s);
                
                  if (new_lane != lane){
                   lane = new_lane;
                   lane_change_last_time = true;   
                  }

                }
                
              }  //Bracket C closes
             
              
            } //Bracket B closes
            
          }
          
		  // If ego is too close to vhehicle in front & is not changing lane
          bool apply_brake = emergency_brakes( sensor_fusion, lane, car_s_present);
          
          if( (too_close && (lane_change_last_time == false)) || apply_brake )
          {
            ref_vel -= .224;
            //cout<<"ref_vel A"<<ref_vel<<endl;
          }
          else if(ref_vel < 49.5)
          {
            ref_vel += .224;
            //cout<<"ref_vel B"<<ref_vel<<endl;
          }
          
            
          //////////////////////////////////////////////// END OF CODE TO AVOID COLLISION 
          
          
          //////////////////////////////////////////////// START OF CODE TO MOVE IN CAR'S LANE
          
          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // Later we will interpolate these waypoints with a spline an fill it in with more points that control speed
          
          vector<double> ptsx;
          vector<double> ptsy;
          
          // Keep a track of reference state  
          // (either going to be where the car is at OR going to be at this previous path's endpoint)
          // reference x, y, yaw states
          // either we will reference the starting point as where the car is or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
                   
          // The previous path would either be empty or will have some points that we can take advantage of
          
          // if previous size is almost empty(we're just starting out), use the car as starting reference 
          if (prev_size<2) 
          {

            //Use two points that make the path tangent to the angle of the car
            double prev_car_x = car_x - cos(car_yaw);  //going backward in time basis angle
            double prev_car_y = car_y - sin(car_yaw);	
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } 
          // use the previous path's end point as starting reference
          else
          {
            
            //Redefining reference state as previous path end point 
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            
            // Use two points that make the path tangent to the previous path's end point 
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
            
          }
               
          // In Frenet add evently 30m spaced points ahead of starting reference
          // Instead of looking at one distance increment, we're looking at 30,60,90
          // Earlier we created 50 points, now we're creating just 3 of them
          
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
             
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // So far the vector of points have 2 previous points, and location of car at 30,60,90m (5 points)
          
          
          //Now we want to shift the frame of reference to car's reference coordinates (ref_x, ref_y, ref_yaw)
          for (int i=0; i< ptsx.size(); i++)
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            
            // NOW WE'LL MAKE THE SHIFT IN ROTATION:
            // Have a look  at homogeneous transformation to understand the code properly
            // Above, we have taken origin to the car coordinates
            // Now we want the align the x and y axis according to the car's yaw angle
            // Check homogeneous notes, we'll have to take (-angle) because points are expressed in rotated frame and 
			// we have to get them into rotate frame. Jis ke reference mein chahiye, usko -theta rotate karna padega na
            ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            
          }
          
          //create a spline 
          tk::spline s;
          
          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);
          
          
          // Start with all of the previous path points from the last time
          for (int i=0; i< previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
            
          }
          
          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
                    
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          
          double x_add_on = 0;
          
          // Fill up the rest of our path planner after filling it with previous points,
          // here we will always output 50 points
          for (int i = 1; i <= 50-previous_path_x.size(); i++)
          {
            // Refer notes (https://tinyurl.com/yxkfvqzm) to find how we find N
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point; 
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // rotate back to normal after rotating it earlier
            // This is proper homogeneous transformation:
            x_point = ref_x + (x_ref *cos(ref_yaw)) - (y_ref *sin(ref_yaw));
            y_point = ref_y + (x_ref *sin(ref_yaw)) + (y_ref *cos(ref_yaw));
          
			next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);       
            
          }

          //////////////////////////////////////////////// START OF CODE TO MOVE IN CAR'S LANE
		  
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