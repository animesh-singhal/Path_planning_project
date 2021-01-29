#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <iostream>

// for convenience
using std::cout;
using std::endl;
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

int behaviour_planner( vector<vector<double>> sensor_fusion, int prev_size, int ego_lane, double ego_prev_path_end_s){
  
  vector <vector<string>>options = { {"right"}, {"left", "right"}, {"left"} };
  //options[d] will give possible directions that the car can go to
  
  vector <int> lane_val;
  vector <double> cost_lane;
  
  for (int i=0; i<options[ego_lane].size(); i++){
   // iterating over each direction
    
   string direction = options[ego_lane][i];
   int new_lane;    //resulting lane when the car moves in that direction
    
   if(direction=="left"){
     new_lane = ego_lane-1;
   }
   else if (direction=="right") {
     new_lane = ego_lane+1;
   }
    
   double del_s_back = 6000;   // Difference in s value of ego vehicle and vehicle behind 
   double del_s_front = 6000;  // Difference in s value of ego vehicle and vehicle in front  
    
   bool possible = true; 
    
   for(int i =0; i< sensor_fusion.size(); i++){
     
     float d = sensor_fusion[i][6];
     if(d < (2+4*new_lane+2) && d > (2+4*new_lane-2) ){
     
       double vx = sensor_fusion[i][3];
       double vy = sensor_fusion[i][4];
       double check_speed = sqrt( pow(vx,2) + pow(vy,2) );
       double check_car_s = sensor_fusion[i][5];

       //Assuming that all cars will move with constant velocity in their lane till the time our ego vehicle comes previous points
       check_car_s+=((double)prev_size*0.02*check_speed); //if using previous points can preoject s value out
       // check s values greater than mine and s gap 

       if((check_car_s > ego_prev_path_end_s-20) && (check_car_s < ego_prev_path_end_s+20)){
         bool possible = false; 
         break;
         
       }
       
       double difference_s = check_car_s - ego_prev_path_end_s;

       if ( (difference_s>0) && (difference_s < del_s_front) ){
         del_s_front = difference_s;
       }
       else if ( (difference_s<0) && (fabs(difference_s) < del_s_back) )  {
         del_s_back = fabs(difference_s);
       }

     } // if condition closes for vehicle 
   } //loop closes which iterates over all vehicles

   if (possible){
     lane_val.push_back(new_lane);
     cost_lane.push_back( (exp(-1*(del_s_front - 20))) + (exp(-1*(del_s_back - 10))) );
   }
    
  }//loop closes for each direction
  
  //cout<<"options[ego_lane] "<<options[ego_lane]<<endl;
  //cout<<"lane_val.size() "<<lane_val.size()<<endl;
  
  if (lane_val.size() == 0) 
    { return ego_lane; }

  else if (lane_val.size() == 1)
    { return lane_val[0]; }

  else{
   int minElementIndex = std::min_element(cost_lane.begin(), cost_lane.end()) - cost_lane.begin(); 
   return lane_val[minElementIndex];
   }
  
}



#endif  // HELPERS_H