#include <vector>
#include "helpers.h"
#include "sensor_fusion.h"

using std::vector;

double car_speed_cost(double target_speed, double car_speed) {
  if (car_speed > target_speed) {
    return 0;
  }
  else {
    return (target_speed - car_speed)  / target_speed;  
  }
}

/**
 * Gets the speed of the closest hevicle for the target d
 * 
 * param v - speed of the car in m/s
 */
double target_lane_safe_speed(double s, double d, double v, double delta_t,
                              std::vector<std::vector<double> > sensor_fusion,
                              double forward_distance) {
  
  double sf_id, sf_s, sf_d, sf_x, sf_y, sf_xv, sf_yv, sf_v;
  
  double cost = 0.0;
  
  double s_min = -1;
  double speed_limit = 47.5 / 2.23694;
  double safe_speed = speed_limit;
  
  for (int i = 0; i < sensor_fusion.size(); i ++) {
    sf_id = sensor_fusion[i][0];
    sf_x = sensor_fusion[i][1];
    sf_y = sensor_fusion[i][2];
    sf_xv = sensor_fusion[i][3]; // speed in m/s
    sf_yv = sensor_fusion[i][4]; // speed in m/s
    sf_s = sensor_fusion[i][5];
    sf_d = sensor_fusion[i][6];
    
    
    sf_v = sqrt(sf_xv * sf_xv + sf_yv * sf_yv);
    
    double sf_s_predicted = sf_s + sf_v * delta_t;
    
    double s_distance = sf_s_predicted - s;
    double d_distance = abs(sf_d - d);
   
    
    if (d_distance < 1.8) { // car is in the same lane
      if (s_distance > 0 && s_distance < forward_distance) { // car is ahead
        if (s_min > s_distance || s_min < 0) { // is closest
          s_min = s_distance;
          safe_speed = sf_v;
        }
      }
    }
    
  }
  
  if (safe_speed > speed_limit) {
    return speed_limit;
  }
  
  return safe_speed;
  
}


/** 
 * calculates the cost associated with the colision with other cars
 * 
 * predictions is a 2d vector of cars with id, predicted_s and predicted_d
 * 
 * param car_s_min, car_s_max represent begining and end of trajectory. 
 *   we want to make sure that no other cars are predicted to be in 5m range
 *   behind or ahead of this trajectory for it to be safe
 */
double colision_cost(double car_s_min, double car_s_max, double car_d, vector<vector <double> > predictions) {
  double d, s;
  double cost, max_cost = -1;
  
  
  for (int i = 0; i < predictions.size(); i++) {
    cost = 0;
    d = predictions[i][2];
    s = predictions[i][1];
    
    if (abs(car_d - d) > 3) { // vehicle is ahead of the car
      continue;
    }
    
    double distance = abs(car_s_max - s);
    
    if (car_s_min - 5 < s && car_s_max + 5 > s) { // car is inside the trajectory 
      cost = 1; 
    }
   
    else if (distance < 10) {
      cost = 1 - (distance - 5) / 5;
    } 
    else { // distance is more than 15m
      cost = 0; 
    }
    
    if (max_cost < cost || max_cost < 0) {
      max_cost = cost;
    }
  }
  return max_cost < 0 ? 0 : max_cost;
}


double target_lane_switch_cost(double car_d, double target_d) {
  double change = abs(car_d - target_d);
  return  change >=4 ? 1 : change / 4;
}

double trajectory_cost(vector<vector <double> > trajectory, double ref_d, double target_speed, double delta_t, 
                       vector<vector<double> > sensor_fusion,
                       vector<double> map_waypoints_x,
                       vector<double> map_waypoints_y) {
  
  int trajectory_size = 50;
  
  vector<double> trajectory_x = trajectory[0];
  vector<double> trajectory_y = trajectory[1];
  
  double ref_point_x = trajectory_x[trajectory_x.size() - 1];
  double ref_point_y = trajectory_y[trajectory_y.size() - 1];
  
  double ref_point_prev_x = trajectory_x[trajectory_x.size() - 2];
  double ref_point_prev_y = trajectory_y[trajectory_y.size() - 2];
  
  double theta = atan2(
    ref_point_y - ref_point_prev_y, 
    ref_point_x - ref_point_prev_x
  );
  
  // std::cout << "Experimental trajectory coordinates: x, y, theta: " 
  //   << ref_point_x << ", " 
  //   << ref_point_y << ", " 
  //   << theta << std::endl;
  
  
  vector<double> frenet  = getFrenet(
    ref_point_x, ref_point_y, theta, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  double trajectory_ref_s = frenet[0];
  double trajectory_ref_d = frenet[1];
  
 
  double predict_detla_t = trajectory_size * delta_t;
  
  vector<vector <double> > prediction = predict_sensor_fusion(sensor_fusion, predict_detla_t);
  double colision = colision_cost(trajectory_ref_s - predict_detla_t * target_speed, trajectory_ref_s, trajectory_ref_d, prediction);
  
  // std::cout << "Experimental colision cost. " << std::endl 
  //           << "  Ref s: " << trajectory_ref_s << "," << std::endl
  //           << "  Ref d: " << trajectory_ref_d << "," << std::endl
  //           << "  Colision cost: " << colision << std::endl;
 
  
  // for (int i = 0; i < prediction.size(); i++) {
  //   std::cout << "    car " << i << ": " << prediction[i][1] << "," << prediction[i][2] << std::endl;
  // }
  
  double safe_speed = target_lane_safe_speed(
    trajectory_ref_s, 
    trajectory_ref_d, 
    target_speed, predict_detla_t, sensor_fusion, 50
  );
  
  double speed_cost = car_speed_cost(target_speed, safe_speed);
  
  // std::cout << "Experimental trajectory speed cost. " << std::endl 
  //           << "  Target speed: " << target_speed << "," << std::endl
  //           << "  Safe speed: " << safe_speed << "," << std::endl
  //           << "  Speed cost: " << speed_cost << std::endl;
  double lane_switch_cost = target_lane_switch_cost(frenet[1], ref_d);
  return 20 * colision   + 5 * speed_cost + lane_switch_cost * 0.1;
  
}