#include <vector>
#include "helpers.h"
#include "sensor_fusion.h"

using std::vector;

double lane_speed_cost(double target_speed, double next_car_speed) {
  if (next_car_speed >= target_speed) {
    return 0;
  } 
  else {
    return (target_speed - next_car_speed) / target_speed;
  }
}


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
double target_lane_safe_speed(double s, double d, double v, double speed_limit, double delta_t,
                              std::vector<std::vector<double> > sensor_fusion,
                              double forward_distance) {
  
  double sf_id, sf_s, sf_d, sf_x, sf_y, sf_xv, sf_yv, sf_v;
  
  double cost = 0.0;
  
  double s_min = -1;
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
    
    double s_distance = sf_s - s;
    double d_distance = fabs(sf_d - d);
   
    double break_power = 0.1 * forward_distance / s_distance;
    if (d_distance < 1.8) { // car is in the same lane
      if (s_distance > 0 && s_distance < forward_distance) { // car is ahead
        if (s_min > s_distance || s_min < 0) { // is closest
          s_min = s_distance;
          safe_speed = sf_v - break_power;
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
 * This cost penalises left and write lanes make car more likely to maneuver via central lane 
 */
double road_position_cost(double d) {
  return abs(d - 6) / 12;
}


/** 
 * calculates the cost associated with the colision with other cars
 * 
 * predictions is a 2d vector of cars with id, predicted_s and predicted_d
 * 
 * param trajectory represents proposed trajectory in Frenet coordinates. 
 *   we want to make sure that no other cars are predicted to be in 5m range
 *   behind or ahead of this trajectory for it to be safe
 */
double colision_cost(vector<vector <double> > trajectory, double target_d, vector<vector <double> > predictions) {
  double d, s;
  double cost, cost_front, cost_behind, max_cost = -1;
  vector <double> trajectory_s = trajectory[0];
  vector <double> trajectory_d = trajectory[1];
  
  double car_d_max = trajectory_d[trajectory_d.size() -1];
  double car_d_min = trajectory_d[0];
  double car_s_min = trajectory_s[0];
  double car_s_max = trajectory_s[trajectory_s.size() -1];
  

  for (int i = 0; i < predictions.size(); i++) {
    cost = 0;
    d = predictions[i][2];
    s = predictions[i][1];
    
   
    
    if (fabs(target_d - d) > 1.8) { // vehicle is not in the same lane as trajectory target
      continue;
    }
    
    double distance_front = fabs(car_s_max - s);
    double car_d_change = abs(car_d_min - target_d);
    if (car_d_change < 0.5) {
      cost = 0;
    }
    // TODO: here we need to know what is the distance that is covered by he existnig trajectory
    else if (car_s_min - 5 < s && car_s_max > s) { // car is inside the trajectory 
      cost = 1; 
    }
    else if (distance_front < 10) {
      cost = (10 - distance_front) / 10;
    }
    
    if (max_cost < cost || max_cost < 0) {
      max_cost = cost;
    }
  }
  return max_cost < 0 ? 0 : max_cost;
}


double target_lane_switch_cost(double car_d, double target_d) {
  double change = fabs(car_d - target_d);
  return  change >=4 ? 1 : change / 4;
}

vector<vector <double> > trajectory_derivative(vector<vector <double> > trajectory, double delta_t) {
  vector<vector <double> > derivative;
  
  vector<double> x = trajectory[0];
  vector<double> y = trajectory[1];
  vector<double> vx, vy;
  double delta, dx, dy;
  
  for (int i = 0; i < x.size() - 1; i++) {
    dx = x[i+1] - x[i];
    dy = y[i+1] - y[i];
    vx.push_back(dx / delta_t);
    vy.push_back(dy / delta_t);
  }
  
  derivative.push_back(vx);
  derivative.push_back(vy);
  return derivative;
}

double trajectory_acceleration(vector<vector <double> > trajectory, double delta_t) {

  vector<vector <double>> v = trajectory_derivative(trajectory, delta_t);
  vector<vector <double>> a = trajectory_derivative(v, delta_t);
  
  vector<double> ax = a[0];
  vector<double> ay = a[1];
  double abs_a, max_a = -1;
  double delta_v;
  for (int i = 0; i < ax.size() - 1; i++) {
    abs_a = sqrt(pow(ax[i], 2) + pow(ay[i], 2));
    if (abs_a > max_a || max_a < 0) {
      max_a = abs_a;
    }
  }
  return max_a;
}

double trajectory_acceleration_cost(double acceleration) {
  if (acceleration < 10) {
    return 0;
  } 
  else {
    return 1;
  }
}

double trajectory_jerk(vector<vector <double> > trajectory, double delta_t) {
  vector<vector <double>> v = trajectory_derivative(trajectory, delta_t);
  vector<vector <double>> a = trajectory_derivative(v, delta_t);
  vector<vector <double>> j = trajectory_derivative(a, delta_t);
  
  vector<double> jx = j[0];
  vector<double> jy = j[1];

  double abs_j, max_j = -1;
  for (int i = 0; i < jx.size(); i++) {
    abs_j = sqrt(pow(jx[i], 2) + pow(jy[i], 2));
   
    if (abs_j > max_j || max_j < 0) {
      max_j = abs_j;
    }
    
  }
  return max_j;
}

double trajectory_jerk_cost(double jerk) {
  if (jerk < 10) {
    return 0;
  } 
  else {
    return 1;
  }
}

double trajectory_cost(vector<vector <double> > trajectory, double target_d, double safe_speed, double target_speed, double delta_t, 
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
  
  vector<double> trajectory_s, trajectory_d;
  vector<double> frenet;
  for (int i = 0; i < trajectory_x.size() - 1; i++) {
    
    theta = atan2(
      trajectory_y[i + 1] - trajectory_y[i], 
      trajectory_x[i + 1] - trajectory_x[i]
    );
    
    frenet  = getFrenet(
      trajectory_x[i], 
      trajectory_y[i], 
      theta, 
      map_waypoints_x, 
      map_waypoints_y
    );
    
    trajectory_s.push_back(frenet[0]);
    trajectory_d.push_back(frenet[1]);
  }

  vector<vector <double>> trajectory_frenet;
  trajectory_frenet.push_back(trajectory_s);
  trajectory_frenet.push_back(trajectory_d);
 
  double predict_detla_t = trajectory_size * delta_t;
  
  vector<vector <double> > prediction = predict_sensor_fusion(sensor_fusion, predict_detla_t);
  double colision = colision_cost(
    trajectory_frenet,
    target_d,
    prediction
  );
  
  double trajectory_ref_s = trajectory_s[trajectory_s.size() -1];
  double trajectory_ref_d = trajectory_d[trajectory_d.size() -1];

  double speed_cost = car_speed_cost(target_speed, safe_speed);
  double acceleration = trajectory_acceleration(trajectory, delta_t);
  double acceleration_cost = trajectory_acceleration_cost(acceleration);
  double jerk = trajectory_jerk(trajectory, delta_t);
  double jerk_cost = trajectory_jerk_cost(jerk);
  double position_cost = road_position_cost(trajectory_ref_d);
  std::cout << "Experimental trajectory speed cost. " << std::endl
            << "  Ref s: " << trajectory_ref_s << "," << std::endl
            << "  Ref d: " << trajectory_ref_d << "," << std::endl
            << "  Target speed: " << target_speed << "," << std::endl
            << "  Safe speed: " << safe_speed << "," << std::endl
            << "  Colision cost: " << colision << std::endl
            << "  Speed cost: " << speed_cost << std::endl
            << "  Max Acceleration (cost)" << acceleration << "(" << acceleration_cost << ")" << std::endl
            << "  Max Jerk (cost)" << jerk << "(" << jerk_cost << ")" << std::endl;
  double lane_switch_cost = target_lane_switch_cost(frenet[1], target_d);
  return 
    100.0 * colision +
    20.0 * speed_cost + 
    // 1.0 * lane_switch_cost +
    2.0 * position_cost;
  
}