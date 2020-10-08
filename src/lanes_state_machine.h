#include <vector>
#include "helpers.h"
#include "spline.h"
#include "trajectory_cost.h"
#include "vehicle_states.h"
#include "map.h"

using std::vector;

// sensor_fusion
// A 2d vector of cars and then that car's 
// [
//   car's unique ID, 
//   car's x position in map coordinates, 
//   car's y position in map coordinates, 
//   car's x velocity in m/s, 
//   car's y velocity in m/s,
//   car's s position in frenet coordinates, 
//   car's d position in frenet coordinates.

/**
 * returns lane number for give d frenet coordinate
 */
int frenet_to_lane_number(double d) {
  if (d > 0.0 & d < 12.0) {
    return floor(d / 4.0);
  }
  else {
    return -1;
  }
}

/**
 * returns d frenet coordinate for the center of the lane
 * 
 * param l - number of the lane starting from zero
 */
double lane_number_to_frenet(int l) {
  if (l >=0 & l <=2) {
    return l * 4.0 + 2.0;
  }
  else {
    return -1.0;
  }
}

double frenet_distance(double car_s, double object_s, double max_s) {
  return std::fmod(object_s, max_s) - car_s;
}


class Vehicle {
  
public:
  Vehicle(int lane, double speed, double target_velocity,
          vector<double> map_waypoints_s, 
          vector<double> map_waypoints_x, 
          vector<double> map_waypoints_y);
  
  void Accelerate();
  
  void SwitchState(vector<vector <double> > sensor_fusion);
  double ReferenceSpeed();
  void Update(double x, double y, double yaw, double car_speed,
              double s, double d, 
              std::vector<double> previous_path_x, std::vector<double> previous_path_y,
              double ref_s, double ref_d);
  
  vector<vector<double> > Trajectory();
  
  double speed_update(double speed, double acceleration);
  
private:
  int lane; // final target lane
  int trajectory_size;
  double car_x, car_y, car_yaw;
  double car_d, car_s;
  double speed;
  double reference_speed; // speed at the end of the previous used trajectory
  double target_speed; // this is the speed that is safe for the current state (m/s)
  double speed_limit;  // current road speed limit (m/s)
  double delta_t;
  double ref_x, ref_y, ref_yaw, ref_s, ref_d; // this refers to the last point in the current trajectory
  
  double acceleration = 0;
  
  VehicleState state;
  
  vector<double> trajectory_x;
  vector<double> trajectory_y;
  
  // Map
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_x; 
  vector<double> map_waypoints_y;
  
  double acceleration_update(int sign);
    

  vector<vector<double> > compute_trajectory(int target_lane, double distance, int size, double safe_speed);
  
};

/**
 * Vehicle initialiser
 */
Vehicle::Vehicle(int lane, double speed, double target_speed,
                 vector<double> map_waypoints_s, 
                 vector<double> map_waypoints_x, 
                 vector<double> map_waypoints_y)  {
  this->lane = lane;
  this->speed = speed; // m/s
  
  this->state = VehicleState::KL;
  this->reference_speed = speed;
  this->speed_limit = target_speed;
  this->target_speed = target_speed;
  delta_t = .02;
  this->trajectory_size = 50;
  this->map_waypoints_s = map_waypoints_s;
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
}

/**
 * Vehicle update from the sensor fusion 
 * 
 * x - x position of the car
 * y - y position of the car
 * yaw - yaw of the car in radians
 * car_speed - spped of the car in mph
 * previous_path_x - x poins left from the previous trajectory
 * previous_path_y - y poins left from the previous trajectory
 */
void Vehicle::Update(double x, double y, double yaw, double car_speed,
                     double s, double d, 
                     vector<double> previous_path_x, vector<double> previous_path_y,
                     double ref_s, double ref_d) {
  
  car_x = x;
  car_y = y;
  car_yaw = yaw;
  car_s = s;
  car_d = d;
  
  
  // 1 mhph is 0.44704 m/s
  double ms_to_mph = 2.23694; // 1 m/s is 2.23694 mhph
  
  this->speed = car_speed / ms_to_mph;
  
  trajectory_x.clear();
  trajectory_y.clear();
  
  int prev_size = previous_path_x.size();
  
  
  for (int i = 0; i < prev_size; ++i) {
    trajectory_x.push_back(previous_path_x[i]);
    trajectory_y.push_back(previous_path_y[i]);
    
  }
  
  if (prev_size < 2) {
     // std::cout << "Initialising reference position" << std::endl;
     ref_x = x;
     ref_y = y;
     ref_yaw = yaw;
     
     this->ref_s = s;
     this->ref_d = d;
     trajectory_x.push_back(ref_x - 1  * cos(ref_yaw));
     trajectory_y.push_back(ref_y - 1  * sin(ref_yaw));
     
  } 
  else {
    ref_x = previous_path_x[prev_size -1];
    ref_y = previous_path_y[prev_size -1];
    double ref_x_prev = previous_path_x[prev_size -2];
    double ref_y_prev = previous_path_y[prev_size -2];
    
    
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
      
    this->ref_s = ref_s;
    this->ref_d = ref_d;
    double dx = ref_x - ref_x_prev;
    double dy = ref_y - ref_y_prev;
    this->reference_speed = sqrt(dx * dx + dy * dy) / delta_t;
  }
 
  std::cout << "Car Update" << std::endl
            << "  ref s: " << ref_s << ", car s: " << car_s << std::endl
            << "  ref d: " << ref_d << ", car d: " << car_d << std::endl
            << "  ref x, y, yaw: " << ref_x << ", " << ref_y << ", " << ref_yaw << std::endl
            << "  car speed: " << car_speed << std::endl 
            << "  ref speed: " << reference_speed << std::endl;
}


void Vehicle::Accelerate() {
  return;
}

void Vehicle::SwitchState(vector<vector <double> > sensor_fusion) {
  // std::cout << "Switching veichle state" << std::endl;

  vector<vector<double> > trajectory;
  VehicleState optimal_state;
  vector<double> ref_point, ref_point_prev;
  double cost, trajectory_cost_min = -1;

  
  if (this->speed < 10) {
    this->state = VehicleState::KL;
    return;
  }
  
  vector<VehicleState> states = possible_next_states(this->state);
  VehicleState next_state;
  for (int i = 0; i < states.size(); i++) {
    next_state = states[i];
    int final_lane = get_final_lane(lane, next_state);
    
   // std::cout << "Experimental target lane: " << final_lane 
    //          << ", for state: " << label_vehicle_state(next_state) << std::endl;
    
    
    double safe_speed = target_lane_safe_speed(
      ref_s, final_lane * 4 + 2, reference_speed, delta_t, 
      sensor_fusion,
      30
    );
    
    vector<vector<double> > trajectory = this->compute_trajectory(final_lane, 50, 100, safe_speed);

    cost = trajectory_cost(trajectory, this->ref_d, 
                                      this->speed_limit, 
                                      delta_t, 
                                      sensor_fusion,     
                                      this->map_waypoints_x, 
                                      this->map_waypoints_y);
    
    if (cost < trajectory_cost_min || trajectory_cost_min < 0) {
      trajectory_cost_min = cost;
      optimal_state = next_state;
    }
  }
    
  // std::cout << "New optimal state: " << label_vehicle_state(optimal_state) << std::endl;
    
    
  this->state = optimal_state;
  this->lane = get_final_lane(this->lane, this->state);
  
  this->target_speed = target_lane_safe_speed(
    car_s, 
    this->lane * 4 + 2, 
    reference_speed, 
    delta_t, 
    sensor_fusion,
    15
  );
  
  // std::cout << "New target lane: " << this->lane << std::endl;
  // std::cout << "Trajectory min cost: " << trajectory_cost_min << std::endl;
  
}

double Vehicle::ReferenceSpeed() {
  return reference_speed;
}


vector<vector<double> > Vehicle::compute_trajectory(int target_lane, double distance, int size, double safe_speed) {
  vector<double> sx, sy; // points that define spline
  
  if (trajectory_x.size() > 1) {
    sx.push_back(trajectory_x[trajectory_x.size() - 2]);
    sy.push_back(trajectory_y[trajectory_x.size() - 2]);
  }
  
  sx.push_back(ref_x);
  sy.push_back(ref_y);
  
  // std::cout << "Generating Trajectory" << std::endl
  //           << "      target lane: "<< target_lane << std::endl
  //           << "      d value: "<< lane_number_to_frenet(target_lane) << std::endl;
  // 
  // std::cout << "    reference points "    << std::endl
  //           << "      s: "   << ref_s << std::endl
  //           << "      d: "   << ref_d << std::endl
  //           << "      x: "   << ref_x << std::endl
  //           << "      y: "   << ref_y << std::endl
  //           << "      yaw: " << ref_yaw << std::endl;
  
  vector<double> next_wp0 = getXY(
    ref_s + 20, 
    (lane_number_to_frenet(target_lane) + ref_d) / 2, 
    map_waypoints_s, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  vector<double> next_wp1 = getXY(
    ref_s + 40, 
    (lane_number_to_frenet(target_lane) * 2 + ref_d) / 3, 
    map_waypoints_s, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  vector<double> next_wp2 = getXY(
    ref_s + 60, 
    (lane_number_to_frenet(target_lane)), 
    map_waypoints_s, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  vector<double> next_wp3 = getXY(
    ref_s + 75, 
    lane_number_to_frenet(target_lane), 
    map_waypoints_s, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  vector<double> next_wp4 = getXY(
    ref_s + 90, 
    (lane_number_to_frenet(target_lane)), 
    map_waypoints_s, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  
  sx.push_back(next_wp0[0]);
  sx.push_back(next_wp1[0]);
  sx.push_back(next_wp2[0]);
  sx.push_back(next_wp3[0]);
  sx.push_back(next_wp4[0]);
  
  sy.push_back(next_wp0[1]);
  sy.push_back(next_wp1[1]);
  sy.push_back(next_wp2[1]);
  sy.push_back(next_wp3[1]);
  sy.push_back(next_wp4[1]);
  
  vector<double> vx, vy; // veichle coordinates
  for(int i = 0; i < sx.size(); i++) {
    // tranform points to car coordinates
    
    vector<double> local = LocalCoordinates(ref_x, ref_y, ref_yaw, sx[i], sy[i]);
 
    vx.push_back(local[0]);
    vy.push_back(local[1]);
    
    // if (ref_s > 1200) {
    //   std::cout << "Points in car coordinates x, y : (" << vx[i] << ", " << vy[i] << ")" << std::endl;
    // }
   
  }
  
  tk::spline s;
  
  // std::cout << "ptsx: " << vx[0] << std::endl;
  
  s.set_points(vx, vy);
  
  // std::cout << "Trajectory end point distance x: " << distance << std::endl;
  // std::cout << "Trajectory end point distance y: " << s(distance) << std::endl;
 
  double target_x = distance;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  
  double x_add_on = 0;
  double point_speed = reference_speed;
  
  
  // copy current trajectory, so it is not
  // changed by experimental calculations
  vector<double> new_x = trajectory_x;
  vector<double> new_y = trajectory_y;
  
  for (int i = 0; i < size - trajectory_x.size(); i ++) {
    
    point_speed = speed_update(point_speed, safe_speed);
    //std::cout << "  point speed: " << point_speed;
    
    x_add_on += (delta_t * point_speed);
    double x = x_add_on;
    double y = s(x);
    
    
    // if (ref_s > 1200) {
    //   std::cout << "Releative position: "
    //             << "(" << x << ", " << y << ")" << " point speed: "
    //             << point_speed << ", distance: "
    //             << distance  << std::endl;
    // }
    // 
    
    vector<double> g;
    g = GlobalCoordinates(ref_x, ref_y, ref_yaw, x, y);
  
    new_x.push_back(g[0]);
    new_y.push_back(g[1]);
    
  }
  
  std::vector<std::vector<double> > trajectory;
  
  trajectory.push_back(new_x);
  trajectory.push_back(new_y);
  // std::cout << "New trajectory xy tail" << std::endl 
  //          << "  x : " << new_x[new_x.size() -1 ] << std::endl 
  //           << "  y : " << new_y[new_y.size() -1 ] << std::endl;
  
  return trajectory;
}

double Vehicle::acceleration_update(int sign) {
  if (acceleration < 5 && sign > 0) {
    acceleration += .2;
  }
  else if(acceleration > -5 && sign < 0) {
    acceleration -= .2;
  }
}

double Vehicle::speed_update(double speed, double safe_speed) {

  if(abs(speed - safe_speed) <= 0.5) {
    return speed;
  }
  if (speed > safe_speed) {
    acceleration_update(-1);
  } 
  if (speed < safe_speed) {
    acceleration_update(1);
  }
  return speed + acceleration * 0.02;
}

std::vector<std::vector<double> > Vehicle::Trajectory() {

  return this->compute_trajectory(lane, 30.0, this->trajectory_size, this->target_speed);
}
