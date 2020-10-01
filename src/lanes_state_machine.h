#include <vector>
#include "helpers.h"
#include "spline.h"

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


int fastests_lane(double car_s, std::vector<std::vector<double>> sensor_fusion) {
  for (int i = 0; i < sensor_fusion.size(); i ++) {
    return -1; 
  }
}


/**
 * Possible states of the vehicle in the high way traffic mode
 */
enum class VehicleState {
  LCL,  // lane change left
  PLCL, // prepare lane change left
  KL,   // keep lane
  PLCR, // prepare lane change right
  LCR   // lane change right
};

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
    return 1;
  }
  else {
    return (target_speed - car_speed) / target_speed;  
  }
}

class Vehicle {
  
public:
  Vehicle(int lane, double speed, double target_velocity);
    
  std::vector<VehicleState> possible_next_states();
  
  void Accelerate();
  double ReferenceSpeed();
  void Update(double x, double y, double yaw, 
              double s, double d, 
              std::vector<double> previous_path_x, std::vector<double> previous_path_y,
              double ref_s, double ref_d);
  
  vector<vector<double> > Trajectory(int target_line, 
                                     vector<double> map_waypoints_s, 
                                    vector<double> map_waypoints_x, 
                                    vector<double> map_waypoints_y);
  
  
private:
  int lane; // final lane
  double car_x, car_y, car_yaw;
  double car_d, car_s;
  double speed;
  double reference_speed;
  double target_speed;
  double delta_t;
  double ref_x, ref_y, ref_yaw, ref_s, ref_d; // this refers to the last point in the current trajectory
  
  VehicleState state;
  
  vector<double> trajectory_x;
  vector<double> trajectory_y;
  
};

/**
 * Vehicle initialiser
 */
Vehicle::Vehicle(int lane, double speed, double target_speed)  {
  this->lane = lane;
  this->speed = speed;
  this->state = VehicleState::KL;
  this->reference_speed = speed;
  this->target_speed = target_speed;
  delta_t = .02;
}

/**
 * Vehicle update from the sensor fusion 
 * 
 * x - x position of the car
 * y - y position of the car
 * yaw - yaw of the car in radians
 * previous_path_x - x poins left from the previous trajectory
 * previous_path_y - y poins left from the previous trajectory
 */
void Vehicle::Update(double x, double y, double yaw, 
                     double s, double d, 
                     vector<double> previous_path_x, vector<double> previous_path_y,
                     double ref_s, double ref_d) {
  
  car_x = x;
  car_y = y;
  car_yaw = yaw;
  car_s = s;
  car_d = d;

  
  trajectory_x.clear();
  trajectory_y.clear();
  
  int prev_size = previous_path_x.size();
  
  
  for (int i = 0; i < prev_size; ++i) {
    trajectory_x.push_back(previous_path_x[i]);
    trajectory_y.push_back(previous_path_y[i]);
    
  }
  
  if (prev_size < 2) {
     std::cout << "Initialising reference position" << std::endl;
     ref_x = x;
     ref_y = y;
     ref_yaw = yaw;
     
     this->ref_s = s;
     this->ref_d = d;
  } 
  else {
    ref_x = previous_path_x[prev_size -1];
    ref_y = previous_path_y[prev_size -1];
    double ref_x_prev = previous_path_x[prev_size -2];
    double ref_y_prev = previous_path_y[prev_size -2];
    
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    
    this->ref_s = ref_s;
    this->ref_d = ref_d;
  }
 
  std::cout << "Car ref s: " << ref_s << std::endl;
  std::cout << "Car ref d: " << ref_d << std::endl;
  std::cout << "Reference x,y,yaw: " << ref_x << ", " << ref_y << ", " << ref_yaw << std::endl;
}


void Vehicle::Accelerate() {
  if(reference_speed < target_speed) {
    reference_speed += .224;
  }
}

double Vehicle::ReferenceSpeed() {
  return reference_speed;
}

/**
 * Possible states that vehicle can transit to from the current state
 */
std::vector<VehicleState> Vehicle::possible_next_states() {
  
  std::vector<VehicleState> states;
  states.push_back(VehicleState::KL);
  if (this->state == VehicleState::KL) {
    states.push_back(VehicleState::PLCL);
    states.push_back(VehicleState::PLCR);
  } 
  else if (this->state == VehicleState::PLCL) {
    states.push_back(VehicleState::PLCL);
    states.push_back(VehicleState::LCL);
  }
  else if (this->state == VehicleState::PLCR) {
    states.push_back(VehicleState::PLCR);
    states.push_back(VehicleState::LCR);
  }
  return states;
}

std::vector<std::vector<double> > Vehicle::Trajectory(int target_lane, 
                                                     vector<double> map_waypoints_s, 
                                                     vector<double> map_waypoints_x, 
                                                     vector<double> map_waypoints_y) {
  vector<double> sx, sy; // points that define spline
  
  
  double trajectory_step =  (delta_t * reference_speed / 2.24);
  
  sx.push_back(ref_x - trajectory_step  * cos(ref_yaw));
  sy.push_back(ref_y - trajectory_step  * sin(ref_yaw));
  
  sx.push_back(ref_x);
  sy.push_back(ref_y);
  
  // std::cout << "Target lane: " << lane << ", d value: "<< lane_number_to_frenet(lane) << std::endl;
  // std::cout << "Reference point yaw: " << ref_yaw << std::endl;
  // std::cout << "Reference point d: " << ref_d << std::endl;
  // std::cout << "Reference point s: " << ref_s << std::endl;
  
  vector<double> next_wp0 = getXY(
    ref_s + 10, 
    lane_number_to_frenet(target_lane), 
    map_waypoints_s, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  vector<double> next_wp1 = getXY(
    ref_s + 15, 
    lane_number_to_frenet(target_lane), 
    map_waypoints_s, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  vector<double> next_wp2 = getXY(
    ref_s + 25, 
    lane_number_to_frenet(target_lane), 
    map_waypoints_s, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  vector<double> next_wp3 = getXY(
    ref_s + 45, 
    lane_number_to_frenet(target_lane), 
    map_waypoints_s, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  
  sx.push_back(next_wp0[0]);
  sx.push_back(next_wp1[0]);
  sx.push_back(next_wp2[0]);
  sx.push_back(next_wp3[0]);
  
  sy.push_back(next_wp0[1]);
  sy.push_back(next_wp1[1]);
  sy.push_back(next_wp2[1]);
  sy.push_back(next_wp3[1]);
  
  std::cout << "Initial points in car coordinates" << std::endl;
  
  vector<double> vx, vy; // veichle coordinates
  for(int i = 0; i < sx.size(); i++) {
    // tranform points to car coordinates
    double shift_x = sx[i] - ref_x;
    double shift_y = sy[i] - ref_y;
    
    vx.push_back((shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw)));
    vy.push_back((shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw)));
    
    // std::cout << "Point x, y : (" << vx[i] << ", " << vy[i] << ")" << std::endl;
    
  }
  
  tk::spline s;
  
  // std::cout << "ptsx: " << vx[0] << std::endl;
  
  s.set_points(vx, vy);
  
  double target_x = 30;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  
  double x_add_on = 0;
  
  double N = target_dist / trajectory_step;
  
  
  for (int i = 1; i <= 50 - trajectory_x.size(); ++i) {
    
    
    double x = x_add_on + target_x / N;
    double y = s(x);
    
    x_add_on = x;
    
    x = x * cos(ref_yaw) - y * sin(ref_yaw);
    y = x * sin(ref_yaw) + y * cos(ref_yaw);
    
    x += ref_x;
    y += ref_y;
    
    
    trajectory_x.push_back(x);
    trajectory_y.push_back(y);
  }
  
  std::vector<std::vector<double> > trajectory;
  
  trajectory.push_back(trajectory_x);
  trajectory.push_back(trajectory_y);
  return trajectory;
}
