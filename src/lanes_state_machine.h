#include <vector>
#include "helpers.h"
#include "spline.h"
#include "trajectory_cost.h"
#include "vehicle_states.h"

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





double lane_speed_cost(double target_speed, double next_car_speed) {
  if (next_car_speed >= target_speed) {
    return 0;
  } 
  else {
    return (target_speed - next_car_speed) / target_speed;
  }
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
  
  
private:
  int lane; // final target lane
  int trajectory_size;
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
  
  // Map
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_x; 
  vector<double> map_waypoints_y;
  
  
  vector<vector<double> > compute_trajectory(int target_lane, double distance, int size);
  
};

/**
 * Vehicle initialiser
 */
Vehicle::Vehicle(int lane, double speed, double target_speed,
                 vector<double> map_waypoints_s, 
                 vector<double> map_waypoints_x, 
                 vector<double> map_waypoints_y)  {
  this->lane = lane;
  this->speed = speed;
  this->state = VehicleState::KL;
  this->reference_speed = speed;
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
  this->speed = car_speed;
  
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

void Vehicle::SwitchState(vector<vector <double> > sensor_fusion) {
  std::cout << "Switching veichle state" << std::endl;

  vector<vector<double> > trajectory;
  VehicleState optimal_state;
  vector<double> ref_point, ref_point_prev;
  double cost, trajectory_cost_min = -1;

  
  if (this->speed < 30) {
    this->state = VehicleState::KL;
    return;
  }
  
  vector<VehicleState> states = possible_next_states(this->state);
  VehicleState next_state;
  for (int i = 0; i < states.size(); i++) {
    next_state = states[i];
    int final_lane = get_final_lane(lane, next_state);
    
    std::cout << "Experimental target lane: " << final_lane << ", for state: " << label_vehicle_state(next_state) << std::endl;
    vector<vector<double> > trajectory = this->compute_trajectory(final_lane, 50, 100);

    double dt = trajectory_x.size() * this->delta_t;
    cost = trajectory_cost(trajectory, this->ref_d, 
                                      target_speed, dt, sensor_fusion,     
                                      this->map_waypoints_x, 
                                      this->map_waypoints_y);
    
    if (cost < trajectory_cost_min || trajectory_cost_min < 0) {
      trajectory_cost_min = cost;
      optimal_state = next_state;
    }
  }
    
  std::cout << "New optimal state: " << label_vehicle_state(optimal_state) << std::endl;
    
    
  this->state = optimal_state;
  this->lane = get_final_lane(this->lane, this->state);
  
  std::cout << "New target lane: " << this->lane << std::endl;
  std::cout << "Trajectory min cost: " << trajectory_cost_min << std::endl;
  
}

double Vehicle::ReferenceSpeed() {
  return reference_speed;
}



vector<vector<double> > Vehicle::compute_trajectory(int target_lane, double distance, int size) {
  vector<double> sx, sy; // points that define spline
  
  
  double trajectory_step =  (delta_t * reference_speed / 2.24);
  
  sx.push_back(ref_x - trajectory_step  * cos(ref_yaw));
  sy.push_back(ref_y - trajectory_step  * sin(ref_yaw));
  
  sx.push_back(ref_x);
  sy.push_back(ref_y);
  
  std::cout << "Generating Trajectory for Target Lane: " << target_lane << ", d value: "<< lane_number_to_frenet(target_lane) << std::endl;
  // std::cout << "Reference point yaw: " << ref_yaw << std::endl;
  std::cout << "Reference point d: " << ref_d << std::endl;
  std::cout << "Reference point s: " << ref_s << std::endl;
  
  vector<double> next_wp0 = getXY(
    ref_s + 5, 
    (lane_number_to_frenet(target_lane)) , 
    map_waypoints_s, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  vector<double> next_wp1 = getXY(
    ref_s + 10, 
    (lane_number_to_frenet(target_lane)), 
    map_waypoints_s, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  vector<double> next_wp2 = getXY(
    ref_s + 20, 
    (lane_number_to_frenet(target_lane)), 
    map_waypoints_s, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  vector<double> next_wp3 = getXY(
    ref_s + 40, 
    lane_number_to_frenet(target_lane), 
    map_waypoints_s, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  vector<double> next_wp4 = getXY(
    ref_s + 60, 
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
    double shift_x = sx[i] - ref_x;
    double shift_y = sy[i] - ref_y;
    
    vx.push_back((shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw)));
    vy.push_back((shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw)));
    
    // std::cout << "Point in car coordinates x, y : (" << vx[i] << ", " << vy[i] << ")" << std::endl;
  }
  
  tk::spline s;
  
  // std::cout << "ptsx: " << vx[0] << std::endl;
  
  s.set_points(vx, vy);
  
  double target_x = distance;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  
  double x_add_on = 0;
  
  double N = target_dist / trajectory_step;
  
  // copy current trajectory, so it is not
  // changed by experimental calculations
  vector<double> new_x = trajectory_x;
  vector<double> new_y = trajectory_y;
  
  for (int i = 1; i <= size - new_x.size(); ++i) {
    double x = x_add_on + target_x / N;
    double y = s(x);
    
    x_add_on = x;
    
    x = x * cos(ref_yaw) - y * sin(ref_yaw);
    y = x * sin(ref_yaw) + y * cos(ref_yaw);
    
    x += ref_x;
    y += ref_y;
    
    new_x.push_back(x);
    new_y.push_back(y);
  }
  
  std::vector<std::vector<double> > trajectory;
  
  trajectory.push_back(new_x);
  trajectory.push_back(new_y);
  std::cout << "New trajectory xy tail: " << new_x[new_x.size() -1 ] << "," << new_x[new_y.size() -1 ] << std::endl;
  return trajectory;
}

std::vector<std::vector<double> > Vehicle::Trajectory() {
  return this->compute_trajectory(lane, 30.0, this->trajectory_size);
}
