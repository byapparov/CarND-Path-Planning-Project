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
  
  this->speed = mph_to_ms(car_speed);
  
  trajectory_x.clear();
  trajectory_y.clear();
  
  int prev_size = previous_path_x.size();
  
  
  for (int i = 0; i < prev_size; ++i) {
    trajectory_x.push_back(previous_path_x[i]);
    trajectory_y.push_back(previous_path_y[i]);
    
  }
  
  if (prev_size < 2) {
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
 
}


void Vehicle::Accelerate() {
  return;
}

void Vehicle::SwitchState(vector<vector <double> > sensor_fusion) {
 
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
  
  double current_acceleration = this->acceleration;
  for (int i = 0; i < states.size(); i++) {
    next_state = states[i];
    int final_lane = get_final_lane(lane, next_state);
  
    double safe_speed = target_lane_safe_speed(
      car_s, 
      final_lane * 4 + 2, 
      this->reference_speed, 
      this->speed_limit, 
      delta_t, 
      sensor_fusion,
      50
    );
  
    vector<vector<double> > trajectory = this->compute_trajectory(final_lane, 50, 100, safe_speed);

    cost = trajectory_cost(trajectory,
                          lane_number_to_frenet(final_lane),
                          safe_speed,
                          this->speed_limit, 
                          delta_t, 
                          sensor_fusion,     
                          this->map_waypoints_x, 
                          this->map_waypoints_y);
                    
    if (cost < trajectory_cost_min || trajectory_cost_min < 0) {
      trajectory_cost_min = cost;
      optimal_state = next_state;
    }
    
    // restore the state of the car acceleration
    // this is currently updated by compute_trajectory function
    this->acceleration = current_acceleration; 
  }

    
  this->state = optimal_state;
  this->lane = get_final_lane(this->lane, this->state);
  
  this->target_speed = target_lane_safe_speed(
    car_s, 
    car_d, 
    this->speed, 
    this->speed_limit,
    delta_t, 
    sensor_fusion,
    40
  );
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
  
  vector<double> next_wp0 = getXY(
    ref_s + 30, 
    (lane_number_to_frenet(target_lane) + ref_d) / 2, 
    map_waypoints_s, 
    map_waypoints_x, 
    map_waypoints_y
  );
  
  vector<double> next_wp1 = getXY(
    ref_s + 45, 
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
  }
  
  tk::spline s;
  
  s.set_points(vx, vy);
  
  double target_x = distance;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  
  double x_add_on = 0;
  double point_speed = reference_speed;
  
  double cos_trajectory = target_x / sqrt( target_x * target_x + target_y * target_y);
  // copy current trajectory, so it is not
  // changed by experimental calculations
  vector<double> new_x = trajectory_x;
  vector<double> new_y = trajectory_y;
  
  for (int i = 0; i < size - trajectory_x.size(); i ++) {
    
    point_speed = speed_update(point_speed, safe_speed);
   
    x_add_on += (delta_t * point_speed) * cos_trajectory;
    double x = x_add_on;
    double y = s(x);
    
    vector<double> g;
    g = GlobalCoordinates(ref_x, ref_y, ref_yaw, x, y);
  
    new_x.push_back(g[0]);
    new_y.push_back(g[1]);
    
  }
  
  std::vector<std::vector<double> > trajectory;
  
  trajectory.push_back(new_x);
  trajectory.push_back(new_y);
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

  if(fabs(speed - safe_speed) <= 0.1) {
    return speed;
  }
  if (speed > safe_speed) {
    acceleration_update(-1);
  } 
  if (speed < safe_speed) {
    acceleration_update(1);
  }
  double res =  speed + acceleration * 0.02;
  if (res > speed_limit) {
    return speed_limit;
  }
  return res;
}

std::vector<std::vector<double> > Vehicle::Trajectory() {

  return this->compute_trajectory(lane, 30.0, this->trajectory_size, this->target_speed);
}
