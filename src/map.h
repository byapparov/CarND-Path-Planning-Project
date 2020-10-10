#include <vector>

using std::vector;

vector<double> LocalCoordinates(double ref_x, double ref_y, double ref_yaw, double x, double y) {
  vector<double> v;
  double shift_x = x - ref_x;
  double shift_y = y - ref_y;
  
  v.push_back((shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw)));
  v.push_back((shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw)));
  return v;
}

vector<double> GlobalCoordinates(double ref_x, double ref_y, double ref_yaw, double x, double y) {
  vector<double> v;
  
  double gx = x * cos(ref_yaw) - y * sin(ref_yaw);
  double gy = x * sin(ref_yaw) + y * cos(ref_yaw);
  
  gx += ref_x;
  gy += ref_y;
  
  v.push_back(gx);
  v.push_back(gy);
  return v;
}

double mph_to_ms(double x) {
  // 1 mhph is 0.44704 m/s
  // 1 m/s is 2.23694 mhph
  
  return x / 2.23694;
}


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
