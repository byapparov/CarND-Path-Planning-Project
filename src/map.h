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
