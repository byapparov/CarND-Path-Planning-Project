#include <vector>
using std::vector;

vector<vector <double> > predict_sensor_fusion(vector<vector <double> > sensor_fusion, double delta_t) {
  double sf_id, sf_s, sf_d, sf_x, sf_y, sf_xv, sf_yv, sf_v;
  
  vector<vector <double> > prediction;
  
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
    vector<double> p;
    p.push_back(sf_id);
    p.push_back(sf_s_predicted);
    p.push_back(sf_d); // for now we assume that cars don't change lanes
      
    prediction.push_back(p);

  }
  return prediction;
}
