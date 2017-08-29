#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <vector>

using namespace std;

class SensorData {
public:
  struct SensedVehicleData {
    size_t id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
  };
  
  vector<SensedVehicleData> sensed_vehicle_list;
};

#endif