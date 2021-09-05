#ifndef GPS_DRIVER_GTIMU_H
#define GPS_DRIVER_GTIMU_H

#include "gps_driver/nmea.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string.h>
#include <boost/array.hpp>

namespace nmea{

typedef struct GtimuData
{
  double timestamp;
  double gyroX;
  double gyroY;
  double gyroZ;
  double accX;
  double accY;
  double accZ;
  float tpr;
} GtimuData_t;

class Gtimu: public NMEA<GtimuData_t, sensor_msgs::Imu>{
  using DataT = GtimuData_t;
  using MsgT = sensor_msgs::Imu;

public:
  Gtimu(ros::NodeHandle& nh, std::string topic, std::string frameId, int leap_seconds)
    : NMEA(nh, topic, "$GTIMU", frameId), leapSeconds_(leap_seconds){};
  ~Gtimu(){};
  bool decode(const std::vector<std::string>& raw_msg, DataT& data) override;
  bool parse(const DataT& data, MsgT& msg) override;

private:
  int leapSeconds_;
};
}

#endif