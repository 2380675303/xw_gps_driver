#ifndef GPS_DRIVER_GPFPD_H
#define GPS_DRIVER_GPFPD_H

#include "gps_driver/nmea.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <string.h>
#include <boost/array.hpp>

namespace nmea{
typedef struct GpfpdData
{
  double timestamp;
  float heading;
  float pitch;
  float roll;
  double lat;
  double lon;
  float alt;
  float ve;
  float vn;
  float vu;
  float baseline;
  int nsv1;
  int nsv2;
  char status;
  boost::array<double, 9> position_covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
} GpfpdData_t;

class Gpfpd: public NMEA<GpfpdData_t, sensor_msgs::NavSatFix>{
  using DataT = GpfpdData_t;
  using MsgT = sensor_msgs::NavSatFix;

public:
  Gpfpd(ros::NodeHandle& nh, std::string topic, std::string frameId, int leap_seconds)
    : NMEA(nh, topic, "$GPFPD", frameId), leapSeconds_(leap_seconds){};
  ~Gpfpd(){};
  bool decode(const std::vector<std::string>& raw_msg, DataT& data) override;
  bool parse(const DataT& data, MsgT& msg) override;

private:
  int leapSeconds_;
};
}

#endif