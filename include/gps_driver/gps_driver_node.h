#ifndef GPS_DRIVER_NODE_H
#define GPS_DRIVER_NODE_H

#include <ros/ros.h>
#include <gps_driver/util.h>
#include <gps_driver/nmea.h>

namespace gps_driver
{
class GpsDriverNode
{
private:
  ros::NodeHandle node_;

  ros::Publisher navsatfixPub_;
  ros::Publisher imuPub_;

  GpsDriver gps_;
  std::string gps_device_;
  int baud_;
  int framerate_;
  bool isVerif_;  //是否校验
  int leapSec_;
  std::map<std::string, nmea::NMEA_Base*> nmea_map_;

public:
  GpsDriverNode();
  virtual ~GpsDriverNode();
  bool spin();
  void registerNmea(nmea::NMEA_Base* nmea);
  // registerNmea();
};
}  // namespace gps_driver

#endif