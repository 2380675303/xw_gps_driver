#ifndef GPS_DRIVER_NODE_H
#define GPS_DRIVER_NODE_H

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <serial/serial.h>
#include <sleipnir_msgs/sensorgps.h>
#include <gps_driver/util.h>

namespace gps_driver
{
class GpsDriverNode
{
private:
  ros::NodeHandle node_;
  sleipnir_msgs::sensorgps sensorgps_;
  sensor_msgs::NavSatFix navSatFix_;

  ros::Publisher navsatfixPub_;
  ros::Publisher sensorgpsPub_;
  ros::Publisher imuPub_;
  ros::Subscriber sub_;

  GpsDriver gps_;
  std::string gps_device_;
  GpsStatus gps_status_;
  int baud_;
  int framerate_;
  bool isVerif_;  //是否校验

public:
  GpsDriverNode();
  virtual ~GpsDriverNode();
  bool spin();
  void parseSensorGps(const GpsStatus_t& status, sleipnir_msgs::sensorgps& sensorgps);
  void parseNavSatFix(const GpsStatus_t& status, sensor_msgs::NavSatFix& navSatFix);
  void parseImuData(const GpsStatus_t& status, const ImuData_t& imudata, sensor_msgs::Imu& imu);
};
}  // namespace gps_driver

#endif