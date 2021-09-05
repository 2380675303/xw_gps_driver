#ifndef GPS_DRIVER_GPRMC_H
#define GPS_DRIVER_GPRMC_H

#include "gps_driver/nmea.h"
#include <sensor_msgs/NavSatFix.h>
#include <string.h>

namespace nmea{
typedef struct GprmcData
{
  double timestamp;
  double lat;
  double lon;
  float alt;
  char status;
  boost::array<double, 9> position_covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
} GprmcData_t;


class Gprmc: public NMEA<GprmcData_t, sensor_msgs::NavSatFix>{
  using DataT = GprmcData_t;
  using MsgT = sensor_msgs::NavSatFix;

public:
  Gprmc(ros::NodeHandle& nh, std::string topic, std::string frameId)
    : NMEA(nh, topic, "$GPRMC", frameId){};
  ~Gprmc()override{};
  bool decode(const std::vector<std::string>& raw_msg, DataT& data) override;
  bool parse(const DataT& data, MsgT& msg) override;

};
}

#endif