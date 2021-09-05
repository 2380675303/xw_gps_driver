#include "gps_driver/gprmc.h"
#include <time.h>
#include <string>

namespace nmea{

bool Gprmc::decode(const std::vector<std::string>& raw_msg, GprmcData_t& data){
  char sign;
  char ddd, mm;
  double dddmm;
  auto toInt = [](std::string s) -> int { return (s.c_str()[0] - '0') * 10 + (s.c_str()[1] - '0'); };

  data.status = raw_msg[9].c_str()[0];
  // lat
  sign = raw_msg[4] == "N" ? 1 : -1;
  dddmm = stod(raw_msg[3]);
  ddd = int(dddmm / 100);
  mm = (dddmm - ddd * 100) / 60.0;
  data.lat = sign * (ddd + mm);
  // lon
  sign = raw_msg[6] == "E" ? 1 : -1;
  dddmm = stod(raw_msg[5]);
  ddd = int(dddmm / 100);
  mm = (dddmm - ddd * 100) / 60.0;
  data.lon = sign * (ddd + mm);

  // timestamp
  struct tm t;
  std::string date = raw_msg[9];
  std::string time = raw_msg[1];
  t.tm_mday = toInt(date.substr(0, 2));
  t.tm_mon = toInt(date.substr(2, 2));
  t.tm_year = 100 + toInt(date.substr(4, 2));
  t.tm_hour = toInt(time.substr(0, 2));
  t.tm_min = toInt(time.substr(2, 2));
  t.tm_sec = toInt(time.substr(4, 2));
  float ms = (float)(toInt(time.substr(7, 2))) / 100.0;
  t.tm_isdst = 0;
  data.timestamp = mktime(&t) + ms;
  return true;
}

bool Gprmc::parse(const GprmcData_t& data, sensor_msgs::NavSatFix& msg){
  msg.header.stamp = ros::Time(data.timestamp);
  msg.header.frame_id = this->frameId_;
  msg.altitude = data.alt;
  msg.latitude = data.lat;
  msg.longitude = data.lon;
  msg.position_covariance = data.position_covariance;
  msg.position_covariance_type = 0;
  return true;
}
}