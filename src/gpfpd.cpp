#include "gps_driver/gpfpd.h"


namespace nmea{

bool Gpfpd::decode(const std::vector<std::string>& raw_msg, DataT& data){
  // gps周数+周内秒+gps时间与unix时间之差-闰秒
  data.timestamp =
      stoi(raw_msg[1]) * 604800 + stod(raw_msg[2]) + 315964800 - this->leapSeconds_;
  data.heading = stof(raw_msg[3]);
  data.pitch = stof(raw_msg[4]);
  data.roll = stof(raw_msg[5]);
  data.lat = strtod(raw_msg[6].c_str(), NULL);
  data.lon = strtod(raw_msg[7].c_str(), NULL);
  data.alt = stof(raw_msg[8]);
  data.ve = stof(raw_msg[9]);
  data.vn = stof(raw_msg[10]);
  data.vu = stof(raw_msg[11]);
  data.baseline = stof(raw_msg[12]);
  data.nsv1 = stoi(raw_msg[13]);
  data.nsv2 = stoi(raw_msg[14]);
  data.status = raw_msg[15].c_str()[1];
  return true;
}

bool Gpfpd::parse(const DataT& data, MsgT& msg){
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