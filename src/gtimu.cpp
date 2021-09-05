#include "gps_driver/gtimu.h"
#include "gps_driver/util.h"


namespace nmea{

bool Gtimu::decode(const std::vector<std::string>& raw_msg, DataT& data){
  data.timestamp =
      stoi(raw_msg[1]) * 604800 + stod(raw_msg[2]) + 315964800 - this->leapSeconds_;  // gps周数+周内秒+gps时间与unix时间之差-闰秒
  data.gyroX = strtod(raw_msg[3].c_str(), NULL);
  data.gyroY = strtod(raw_msg[4].c_str(), NULL);
  data.gyroZ = strtod(raw_msg[5].c_str(), NULL);
  data.accX = strtod(raw_msg[6].c_str(), NULL);
  data.accY = strtod(raw_msg[7].c_str(), NULL);
  data.accZ = strtod(raw_msg[8].c_str(), NULL);
  data.tpr = stof(raw_msg[9]);
  return true;
}

bool Gtimu::parse(const DataT& data, MsgT& msg){
  msg.header.stamp = ros::Time(data.timestamp);
  msg.header.frame_id = this->frameId_;
  msg.angular_velocity.x = util::toRad(data.gyroY);
  msg.angular_velocity.y = util::toRad(-data.gyroX);
  msg.angular_velocity.z = util::toRad(data.gyroZ);
  msg.linear_acceleration.x = data.accY * 9.81;
  msg.linear_acceleration.y = -data.accX * 9.81;
  msg.linear_acceleration.z = data.accZ * 9.81;
  return true;
}

}