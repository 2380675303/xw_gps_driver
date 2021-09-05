#include <assert.h>
#include <errno.h>
#include <gps_driver/gps_driver.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include <iostream>
#include <vector>

#include "gps_driver/util.h"

namespace gps_driver
{
GpsDriver::GpsDriver() : fd_(-1), baud_(115200), eof_("\r\n"), isVerif_(false)
{
}

GpsDriver::~GpsDriver()
{
  close_device();
}

bool GpsDriver::verif(const std::string& s)
{
  int len = s.length();
  unsigned char cs = 0;
  unsigned char i = 0;
  char cs_hex[3];
  for (i = 1; i < len - 5; i++)
  {
    cs ^= s[i];
  }
  snprintf(cs_hex, 3, "%X", cs);
  if (cs_hex[0] == s[i + 1] && cs_hex[1] == s[i + 2])
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief 关闭设备文件
 */
void GpsDriver::close_device(void)
{
  ser_.close();
  ROS_ERROR("%s error %d, %s", "close", errno, strerror(errno));
  exit(EXIT_FAILURE);
  fd_ = -1;
}

/**
 * @brief 打开设备文件
 */
void GpsDriver::open_device(const std::string& gps_dev, int baud, bool isVerif)
{
  this->isVerif_ = isVerif;
  this->baud_ = baud;
  this->gps_dev_ = gps_dev;
  try
  {
    ser_.setPort(this->gps_dev_);
    ser_.setBaudrate(this->baud_);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser_.setTimeout(to);
    ser_.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR("Unable to open port %s", gps_dev_.c_str());
    ROS_ERROR("%s", e.what());
  }
}



std::string GpsDriver::read_data(std::vector<std::string>& msg)
{
  std::string buffer;
  ser_.readline(buffer, (size_t)127, eof_);
  util::split(buffer, msg, ',');
  if (isVerif_ == false || (isVerif_ && verif(buffer)))
  {
    return msg[0];
  }
  return "";
}


void GpsDriver::write_data(const std::string& str)
{
  ser_.write(str);
}

bool GpsDriver::is_running(void)
{
  return ser_.available();
}

}  // namespace gps_driver
