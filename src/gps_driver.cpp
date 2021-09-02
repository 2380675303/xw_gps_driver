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
  shutdown();
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
void GpsDriver::open_device(std::string gps_dev, int baud)
{
  try
  {
    ser_.setPort(gps_dev);
    ser_.setBaudrate(baud);
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

void GpsDriver::start(const std::string& dev, int baud, bool isVerif)
{
  isVerif_ = isVerif;
  baud_ = baud;
  gps_dev_ = dev;
  open_device(gps_dev_, baud_);
}

void GpsDriver::shutdown(void)
{
  close_device();
}

GpsStatus_t GpsDriver::decodeGPFPD(const std::vector<std::string>& msg)
{
  GpsStatus_t gpsStatus;
  gpsStatus.timestamp =
      stoi(msg[1]) * 604800 + stod(msg[2]) + 315964800 - 18;  // gps周数+周内秒+gps时间与unix时间之差-闰秒
  gpsStatus.heading = stof(msg[3]);
  gpsStatus.pitch = stof(msg[4]);
  gpsStatus.roll = stof(msg[5]);
  gpsStatus.lat = strtod(msg[6].c_str(), NULL);
  gpsStatus.lon = strtod(msg[7].c_str(), NULL);
  gpsStatus.alt = stof(msg[8]);
  gpsStatus.ve = stof(msg[9]);
  gpsStatus.vn = stof(msg[10]);
  gpsStatus.vu = stof(msg[11]);
  gpsStatus.baseline = stof(msg[12]);
  gpsStatus.nsv1 = stoi(msg[13]);
  gpsStatus.nsv2 = stoi(msg[14]);
  gpsStatus.status = msg[15].c_str()[1];
  return gpsStatus;
}

GpsStatus_t GpsDriver::decodeGPRMC(const std::vector<std::string>& msg)
{
  GpsStatus_t gpsStatus;
  char sign;
  char ddd, mm;
  double dddmm;
  auto toInt = [](std::string s) -> int { return (s.c_str()[0] - '0') * 10 + (s.c_str()[1] - '0'); };

  gpsStatus.status = msg[9].c_str()[0];
  // lat
  sign = msg[4] == "N" ? 1 : -1;
  dddmm = stod(msg[3]);
  ddd = int(dddmm / 100);
  mm = (dddmm - ddd * 100) / 60.0;
  gpsStatus.lat = sign * (ddd + mm);
  // lon
  sign = msg[6] == "E" ? 1 : -1;
  dddmm = stod(msg[5]);
  ddd = int(dddmm / 100);
  mm = (dddmm - ddd * 100) / 60.0;
  gpsStatus.lon = sign * (ddd + mm);

  // timestamp
  struct tm t;
  std::string date = msg[9];
  std::string time = msg[1];
  t.tm_mday = toInt(date.substr(0, 2));
  t.tm_mon = toInt(date.substr(2, 2));
  t.tm_year = 100 + toInt(date.substr(4, 2));
  t.tm_hour = toInt(time.substr(0, 2));
  t.tm_min = toInt(time.substr(2, 2));
  t.tm_sec = toInt(time.substr(4, 2));
  float ms = (float)(toInt(time.substr(7, 2))) / 100.0;
  t.tm_isdst = 0;
  gpsStatus.timestamp = mktime(&t) + ms;
  return gpsStatus;
}

ImuData_t GpsDriver::decodeGTIMU(const std::vector<std::string>& msg)
{
  ImuData_t imuData;
  imuData.timestamp =
      stoi(msg[1]) * 604800 + stod(msg[2]) + 315964800 - 18;  // gps周数+周内秒+gps时间与unix时间之差-闰秒
  imuData.gyroX = strtod(msg[3].c_str(), NULL);
  imuData.gyroY = strtod(msg[4].c_str(), NULL);
  imuData.gyroZ = strtod(msg[5].c_str(), NULL);
  imuData.accX = strtod(msg[6].c_str(), NULL);
  imuData.accY = strtod(msg[7].c_str(), NULL);
  imuData.accZ = strtod(msg[8].c_str(), NULL);
  imuData.tpr = stof(msg[9]);
  return imuData;
}

gps_driver::GP_DATA_t GpsDriver::read_data(std::vector<std::string>& msg)
{
  std::string buffer;
  ser_.readline(buffer, (size_t)127, eof_);
  split(buffer, msg, ',');
  if (isVerif_ == false || (isVerif_ && verif(buffer)))
  {
    if (msg[0] == "$GPFPD")
    {
      return gps_driver::GP_DATA_t::GPFPD;
    }
    else if (msg[0] == "$GPRMC")
    {
      return gps_driver::GP_DATA_t::GPRMC;
    }
    else if (msg[0] == "$GTIMU")
    {
      return gps_driver::GP_DATA_t::GTIMU;
    }
  }
  return gps_driver::GP_DATA_t::ERR;
}


void GpsDriver::write_data(std::string str)
{
  ser_.write(str);
}

bool GpsDriver::is_running(void)
{
  return ser_.available();
}

}  // namespace gps_driver
