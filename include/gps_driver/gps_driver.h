#ifndef GPS_DRIVER_H
#define GPS_DRIVER_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <string>

#include "gps_driver/util.h"

namespace gps_driver
{
class GpsDriver
{
public:
  GpsDriver();
  ~GpsDriver();

  // start
  void start(const std::string& dev, int baud, bool isVerif);
  // shutdown
  void shutdown(void);
  bool is_running(void);
  void write_data(std::string str);
  void close_device(void);
  void open_device(std::string gps_dev_, int baud);
  bool verif(const std::string& s);
  gps_driver::GP_DATA_t read_data(std::vector<std::string>& msg);
  GpsStatus_t decodeGPRMC(const std::vector<std::string>& msg);
  GpsStatus_t decodeGPFPD(const std::vector<std::string>& msg);
  ImuData_t decodeGTIMU(const std::vector<std::string>& msg);

private:
  int fd_;
  std::string gps_dev_;
  serial::Serial ser_;
  int baud_;
  std::string eof_;
  bool isVerif_;  //是否校验
};
}  // namespace gps_driver

#endif
