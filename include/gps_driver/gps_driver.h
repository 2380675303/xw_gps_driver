#ifndef GPS_DRIVER_H
#define GPS_DRIVER_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <string>


namespace gps_driver
{
class GpsDriver
{
public:
  GpsDriver();
  ~GpsDriver();

  bool is_running(void);
  void write_data(const std::string& str);
  void close_device(void);
  void open_device(const std::string& gps_dev_, int baud, bool isVerif);
  bool verif(const std::string& s);
  std::string read_data(std::vector<std::string>& msg);

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
