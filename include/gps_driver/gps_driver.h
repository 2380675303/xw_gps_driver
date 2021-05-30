#ifndef GPS_DRIVER_H
#define GPS_DRIVER_H

#include <ros/ros.h>
#include <string>
#include <sstream>
#include <serial/serial.h>

namespace gps_driver {

class GPSDriver {
 public:

  struct GPSStatus{
      double timestamp;
      float heading;
      float pitch;
      float roll;
      double lat;
      double lon;
      float alt;
      float ve;
      float vn;
      float vu;
      float baseline;
      int nsv1;
      int nsv2;
      char status;
  };

  struct IMUData
  {
      double timestamp;
      double gyroX;
      double gyroY;
      double gyroZ;
      double accX;
      double accY;
      double accZ;
      float tpr;
  };

  IMUData imuData_;

  struct GPSStatus gpsStatus_;
  serial::Serial ser_;
  int baud_;
  std::string eof_;
  bool isVerif_; //是否校验

  GPSDriver();
  ~GPSDriver();

  // start
  void start(const std::string& dev, int baud, bool isVerif);
  // shutdown
  void shutdown(void);
  bool is_running(void);
  struct GPSStatus read_data(void);
  int read_data(std::vector<std::string>& tokens);
  void write_data(std::string str);
  void decodeGPRMC(std::vector<std::string> tokens);
  void decodeGPFPD(std::vector<std::string> tokens);
  void decodeGTIMU(std::vector<std::string> tokens);

 private:
 
  int fd_;
  std::string gps_dev_;
  void close_device(void);
  void open_device(std::string gps_dev_, int baud);
  bool verif(const std::string& s);
};

}


#endif

