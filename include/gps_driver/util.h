#ifndef GPS_UTIL_H
#define GPS_UTIL_H

#include <string.h>

namespace gps_driver
{
typedef enum
{
  ERR = -1,
  GPFPD = 0,
  GPRMC = 1,
  GTIMU = 2
} GP_DATA_t;

typedef struct GpsStatus
{
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
} GpsStatus_t;

typedef struct ImuData
{
  double timestamp;
  double gyroX;
  double gyroY;
  double gyroZ;
  double accX;
  double accY;
  double accZ;
  float tpr;
} ImuData_t;

void split(const std::string& s, std::vector<std::string>& tokens, char delim);

template <typename T>
T toRad(T deg)
{
  return deg * M_PI / 180.0;
}

template <typename T>
T toDeg(T rad)
{
  return rad * 180.0 / M_PI;
}
}  // namespace gps_driver

#endif