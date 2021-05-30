#define __STDC_CONSTANT_MACROS
#include <assert.h>
#include <errno.h>
#include <fcntl.h> /* low-level i/o */
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <time.h>
#include <vector>

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <boost/lexical_cast.hpp>

#include <gps_driver/gps_driver.h>

namespace gps_driver
{
static void error_exit(const char* s)
{
  ROS_ERROR("%s error %d, %s", s, errno, strerror(errno));
  exit(EXIT_FAILURE);
}

/**
 * @brief 字符分割
 * @param s: 要分割的字符串
 * @param tokens: 结果
 * @param delim: 分隔符
 */
void split(const std::string& s, std::vector<std::string>& tokens, char delim = ' ')
{
  tokens.clear();
  auto string_find_first_not = [s, delim](size_t pos = 0) -> size_t {
    for (size_t i = pos; i < s.size(); i++)
    {
      if (s[i] != delim)
        return i;
    }
    return std::string::npos;
  };
  size_t lastPos = string_find_first_not(0);
  size_t pos = s.find(delim, lastPos);
  while (lastPos != std::string::npos)
  {
    tokens.emplace_back(s.substr(lastPos, pos - lastPos));
    lastPos = string_find_first_not(pos);
    pos = s.find(delim, lastPos);
  }
}

GPSDriver::GPSDriver() : fd_(-1), baud_(115200), eof_("\r\n"), isVerif_(false)
{
}

GPSDriver::~GPSDriver()
{
  shutdown();
}

bool GPSDriver::verif(const std::string& s){
    int len = s.length();
    unsigned char cs = 0;
    unsigned char i = 0;
    char cs_hex[3];
    for (i = 1; i < len-5;i++){
        cs ^= s[i];
    }
    snprintf(cs_hex, 3, "%X", cs);
    if(cs_hex[0] == s[i+1] && cs_hex[1] == s[i+2]){
        return true;
    }
    else {
        return false;
    }
}

/**
 * @brief 关闭设备文件
 */
void GPSDriver::close_device(void)
{
  ser_.close();
  gps_driver::error_exit("close");
  fd_ = -1;
}

/**
 * @brief 打开设备文件
 */
void GPSDriver::open_device(std::string gps_dev, int baud)
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

void GPSDriver::start(const std::string& dev, int baud, bool isVerif)
{
  isVerif_ = isVerif;
  baud_ = baud;
  gps_dev_ = dev;
  open_device(gps_dev_, baud_);
}

void GPSDriver::shutdown(void)
{
  close_device();
}

void GPSDriver::decodeGPFPD(std::vector<std::string> tokens)
{
  // std::cout<<"GPS!!!"<<std::endl;
  // int count = tokens.size();
  // for(int it=0;it<count;it++)
  // {
  //   std::cout<<tokens[it]<<",";
  // }
  // std::cout<<std::endl;
    gpsStatus_.timestamp = stoi(tokens[1]) * 604800 + stod(tokens[2]) + 315964800 - 18; //gps周数+周内秒+gps时间与unix时间之差-闰秒
    gpsStatus_.heading = stof(tokens[3]);
    gpsStatus_.pitch = stof(tokens[4]);
    gpsStatus_.roll = stof(tokens[5]);
    gpsStatus_.lat = strtod(tokens[6].c_str(), NULL);
    gpsStatus_.lon = strtod(tokens[7].c_str(), NULL);
    gpsStatus_.alt = stof(tokens[8]);
    gpsStatus_.ve = stof(tokens[9]);
    gpsStatus_.vn = stof(tokens[10]);
    gpsStatus_.vu = stof(tokens[11]);
    gpsStatus_.baseline = stof(tokens[12]);
    gpsStatus_.nsv1 = stoi(tokens[13]);
    gpsStatus_.nsv2 = stoi(tokens[14]);
    gpsStatus_.status = tokens[15].c_str()[1];
}

void GPSDriver::decodeGPRMC(std::vector<std::string> tokens)
{
    char sign;
    char ddd,mm;
    double dddmm;
    auto toInt = [](std::string s)->int{return (s.c_str()[0] - '0')*10+(s.c_str()[1] - '0');};

    gpsStatus_.status = tokens[9].c_str()[0];
    // lat
    sign = tokens[4] == "N"?1:-1;
    dddmm = stod(tokens[3]);
    ddd = int(dddmm/100);
    mm = (dddmm - ddd*100)/60.0;
    gpsStatus_.lat = sign * (ddd + mm);
    // lon
    sign = tokens[6] == "E"?1:-1;
    dddmm = stod(tokens[5]);
    ddd = int(dddmm/100);
    mm = (dddmm - ddd*100)/60.0;
    gpsStatus_.lon = sign * (ddd + mm);

    //timestamp
    struct tm t;
    std::string date = tokens[9];
    std::string time = tokens[1];
    t.tm_mday = toInt(date.substr(0,2));
    t.tm_mon = toInt(date.substr(2,2));
    t.tm_year = 100+toInt(date.substr(4,2));
    t.tm_hour = toInt(time.substr(0,2));
    t.tm_min = toInt(time.substr(2,2));
    t.tm_sec = toInt(time.substr(4,2));
    float ms = (float)(toInt(time.substr(7,2)))/100.0;
    t.tm_isdst = 0;
    gpsStatus_.timestamp = mktime(&t) + ms;
}

void GPSDriver::decodeGTIMU(std::vector<std::string> tokens)
{
  // int count = tokens.size();
  // for(int it=0;it<count;it++)
  // {
  //   std::cout<<tokens[it]<<",";
  // }
  // std::cout<<std::endl;
  // std::cout<<"imu!!!"<<std::endl;
    imuData_.timestamp = stoi(tokens[1]) * 604800 + stod(tokens[2]) + 315964800 - 18; //gps周数+周内秒+gps时间与unix时间之差-闰秒
    imuData_.gyroX = strtod(tokens[3].c_str(), NULL);
    imuData_.gyroY = strtod(tokens[4].c_str(), NULL);
    imuData_.gyroZ = strtod(tokens[5].c_str(), NULL);
    imuData_.accX = strtod(tokens[6].c_str(), NULL);
    imuData_.accY = strtod(tokens[7].c_str(), NULL);
    imuData_.accZ = strtod(tokens[8].c_str(), NULL);
    imuData_.tpr = stof(tokens[9]);
}

struct GPSDriver::GPSStatus GPSDriver::read_data(void)
{
    std::string buffer_;
    std::vector<std::string> tokens;
    ser_.readline(buffer_, (size_t)127, eof_);
    split(buffer_, tokens, ',');
    // for (auto token : tokens){
    //     std::cout << token << std::endl;
    // }
    if (isVerif_ == false || (isVerif_ && verif(buffer_))){
        if (tokens[0] == "$GPFPD")
        {
            decodeGPFPD(tokens);
        }
        else if (tokens[0] == "$GPRMC"){
            decodeGPRMC(tokens);
        }  
    }
    return gpsStatus_;
}

int GPSDriver::read_data(std::vector<std::string>& tokens)
{
    std::string buffer_;
    // std::vector<std::string> tokens;
    ser_.readline(buffer_, (size_t)127, eof_);
    split(buffer_, tokens, ',');
    // for (auto token : tokens){
    //     std::cout << token << std::endl;
    // }
    if (isVerif_ == false || (isVerif_ && verif(buffer_))){
        if (tokens[0] == "$GPFPD")
        {
            // decodeGPFPD(tokens);
            return 1;
        }
        else if (tokens[0] == "$GPRMC"){
            // decodeGPRMC(tokens);
            return 2;
        }  
        else if (tokens[0] == "$GTIMU"){
            // decodeGPRMC(tokens);
            return 3;
        }
        else
        {
           return 0;
        }
    }
    else
    {
      return 0;
    }
    // return gpsStatus_;
}


void GPSDriver::write_data(std::string str)
{
  ser_.write(str);
}

bool GPSDriver::is_running(void)
{
  return ser_.available();
}

}  // namespace gps_driver
