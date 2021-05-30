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

#include <gps_driver/imu_driver.h>

namespace imu_driver
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

IMUDriver::IMUDriver() : fd_(-1), baud_(115200), eof_("\r\n"), isVerif_(false)
{
}

IMUDriver::~IMUDriver()
{
  shutdown();
}

bool IMUDriver::verif(const std::string& s){
    int len = s.length();
    unsigned char cs = 0;
    unsigned char i = 0;
    char cs_hex[3];
    for (i = 1; i < len-5;i++){
        cs ^= s[i];
    }
    // C 库函数 int snprintf(char *str, size_t size, const char *format, ...) 设将可变参数(...)
    // 按照 format 格式化成字符串，并将字符串复制到 str 中，size 为要写入的字符的最大数目，超过 size 会被截断。
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
void IMUDriver::close_device(void)
{
  ser_.close();
  imu_driver::error_exit("close");
  fd_ = -1;
}

/**
 * @brief 打开设备文件
 */
void IMUDriver::open_device(std::string imu_dev_, int baud)
{
  try
  {
    ser_.setPort(imu_dev_);
    ser_.setBaudrate(baud);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser_.setTimeout(to);
    ser_.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR("Unable to open port %s", imu_dev_.c_str());
    ROS_ERROR("%s", e.what());
  }
}

void IMUDriver::start(const std::string dev, int baud, bool isVerif)
{
  isVerif_ = isVerif;
  baud_ = baud;
  imu_dev_ = dev;
  open_device(imu_dev_, baud_);
}

void IMUDriver::shutdown(void)
{
  close_device();
}

void IMUDriver::decodeGTIMU(std::vector<std::string> tokens)
{
    imuData_.timestamp = stoi(tokens[1]) * 604800 + stod(tokens[2]) + 315964800 - 18; //gps周数+周内秒+gps时间与unix时间之差-闰秒
    imuData_.gyroX = strtod(tokens[3].c_str(), NULL);
    imuData_.gyroY = strtod(tokens[4].c_str(), NULL);
    imuData_.gyroZ = strtod(tokens[5].c_str(), NULL);
    imuData_.accX = strtod(tokens[6].c_str(), NULL);
    imuData_.accY = strtod(tokens[7].c_str(), NULL);
    imuData_.accZ = strtod(tokens[8].c_str(), NULL);
    imuData_.tpr = stof(tokens[9]);
}

struct IMUDriver::IMUData IMUDriver::read_data(void)
{
    std::string buffer_;
    std::vector<std::string> tokens;
    ser_.readline(buffer_, (size_t)127, eof_);
    split(buffer_, tokens, ',');
    // for (auto token : tokens){
    //     std::cout << token << std::endl;
    // }
    if (isVerif_ == false || (isVerif_ && verif(buffer_))){
        if (tokens[0] == "$GTIMU")
        {
            decodeGTIMU(tokens);
        }
    }
    return imuData_;
}


void IMUDriver::write_data(std::string str)
{
  ser_.write(str);
}

bool IMUDriver::is_running(void)
{
  return ser_.available();
}
}