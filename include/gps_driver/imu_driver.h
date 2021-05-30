#ifndef _IMU_DRIVER_H_
#define _IMU_DRIVER_H_

#include<ros/ros.h>
#include<string>
#include<iostream>
#include<serial/serial.h>

namespace imu_driver
{

class IMUDriver
{
    public:
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
        serial::Serial ser_;
        int baud_;
        std::string eof_;
        bool isVerif_;

        IMUDriver();
        ~IMUDriver();

        void start(const std::string dev, int buad, bool isVerif);
        void shutdown();
        bool is_running();
        IMUData read_data();
        void write_data(std::string str);

        void close_device();
        void open_device(std::string imu_dev_, int baud);
        bool verif(const std::string& s);
        void decodeGTIMU(std::vector<std::string> tokens);

    private:
        int fd_;
        std::string imu_dev_;
};

}

#endif