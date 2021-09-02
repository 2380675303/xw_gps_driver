#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <gps_driver/gps_driver.h>
#include <gps_driver/gps_driver_node.h>
#include <gps_driver/util.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace gps_driver
{
GpsDriverNode::GpsDriverNode() : node_("~")
{
  // grab the parameters
  node_.param("gps_device", gps_device_, std::string("/dev/ttyUSB1"));
  node_.param("baud", baud_, 115200);
  node_.param("framerate", framerate_, 100);
  node_.param("isVerif", isVerif_, false);
  // check conection
  ROS_INFO("Starting '%s' at %d, Verif: %d", gps_device_.c_str(), baud_, isVerif_);

  // start the gps
  gps_.start(gps_device_.c_str(), baud_, isVerif_);
}

GpsDriverNode::~GpsDriverNode()
{
  gps_.shutdown();
}

bool GpsDriverNode::spin()
{
  float vel = 0;
  navsatfixPub_ = node_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);
  imuPub_ = node_.advertise<sensor_msgs::Imu>("/imu/data", 1000);
  ros::Rate loop_rate(this->framerate_);
  while (node_.ok())
  {
    if (gps_.is_running())
    {
      std::vector<std::string> raw_msg;
      auto msgType = gps_.read_data(raw_msg);
      switch (msgType)
      {
        case GP_DATA_t::GPFPD:
        {
          sensor_msgs::NavSatFix navSatFix_msg;

          gps_status_ = gps_.decodeGPFPD(raw_msg);
          parseNavSatFix(gps_status_, navSatFix_msg);
          navsatfixPub_.publish(navSatFix_msg);
          break;
        }
        case GP_DATA_t::GPRMC:
        {
          gps_status_ = gps_.decodeGPRMC(raw_msg);
          break;
        }
        case GP_DATA_t::GTIMU:
        {
          sensor_msgs::Imu imu_msg;
          ImuData_t imudata = gps_.decodeGTIMU(raw_msg);
          parseImuData(gps_status_, imudata, imu_msg);
          imuPub_.publish(imu_msg);
          break;
        }
        default:
          break;
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  return true;
}

void GpsDriverNode::parseNavSatFix(const GpsStatus_t& status, sensor_msgs::NavSatFix& navSatFix_msg)
{
  boost::array<double, 9> position_covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  navSatFix_msg.header.stamp = ros::Time(status.timestamp);
  navSatFix_msg.header.frame_id = "gps";
  navSatFix_msg.altitude = status.alt;
  navSatFix_msg.latitude = status.lat;
  navSatFix_msg.longitude = status.lon;
  navSatFix_msg.position_covariance = position_covariance;
  navSatFix_msg.position_covariance_type = 0;
}

void GpsDriverNode::parseImuData(const GpsStatus_t& gps_status, const ImuData_t& imu_data, sensor_msgs::Imu& imu_msg)
{
  // msg.header.stamp = ros::Time::now();
  imu_msg.header.stamp = ros::Time(imu_data.timestamp);
  imu_msg.header.frame_id = "imu";
  imu_msg.angular_velocity.x = toRad(imu_data.gyroY);
  imu_msg.angular_velocity.y = toRad(-imu_data.gyroX);
  imu_msg.angular_velocity.z = toRad(imu_data.gyroZ);
  imu_msg.linear_acceleration.x = imu_data.accY * 9.81;
  imu_msg.linear_acceleration.y = -imu_data.accX * 9.81;
  imu_msg.linear_acceleration.z = imu_data.accZ * 9.81;

  double yaw;
  double pitch;
  double roll;
  yaw = toRad(90.0 - gps_status.heading);
  pitch = toRad(-gps_status.pitch);
  roll = toRad(gps_status.roll);
  geometry_msgs::Quaternion quat;
  quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);  //返回四元数
  imu_msg.orientation.x = quat.x;
  imu_msg.orientation.y = quat.y;
  imu_msg.orientation.z = quat.z;
  imu_msg.orientation.w = quat.w;
}
}  // namespace gps_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_driver");
  gps_driver::GpsDriverNode xw_gi5610;

  xw_gi5610.spin();
  return EXIT_SUCCESS;
}
