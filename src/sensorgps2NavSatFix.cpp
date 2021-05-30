#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <gps_driver/gps_driver.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <boost/array.hpp>

#include <sensor_msgs/NavSatFix.h>
#include <sleipnir_msgs/sensorgps.h>
#include <nav_msgs/Odometry.h>

namespace gps_driver
{
class GPSConvert
{
public:
  // private ROS node handle
  ros::NodeHandle node_;
  sensor_msgs::NavSatFix navSatFix_;
  sleipnir_msgs::sensorgps sensorgps_;
  boost::array<double, 9> position_covariance_;
  
  ros::Publisher pub_;
  ros::Subscriber sub_;
  void gpsCallback(const sleipnir_msgs::sensorgps::ConstPtr& msg);
  bool spin();
  // parameters
  GPSConvert() : node_("~")
  {
    ;
  }

  virtual ~GPSConvert()
  {
    ;
  }

};

void GPSConvert::gpsCallback(const sleipnir_msgs::sensorgps::ConstPtr& msg)
{
  navSatFix_.header.stamp = ros::Time::now ();
  // ROS_INFO_STREAM("gps:\t" << status.gps_time);
  navSatFix_.altitude = msg->alt;
  navSatFix_.latitude = msg->lat;
  navSatFix_.longitude= msg->lon;
  navSatFix_.position_covariance = position_covariance_;
  navSatFix_.position_covariance_type = 0;
  pub_.publish(navSatFix_);
}


bool GPSConvert::spin()
{
  ros::Rate loop_rate(50);
  pub_ = node_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);
  sub_ = node_.subscribe("/sensorgps",1000, &GPSConvert::gpsCallback, this);
  position_covariance_ = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  ros::spin();
  return 0;
}

}  // namespace gps_driver


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_driver");
  gps_driver::GPSConvert getStandardGPS;

  // message_filters::Subscriber<sensor_msgs::Imu> imu_sub(node_,"/imu/data" , 1);
  // typedef sync_policies::ExactTime<Imu, CameraInfo> MySyncPolicy;
  // // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub);
  // sync.registerCallback(boost::bind(&callback, _1));
  getStandardGPS.spin();
  return EXIT_SUCCESS;
}
