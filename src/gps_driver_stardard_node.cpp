#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <gps_driver/gps_driver.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <boost/array.hpp>

namespace gps_driver
{
class GPSDriverNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;
  sensor_msgs::NavSatFix navSatFix_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  GPSDriver gps_;
  struct GPSDriver::GPSStatus status;

  // parameters
  std::string gps_device_;
  int baud_;
  int framerate_;
  bool isVerif_; //是否校验

  GPSDriverNode() : node_("~")
  {
    // grab the parameters
    node_.param("gps_device", gps_device_, std::string("/dev/ttyUSB0"));
    node_.param("baud", baud_, 115200);
    node_.param("framerate", framerate_, 50);
    node_.param("isVerif", isVerif_, false);
    ROS_INFO("Starting '%s' at %d, Verif: %d", gps_device_.c_str(), baud_, isVerif_);

    // start the gps
    gps_.start(gps_device_.c_str(), baud_, isVerif_);
  }

  virtual ~GPSDriverNode()
  {
    gps_.shutdown();
  }

  bool spin()
  {
    // std::string test;
    pub_ = node_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);
    ros::Rate loop_rate(this->framerate_);
    boost::array<double, 9> position_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    while (node_.ok())
    {
      if (gps_.is_running())
      {
        status = gps_.read_data();
        navSatFix_.header.stamp = ros::Time(status.timestamp);
        // ROS_INFO_STREAM("gps:\t" << status.gps_time);
        navSatFix_.altitude = status.alt;
        navSatFix_.latitude = status.lat;
        navSatFix_.longitude= status.lon;
        navSatFix_.position_covariance = position_covariance;
        navSatFix_.position_covariance_type = 0;
        pub_.publish(navSatFix_);
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    return true;
  }
};

}  // namespace gps_driver


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_driver");
  gps_driver::GPSDriverNode xw_g5610;

  // message_filters::Subscriber<sensor_msgs::Imu> imu_sub(node_,"/imu/data" , 1);
  // typedef sync_policies::ExactTime<Imu, CameraInfo> MySyncPolicy;
  // // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub);
  // sync.registerCallback(boost::bind(&callback, _1));
  xw_g5610.spin();
  return EXIT_SUCCESS;
}
