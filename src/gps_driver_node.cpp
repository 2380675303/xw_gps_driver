#include <gps_driver/gps_driver.h>
#include <gps_driver/gps_driver_node.h>
#include <gps_driver/util.h>
#include <ros/ros.h>
#include <gps_driver/gpfpd.h>
#include <gps_driver/gprmc.h>
#include <gps_driver/gtimu.h>
#include <map>

namespace gps_driver
{
GpsDriverNode::GpsDriverNode() : node_("~")
{
  // grab the parameters
  node_.param("gps_device", gps_device_, std::string("/dev/ttyUSB1"));
  node_.param("baud", baud_, 115200);
  node_.param("framerate", framerate_, 100);
  node_.param("isVerif", isVerif_, false);
  node_.param("leapSec", leapSec_, 18);
  // check conection
  ROS_INFO("Starting '%s' at %d, Verif: %d", gps_device_.c_str(), baud_, isVerif_);

  // start the gps
  gps_.open_device(gps_device_.c_str(), baud_, isVerif_);
}

GpsDriverNode::~GpsDriverNode()
{
  gps_.close_device();
}

void GpsDriverNode::registerNmea(nmea::NMEA_Base* nmea){
  nmea_map_[nmea->getMessageId()] = nmea;
}

bool GpsDriverNode::spin()
{

  auto gprmc = nmea::Gprmc(node_, "/gps/gprmc/", "gps");
  auto gpfpd = nmea::Gpfpd(node_, "/gps/gpfpd/", "gps", leapSec_);
  auto gtimu = nmea::Gtimu(node_, "/gps/gpfpd/", "gps", leapSec_);
  registerNmea(&gprmc);
  registerNmea(&gpfpd);
  registerNmea(&gtimu);

  ros::Rate loop_rate(this->framerate_);
  while (node_.ok())
  {
    if (gps_.is_running())
    {
      std::vector<std::string> raw_msg;
      auto msgId = gps_.read_data(raw_msg);
      if(msgId == "") continue;
      if(nmea_map_.contains(msgId)){
        nmea_map_[msgId]->operator()(raw_msg);
      }
    }
    loop_rate.sleep();
  }
  return true;
}

}  // namespace gps_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_driver");
  gps_driver::GpsDriverNode xw_gi5610;

  xw_gi5610.spin();
  return EXIT_SUCCESS;
}
