#include <sstream>
#include <boost/array.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>

#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

#include <gps_driver/gps_driver.h>
#include <sleipnir_msgs/sensorgps.h>
#include <sensor_msgs/NavSatFix.h>

#define todeg 180/3.1415926 
#define torad 3.1415926/180

namespace gps_driver {
    class GPSDriverNode {
    public:
        // private ROS node handledag

        ros::NodeHandle node_;
        sleipnir_msgs::sensorgps sensorgps_;
        sensor_msgs::NavSatFix navSatFix_;

        ros::Publisher navsatfixPub_;
        ros::Publisher sensorgpsPub_;
        ros::Publisher imuPub_;
        ros::Subscriber sub_;

        GPSDriver gps_;
        struct GPSDriver::GPSStatus status;
        GPSDriver::IMUData imudata;

        sensor_msgs::Imu msgImu;

        // parameters
        std::string gps_device_;
        int baud_;
        int framerate_;
        bool isVerif_; //是否校验

        GPSDriverNode() : node_("~") {
            // grab the parameters
            node_.param("gps_device", gps_device_, std::string("/dev/ttyUSB1"));
            node_.param("baud", baud_, 115200);
            node_.param("framerate", framerate_, 100);
            node_.param("isVerif", isVerif_, false);
            // check conection
            ROS_INFO("Starting '%s' at %d, Verif: %d", gps_device_.c_str(), baud_, isVerif_);

            // start the gps
            gps_.start(gps_device_.c_str(), baud_, isVerif_);

            // std::cout<<"hello world!!!"<<std::endl;
        }

        virtual ~GPSDriverNode() {
            gps_.shutdown();
        }

        bool spin() {
            float vel = 0;
            navsatfixPub_ = node_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);
            sensorgpsPub_ = node_.advertise<sleipnir_msgs::sensorgps>("/sensorgps", 1000);
            imuPub_ =node_.advertise<sensor_msgs::Imu>("/imu/data",1000);
//    ros::Rate loop_rate(this->framerate_);
            boost::array<double, 9> position_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            while (node_.ok()) {
                if (gps_.is_running()) {
                    std::vector<std::string> tokens_;
                    // std::cout<<"tokens_:"<<tokens_.size()<<std::endl;
                    int MsgClass = gps_.read_data(tokens_);
                    switch(MsgClass)
                    {
                        case 1:
                            gps_.decodeGPFPD(tokens_);
                            break;
                        case 2:
                            gps_.decodeGPRMC(tokens_);
                            break;
                        case 3:
                            gps_.decodeGTIMU(tokens_);
                            break;
                        default:
                            break;
                    }
                    switch (MsgClass)
                    {
                        case 1:
                        {
                            status = gps_.gpsStatus_;
                            vel = sqrt(status.ve * status.ve + status.vn * status.vn + status.vu * status.vu);

                            geographic_msgs::GeoPoint ll;
                            ll.latitude = status.lat;
                            ll.longitude = status.lon;
                            ll.altitude = status.alt;
                            geodesy::UTMPoint pt(ll);

                            sensorgps_.x = pt.easting;
                            sensorgps_.y = pt.northing;
                            sensorgps_.lat = status.lat;
                            sensorgps_.lon = status.lon;
                            sensorgps_.alt = status.alt;
                            // left hand
                            sensorgps_.heading = status.heading;
                            sensorgps_.pitch = status.pitch;
                            sensorgps_.roll = -status.roll;
                            sensorgps_.velocity = vel;
                            sensorgps_.status = status.status;
                            sensorgps_.satenum = status.nsv1 + status.nsv2;
                            // sensorgps_.header.stamp = ros::Time(status.timestamp);
                            sensorgps_.header.stamp = ros::Time::now();
                            sensorgps_.header.frame_id = 'gps';
                            sensorgpsPub_.publish(sensorgps_);

                            // 标准GPS格式
                            navSatFix_.header.stamp = ros::Time(status.timestamp);
                            navSatFix_.header.frame_id = 'gps';
                            navSatFix_.altitude = status.alt;
                            navSatFix_.latitude = status.lat;
                            navSatFix_.longitude = status.lon;
                            navSatFix_.position_covariance = position_covariance;
                            navSatFix_.position_covariance_type = 0;
                            navsatfixPub_.publish(navSatFix_);
                            break;
                        }
                        case 3:
                        {
                            imudata = gps_.imuData_;
                            msgImu.header.stamp = ros::Time::now();
                            msgImu.header.frame_id = 'imu';
                            msgImu.angular_velocity.x = imudata.gyroY * torad;
                            msgImu.angular_velocity.y = -imudata.gyroX * torad;
                            msgImu.angular_velocity.z = imudata.gyroZ * torad;
                            msgImu.linear_acceleration.x = imudata.accY * 9.81;
                            msgImu.linear_acceleration.y = -imudata.accX * 9.81;
                            msgImu.linear_acceleration.z = imudata.accZ * 9.81;

                            double ros_yaw;
                            double ros_pitch;
                            double ros_roll;
                            ros_yaw = (90.0-gps_.gpsStatus_.heading) * torad;
                            ros_pitch = -gps_.gpsStatus_.pitch * torad;
                            ros_roll = gps_.gpsStatus_.roll * torad;
                            geometry_msgs::Quaternion quat;
                            quat = tf::createQuaternionMsgFromRollPitchYaw(ros_roll, ros_pitch, ros_yaw);//返回四元数
                            msgImu.orientation.x = quat.x;
                            msgImu.orientation.y = quat.y;
                            msgImu.orientation.z = quat.z;
                            msgImu.orientation.w = quat.w;
                            imuPub_.publish(msgImu);
                        }
                        default:
                            break;
                    }
                    // status = gps_.read_data();
                    // vel = sqrt(status.ve * status.ve + status.vn * status.vn + status.vu * status.vu);

                    // geographic_msgs::GeoPoint ll;
                    // ll.latitude = status.lat;
                    // ll.longitude = status.lon;
                    // ll.altitude = status.alt;
                    // geodesy::UTMPoint pt(ll);

                    // sensorgps_.x = pt.easting;
                    // sensorgps_.y = pt.northing;
                    // sensorgps_.lat = status.lat;
                    // sensorgps_.lon = status.lon;
                    // sensorgps_.alt = status.alt;
                    // sensorgps_.heading = status.heading;
                    // sensorgps_.pitch = status.pitch;
                    // sensorgps_.roll = status.roll;
                    // sensorgps_.velocity = vel;
                    // sensorgps_.status = status.status;
                    // sensorgps_.satenum = status.nsv1 + status.nsv2;
                    // // sensorgps_.header.stamp = ros::Time(status.timestamp);
                    // sensorgps_.header.stamp = ros::Time::now();
                    // sensorgps_.header.frame_id = 'gps';
                    // sensorgpsPub_.publish(sensorgps_);

                    // // 标准GPS格式
                    // navSatFix_.header.stamp = ros::Time(status.timestamp);
                    // navSatFix_.header.frame_id = 'gps';
                    // navSatFix_.altitude = status.alt;
                    // navSatFix_.latitude = status.lat;
                    // navSatFix_.longitude = status.lon;
                    // navSatFix_.position_covariance = position_covariance;
                    // navSatFix_.position_covariance_type = 0;
                    // navsatfixPub_.publish(navSatFix_);
                }
                ros::spinOnce();
//      loop_rate.sleep();
            }
            return true;
        }
    };
}  // namespace gps_driver


int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_driver");
    gps_driver::GPSDriverNode xw_gi5610;

    // message_filters::Subscriber<sensor_msgs::Imu> imu_sub(node_,"/imu/data" , 1);
    // typedef sync_policies::ExactTime<Imu, CameraInfo> MySyncPolicy;
    // // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub);
    // sync.registerCallback(boost::bind(&callback, _1));
    xw_gi5610.spin();
    return EXIT_SUCCESS;
}
