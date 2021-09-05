#ifndef GPS_DRIVER_NMEA_H
#define GPS_DRIVER_NMEA_H

#include <vector>
#include <string>
#include <ros/ros.h>

namespace nmea{
    
class NMEA_Base{
    public:
        NMEA_Base(std::string messageId, std::string frameId)
          :messageId_(messageId), frameId_(frameId){};
        virtual ~NMEA_Base() = default;
        virtual bool operator()(const std::vector<std::string>& raw_msg) = 0;
        std::string getMessageId() const{return messageId_;};
    protected:
        std::string messageId_;
        std::string frameId_;
};

template<typename DataT, typename MsgT>
class NMEA: public NMEA_Base{
    public:
        NMEA(ros::NodeHandle& nh, std::string topic, std::string messageId, std::string frameId, int queueSize=1000)
          :NMEA_Base(messageId, frameId), nh_(nh), topic_(topic), queueSize_(queueSize){
              pub_ = nh_.advertise<MsgT>(topic_, queueSize_);
          };
        virtual ~NMEA() = default;
        virtual bool decode(const std::vector<std::string>& raw_msg, DataT& data) = 0;
        virtual bool parse(const DataT& data, MsgT& msg) = 0;
        bool operator()(const std::vector<std::string>& raw_msg) override{
            DataT data;
            MsgT msg;
            if(!decode(raw_msg, data)){
                return false;
            }
            if(!parse(data, msg)){
                return false;
            }
            pub_.publish(msg);
            return true;
        }

    protected:
        MsgT msg_;
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        std::string topic_;
        int queueSize_;
};

}

#endif