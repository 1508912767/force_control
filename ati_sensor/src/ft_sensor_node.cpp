#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include "ft_sensor.h"

int count = 0;
FILE *f;


namespace ftsensor {

class FTSensorPublisher
{
private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    //! The sensor
    boost::shared_ptr<ati::FTSensor> ftsensor_;
    std::string ip_;
    std::string frame_ft_;

    //! Publisher for sensor readings
    ros::Publisher pub_sensor_readings_;

    //! Service for setting the bias
    ros::ServiceServer srv_set_bias_;

public:
    //------------------ Callbacks -------------------
    // Callback for setting bias
    bool setBiasCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

    // Publish the measurements
    void publishMeasurements();

    //! Subscribes to and advertises topics
    FTSensorPublisher(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
        priv_nh_.param<std::string>("frame", frame_ft_, "/ati_ft_link");
        priv_nh_.param<std::string>("ip", ip_, "192.168.1.1");

        ROS_INFO_STREAM("ATISensor IP : "<< ip_);
        ROS_INFO_STREAM("ATISensor frame : "<< frame_ft_);

        // Create a new sensor
        ftsensor_ = boost::shared_ptr<ati::FTSensor>(new ati::FTSensor());

        // Init FT Sensor
        if (ftsensor_->init(ip_))
        {
            // Set bias
            ftsensor_->setBias();

            ROS_INFO_STREAM("ATISensor RDT Rate : "<< ftsensor_->getRDTRate());

            // Advertise topic where readings are published
            pub_sensor_readings_ = priv_nh_.advertise<geometry_msgs::WrenchStamped>("data", 10);

            // Advertise service for setting the bias
            srv_set_bias_ = priv_nh_.advertiseService("set_bias", &FTSensorPublisher::setBiasCallback, this);
        }
        else
        {
            throw std::runtime_error("FT sensor could not initialize");
        }
    }

    //! Empty stub
    ~FTSensorPublisher() {}

};

/* 每次启动自动设置bias vetor,用来消除运动时候的偏差 */
bool FTSensorPublisher::setBiasCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    ftsensor_->setBias();
}

void FTSensorPublisher::publishMeasurements()
    {
    // Recall that this has to be transformed using the stewart platform
    //tf_broadcaster_.sendTransform(tf::StampedTransform(nano_top_frame_, ros::Time::now(), "/world", "/nano_top_frame"));
    geometry_msgs::WrenchStamped ftreadings;
    float measurements[6];
    ftsensor_->getMeasurements(measurements);

    ftreadings.wrench.force.x = measurements[0];
    ftreadings.wrench.force.y = measurements[1];
    ftreadings.wrench.force.z = measurements[2];
    ftreadings.wrench.torque.x = measurements[3];
    ftreadings.wrench.torque.y = measurements[4];
    ftreadings.wrench.torque.z = measurements[5];

    ftreadings.header.stamp = ros::Time::now();
    ftreadings.header.frame_id = frame_ft_;

    pub_sensor_readings_.publish(ftreadings);

    // ROS_INFO("Measured Force: %f %f %f Measured Torque: %f %f %f", measurements[0], measurements[1], measurements[2], measurements[3], measurements[4], measurements[5]);
}

} // namespace ftsensor

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ft_sensor");
    ros::NodeHandle nh;
    ros::Rate loop(100);
    try
    {
        ftsensor::FTSensorPublisher node(nh);
        while(ros::ok())
        {
            node.publishMeasurements();
            ros::spinOnce();

            loop.sleep();
        }
    }
    catch (std::exception &ex) {
        ROS_FATAL_STREAM("Failed with error : " << ex.what());
        return -1;
    }
    return 0;
}
