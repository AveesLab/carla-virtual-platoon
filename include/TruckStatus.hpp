#include <iostream>
#include <fstream>
#include <string>
#include <sys/time.h>
#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"


class TruckStatusPublisher : public rclcpp::Node {

public:
    TruckStatusPublisher(boost::shared_ptr<carla::client::Vehicle> vehicle_);

    float velocity_ = 0.f;
    float acceleration_ = 0.f;
    float distance_ = 0.f;
    struct timeval init_;
    std::string log_path_;
    carla::geom::Vector3D vel_, acc_;
private:
    rclcpp::TimerBase::SharedPtr timer_1ms_accel;
    rclcpp::TimerBase::SharedPtr timer_1ms_velocity;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr AccelPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr VelocityPublisher_;
    boost::shared_ptr<carla::client::Vehicle> Vehicle_;
    void TruckStatusPublisher_accel_callback();
    void TruckStatusPublisher_velocity_callback();
    void shutdown_callback(const std_msgs::msg::String::SharedPtr msg);
    std::string info_topic_name;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ShutdownSubscriber;

    void DistanceSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_100ms_record;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr DistanceSubscriber_;
    void TruckStatus_record_callback();
    void recordData(struct timeval startTime);

};
