#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>


class TruckStatusPublisher : public rclcpp::Node {

public:
    TruckStatusPublisher(boost::shared_ptr<carla::client::Vehicle> vehicle_);

private:
    rclcpp::TimerBase::SharedPtr timer_1ms_accel;
    rclcpp::TimerBase::SharedPtr timer_1ms_velocity;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr AccelPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr VelocityPublisher_;
    boost::shared_ptr<carla::client::Vehicle> Vehicle_;
    void TruckStatusPublisher_accel_callback();
    void TruckStatusPublisher_velocity_callback();
    std::string info_topic_name;
};