#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>


class CarlaVehicleController : public rclcpp::Node {

public:
    CarlaVehicleController(boost::shared_ptr<carla::client::Vehicle> vehicle_);

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr SteerSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr VelocitySubscriber_;
    rclcpp::TimerBase::SharedPtr timer_1ms;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr VelocityPublisher_;
    void SteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void VelocitySubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    boost::shared_ptr<carla::client::Vehicle> Vehicle_;
    carla::rpc::VehicleControl control;
    void VelocityPublisher_callback();
    std::string info_topic_name;
    std::string steer_topic_name;
    std::string velocity_topic_name;
};