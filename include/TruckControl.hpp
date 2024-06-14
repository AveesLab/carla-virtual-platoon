#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>

class TruckControl : public rclcpp::Node {

public:
    TruckControl(boost::shared_ptr<carla::client::Vehicle> vehicle_);

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr SteerSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ThrottleSubscriber_;
    void SteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void ThrottleSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    boost::shared_ptr<carla::client::Vehicle> Vehicle_;
    carla::rpc::VehicleControl control;
    std::string steer_topic_name;
    std::string throttle_topic_name;
    bool initial_hand_brake;
};