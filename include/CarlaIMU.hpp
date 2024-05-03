#include "shared_carlalib.h"



class CarlaIMUPublisher : public rclcpp::Node {

public:
    CarlaIMUPublisher(boost::shared_ptr<carla::client::Vehicle> vehicle_,std::string name_);

private:
    rclcpp::TimerBase::SharedPtr timer_1ms;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr IMUPublisher_;
    boost::shared_ptr<carla::client::Vehicle> Vehicle_;
    void CarlaIMUPublisher_callback();
    std::string role_name_;
};