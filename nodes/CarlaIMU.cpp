#include "CarlaIMU.hpp"
#include <boost/make_shared.hpp>

CarlaIMUPublisher::CarlaIMUPublisher(boost::shared_ptr<carla::client::Vehicle> vehicle_,std::string name_)
    : Node("carla_lidar_publisher", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)),Vehicle_(vehicle_) {

            IMUPublisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(name_ + "accel",1);
            timer_1ms = this->create_wall_timer(
            1ms, std::bind(&CarlaIMUPublisher::CarlaIMUPublisher_callback, this));
           }

void CarlaIMUPublisher::CarlaIMUPublisher_callback() {
    carla::geom::Vector3D acc_ = Vehicle_->GetAcceleration();
    auto message = std_msgs::msg::Float32MultiArray();
    message.data.push_back(acc_.x);
    message.data.push_back(acc_.y);
    message.data.push_back(acc_.z);
    IMUPublisher_->publish(message);
}