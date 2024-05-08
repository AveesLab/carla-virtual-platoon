#include "TruckStatus.hpp"

TruckStatusPublisher::TruckStatusPublisher(boost::shared_ptr<carla::client::Vehicle> vehicle_)
    : Node("truck_status_node", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)),Vehicle_(vehicle_) {

    this->get_parameter_or("info_topic_name",info_topic_name,std::string("velocity_info"));
    AccelPublisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("accel",1);
    VelocityPublisher_ = this->create_publisher<std_msgs::msg::Float32>(info_topic_name,1);
    timer_1ms_accel = this->create_wall_timer(1ms, std::bind(&TruckStatusPublisher::TruckStatusPublisher_accel_callback, this));
    timer_1ms_velocity = this->create_wall_timer(1ms, std::bind(&TruckStatusPublisher::TruckStatusPublisher_velocity_callback, this));
    
}

void TruckStatusPublisher::TruckStatusPublisher_accel_callback() {
    carla::geom::Vector3D acc_ = Vehicle_->GetAcceleration();
    auto message = std_msgs::msg::Float32MultiArray();
    message.data.push_back(acc_.x);
    message.data.push_back(acc_.y);
    message.data.push_back(acc_.z);
    AccelPublisher_->publish(message);
}


void TruckStatusPublisher::TruckStatusPublisher_velocity_callback() {
    carla::geom::Vector3D vel_ = Vehicle_->GetVelocity();
    auto message = std_msgs::msg::Float32();
    float result = std::sqrt(std::pow(vel_.x,2)  + std::pow(vel_.y,2) + std::pow(vel_.z,2)); // m/s
    message.data = result;
    VelocityPublisher_->publish(message);
}