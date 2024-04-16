#include "CarlaVehicle.hpp"
#include <boost/make_shared.hpp>

CarlaVehicleController::CarlaVehicleController(boost::shared_ptr<carla::client::Vehicle> vehicle_)
    : Node("carla_lidar_publisher", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)),Vehicle_(vehicle_) {






            VelocityPublisher_ = this->create_publisher<std_msgs::msg::Float32>("velocity_info",1);
            timer_1ms = this->create_wall_timer(
            1ms, std::bind(&CarlaVehicleController::VelocityPublisher_callback, this));

            
            SteerSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("steer", 1, std::bind(&CarlaVehicleController::SteerSubCallback, this, std::placeholders::_1));
            VelocitySubscriber_ = this->create_subscription<std_msgs::msg::Float32>("velocity", 1, std::bind(&CarlaVehicleController::VelocitySubCallback, this, std::placeholders::_1));

            this->control.hand_brake = true;
            Vehicle_->ApplyControl(control);
           }


void CarlaVehicleController::SteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    float control_value = ((msg->data * -1.0) / 140) * 2 ;
    if (control_value > 1.0) this->control.steer = 1.0f;
    else if (control_value < -1.0) this->control.steer = -1.0f;
    else this->control.steer = control_value;
//if (msg->data < 8.0f && msg->data > -8.0f) this->control.steer = 0;
    Vehicle_->ApplyControl(control);
}

void CarlaVehicleController::VelocityPublisher_callback() {
    carla::geom::Vector3D vel_ = Vehicle_->GetVelocity();
    auto message = std_msgs::msg::Float32();
    float result = 3.6 * std::sqrt(std::pow(vel_.x,2)  + std::pow(vel_.y,2) + std::pow(vel_.z,2)); // km/h
    message.data = result;
    VelocityPublisher_->publish(message);
}

void CarlaVehicleController::VelocitySubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    float control_value = msg->data;
    if(this->control.hand_brake == true ) {
        if (control_value > 0.5) this->control.hand_brake = false;
    }
    else {
    if (control_value >= 0) 
    {
        this->control.throttle = control_value;
        this->control.brake = 0;
    }
    else if (control_value < 0) 
    {
    this->control.throttle = 0;
    this->control.brake = -control_value;
    }
    Vehicle_->ApplyControl(control);
    }
}
