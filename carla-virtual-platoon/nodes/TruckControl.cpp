#include "TruckControl.hpp"

TruckControl::TruckControl(boost::shared_ptr<carla::client::Vehicle> vehicle_, int num)
    : Node("truck_control_node", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)),Vehicle_(vehicle_) {
    
    this->get_parameter_or("steer_topic_name",steer_topic_name,std::string("steer"));
    this->get_parameter_or("velocity_topic_name",velocity_topic_name,std::string("velocity"));
    this->get_parameter_or("carla/sync",sync_,true);

    if(sync_) SyncThrottlePublisher_ = this->create_publisher<std_msgs::msg::Int32>("/sync_throttle",10);
    if(sync_) SyncSteerPublisher_ = this->create_publisher<std_msgs::msg::Int32>("/sync_steer",10);
    SteerSubscriber_ = this->create_subscription<std_msgs::msg::Float32>(steer_topic_name, 1, std::bind(&TruckControl::SteerSubCallback, this, std::placeholders::_1));
    VelocitySubscriber_ = this->create_subscription<std_msgs::msg::Float32>(velocity_topic_name, 1, std::bind(&TruckControl::VelocitySubCallback, this, std::placeholders::_1));
    
    this->trucknum = num; 
    this->control.hand_brake = false;
    Vehicle_->ApplyControl(control);
}


void TruckControl::SteerSubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    float control_value = ((msg->data * -1.0) / 140) * 2 ;
    if (control_value > 1.0) this->control.steer = 1.0f;
    else if (control_value < -1.0) this->control.steer = -1.0f;
    else this->control.steer = control_value;
    Vehicle_->ApplyControl(control);
    if(sync_) {
        std_msgs::msg::Int32 msg;
        msg.data = this->trucknum;
        SyncSteerPublisher_->publish(msg);
        std::cout << "sync pub" << std::endl;     
    }
}


void TruckControl::VelocitySubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    float control_value = msg->data;
    if (control_value >= 0) {
        this->control.throttle = control_value;
        this->control.brake = 0;
    }
    else if (control_value < 0) {
        this->control.throttle = 0;
        this->control.brake = -control_value;
    }
    Vehicle_->ApplyControl(control);
    
    if(sync_) {
        std_msgs::msg::Int32 msg;
        msg.data = this->trucknum;
        SyncThrottlePublisher_->publish(msg);
        std::cout << "throttle pub" << std::endl;     
    }
}
