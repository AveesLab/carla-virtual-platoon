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
    ShutdownSubscriber = this->create_subscription<std_msgs::msg::String>("/shutdown_topic", 10, std::bind(&TruckStatusPublisher::shutdown_callback, this, std::placeholders::_1));
    DistanceSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("min_distance", 10, std::bind(&TruckStatusPublisher::DistanceSubCallback, this, std::placeholders::_1));

    timer_10ms_record = this->create_wall_timer(10ms, std::bind(&TruckStatusPublisher::TruckStatus_record_callback, this));
}

void TruckStatusPublisher::TruckStatusPublisher_accel_callback() {
    acc_ = Vehicle_->GetAcceleration();
    auto message = std_msgs::msg::Float32MultiArray();
    message.data.push_back(acc_.x);
    message.data.push_back(acc_.y);
    message.data.push_back(acc_.z);
    float result_acc = std::sqrt(std::pow(acc_.x,2)  + std::pow(acc_.y,2) + std::pow(acc_.z,2)); // m/s
    acceleration_ = result_acc;
    AccelPublisher_->publish(message);
}


void TruckStatusPublisher::TruckStatusPublisher_velocity_callback() {
    vel_ = Vehicle_->GetVelocity();
    auto message = std_msgs::msg::Float32();
    float result_vel = std::sqrt(std::pow(vel_.x,2)  + std::pow(vel_.y,2) + std::pow(vel_.z,2)); // m/s
    message.data = result_vel;
    velocity_ = result_vel;
    VelocityPublisher_->publish(message);
}

void TruckStatusPublisher::DistanceSubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    distance_ = msg->data;
}

void TruckStatusPublisher::shutdown_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received shutdown message: '%s'", msg->data.c_str());
    rclcpp::shutdown(); // 종료 명령
}

void TruckStatusPublisher::TruckStatus_record_callback() {
    gettimeofday(&init_, NULL);
    if(velocity_ != 0.) recordData(init_);
}

void TruckStatusPublisher::recordData(struct timeval startTime){
    struct timeval currentTime;
    char file_name[] = "SCT_log00.csv";
    static char file[128] = {0x00, };
    char buf[256] = {0x00,};
    static bool flag = false;
    double diff_time;
    std::ifstream read_file;
    std::ofstream write_file;
    if(!flag){
      for(int i = 0; i < 100; i++){
        file_name[7] = i/10 + '0';  //ASCII
        file_name[8] = i%10 + '0';
        sprintf(file, "%s%s", log_path_.c_str(), file_name);
        read_file.open(file);
        if(read_file.fail()){  //Check if the file exists
          read_file.close();
          write_file.open(file);
          break;
        }
        read_file.close();
      }
      write_file << "Time, Velocity, Acceleration, Distance" << std::endl; 
      flag = true;
    }
    if(flag){
  //    std::scoped_lock lock(lane_mutex_, rlane_mutex_, vel_mutex_, dist_mutex_, rep_mutex_);
      gettimeofday(&currentTime, NULL);
      diff_time = ((currentTime.tv_sec - startTime.tv_sec)) + ((currentTime.tv_usec - startTime.tv_usec)/1000000.0);
      sprintf(buf, "%.10e, %.3f, %.3f, %.3f", diff_time, velocity_, acceleration_, distance_);
      write_file.open(file, std::ios::out | std::ios::app);
      write_file << buf << std::endl;
    }
    write_file.close();
}

