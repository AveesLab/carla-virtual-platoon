#include <iostream>
#include <fstream>
#include <string>
#include <sys/time.h>
#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ros2_msg/msg/imu.hpp"
#include "ros2_msg/msg/gnss.hpp"

class TruckStatusPublisher : public rclcpp::Node {

public:
    TruckStatusPublisher(boost::shared_ptr<carla::client::Vehicle> vehicle_,boost::shared_ptr<carla::client::Actor> actor);

    float velocity_ = 0.f;
    float acceleration_ = 0.f;
    float distance_ = 0.f;
    struct timeval init_;
    std::string log_path_;
    carla::geom::Vector3D vel_, acc_;
    float cut_in_flag = 0.0f;
    bool check = false;
    float sotif_flag = 0.0f;
private:
    boost::shared_ptr<carla::client::Actor> actor_;
    rclcpp::TimerBase::SharedPtr timer_1ms_accel;
    rclcpp::TimerBase::SharedPtr timer_1ms_velocity;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr AccelPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr VelocityPublisher_;
    rclcpp::Publisher<ros2_msg::msg::IMU>::SharedPtr IMUPublisher_;
    rclcpp::Publisher<ros2_msg::msg::GNSS>::SharedPtr GnssPublisher_;
    boost::shared_ptr<carla::client::Vehicle> Vehicle_;
    void TruckStatusPublisher_accel_callback();
    void TruckStatusPublisher_velocity_callback();
    void shutdown_callback(const std_msgs::msg::String::SharedPtr msg);
    std::string info_topic_name;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ShutdownSubscriber;
    void SotifSubCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void DistanceSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void CutinFlagSubCallback(const std_msgs::msg::Bool::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_100ms_record;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr DistanceSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr CutinFlagSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr SotifSubscriber_;
    void TruckStatus_record_callback();
    void recordData(struct timeval startTime);
    void generate_imu();
    void generate_gnss();

    boost::shared_ptr<carla::client::Sensor> imu;
    boost::shared_ptr<carla::client::Actor> imu_actor;
    carla::geom::Transform imu_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> imu_bp;

    boost::shared_ptr<carla::client::Sensor> gnss;
    boost::shared_ptr<carla::client::Actor> gnss_actor;
    carla::geom::Transform gnss_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> gnss_bp;

    void publishIMUData(const boost::shared_ptr<csd::IMUMeasurement> &carla_imu_measurement);
    void publishGnssData(const boost::shared_ptr<csd::GnssMeasurement> &carla_gnss_measurement);
};
