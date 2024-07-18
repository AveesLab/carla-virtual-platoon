#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>

class FrontRadarPublisher : public rclcpp::Node {

public:
    FrontRadarPublisher(boost::shared_ptr<carla::client::Actor> actor);
    ~FrontRadarPublisher(){
        radar->Destroy();
  }
private:
    void publishRadarData(const boost::shared_ptr<csd::RadarMeasurement> &radar_data, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher);
//    void publishRadarData(boost::shared_ptr<carla::sensor::data::RadarMeasurement> radar_data, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher) ;
//    void publishRadarData(const boost::shared_ptr<csd::RadarMeasurement> &radar_data);
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> publishers_;
    std::vector<boost::shared_ptr<cc::Sensor>> radar_sensors;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;

    boost::shared_ptr<carla::client::Sensor> radar;
    boost::shared_ptr<carla::client::Actor> radar_actor;
    carla::geom::Transform radar_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> radar_bp;

    int num_radars_;

    float radar_x;
    float radar_y;
    float radar_z;
    float radar_pitch;
    float radar_yaw;
    float radar_roll;
    std::string radar_sensor_tick;
    std::string radar_horizontal_fov;
    std::string radar_vertical_fov;
    std::string radar_points_per_second;
    std::string radar_range;
    std::string radar_topic_name;
    std::string role_name_;
    bool f_radar_;

    //left_radar
    void publishLeftRadarData(const boost::shared_ptr<csd::RadarMeasurement> &carla_radar_measurement);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_radar_publisher_;

    boost::shared_ptr<carla::client::Sensor> left_radar;
    boost::shared_ptr<carla::client::Actor> left_radar_actor;
    carla::geom::Transform left_radar_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> left_radar_bp;

    float left_radar_x;
    float left_radar_y;
    float left_radar_z;
    float left_radar_pitch;
    float left_radar_yaw;
    float left_radar_roll;
    std::string left_radar_sensor_tick;
    std::string left_radar_horizontal_fov;
    std::string left_radar_vertical_fov;
    std::string left_radar_points_per_second;
    std::string left_radar_range;
    std::string left_radar_topic_name;
    bool l_radar_;

    //right_radar
    void publishRightRadarData(const boost::shared_ptr<csd::RadarMeasurement> &carla_radar_measurement);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_radar_publisher_;

    boost::shared_ptr<carla::client::Sensor> right_radar;
    boost::shared_ptr<carla::client::Actor> right_radar_actor;
    carla::geom::Transform right_radar_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> right_radar_bp;

    float right_radar_x;
    float right_radar_y;
    float right_radar_z;
    float right_radar_pitch;
    float right_radar_yaw;
    float right_radar_roll;
    std::string right_radar_sensor_tick;
    std::string right_radar_horizontal_fov;
    std::string right_radar_vertical_fov;
    std::string right_radar_points_per_second;
    std::string right_radar_range;
    std::string right_radar_topic_name;
    bool r_radar_;
};
