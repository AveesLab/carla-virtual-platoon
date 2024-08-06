#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>

class RadarPublisher : public rclcpp::Node {

public:
    RadarPublisher(boost::shared_ptr<carla::client::Actor> actor);
    ~RadarPublisher(){
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
    bool sync_ = false;
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
};
