#include "shared_carlalib.h"





class CarlaRadarPublisher : public rclcpp::Node {

public:
    CarlaRadarPublisher(boost::shared_ptr<carla::client::BlueprintLibrary> blueprint_library, boost::shared_ptr<carla::client::Actor> actor, carla::client::World& world_,std::string name_);
    ~CarlaRadarPublisher(){
        radar->Destroy();
  }
private:
    boost::shared_ptr<carla::client::BlueprintLibrary> blueprint_library = nullptr;
    boost::shared_ptr<carla::client::Actor> actor = nullptr; 
    carla::client::World& world_;
    void publishRadarData(const boost::shared_ptr<csd::RadarMeasurement> &radar_data);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    boost::shared_ptr<carla::client::Sensor> radar;
    boost::shared_ptr<carla::client::Actor> radar_actor;
    carla::geom::Transform radar_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> radar_bp;

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