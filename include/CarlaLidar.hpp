#include "shared_carlalib.h"



class CarlaLidarPublisher : public rclcpp::Node {

public:
    CarlaLidarPublisher(boost::shared_ptr<carla::client::Actor> actor,int num);
    ~CarlaLidarPublisher(){
        lidar->Destroy();
  }
private:
    boost::shared_ptr<carla::client::Actor> actor = nullptr; 
    void publishLidarData(const boost::shared_ptr<csd::LidarMeasurement> &lidar_data);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    boost::shared_ptr<carla::client::Sensor> lidar;
    boost::shared_ptr<carla::client::Actor> lidar_actor;
    carla::geom::Transform lidar_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> lidar_bp;

    float lidar_x;
    float lidar_y;
    float lidar_z;
    float lidar_pitch;
    float lidar_yaw;
    float lidar_roll;
    std::string lidar_sensor_tick;
    std::string lidar_horizontal_fov;
    std::string lidar_upper_fov;
    std::string lidar_lower_fov;
    std::string lidar_points_per_second;
    std::string lidar_range;
    std::string lidar_topic_name;
    std::string lidar_rotation_frequency;
};