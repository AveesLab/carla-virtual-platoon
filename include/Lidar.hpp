#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>


class LidarPublisher : public rclcpp::Node {

public:
    LidarPublisher(boost::shared_ptr<carla::client::Actor> actor);
    ~LidarPublisher(){
        lidar->Destroy();
  }
private:
    void publishLidarData(const boost::shared_ptr<csd::LidarMeasurement> &lidar_data, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> publishers_;
    std::vector<boost::shared_ptr<cc::Sensor>> lidar_sensors;

    boost::shared_ptr<carla::client::Sensor> lidar;
    boost::shared_ptr<carla::client::Actor> lidar_actor;
    carla::geom::Transform lidar_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> lidar_bp;

    int num_lidars_;

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
