#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <carla/sensor/data/LidarMeasurement.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
using namespace std::chrono_literals;
using namespace std::string_literals;




class CarlaLidarPublisher : public rclcpp::Node {

public:
    CarlaLidarPublisher(boost::shared_ptr<carla::client::BlueprintLibrary> blueprint_library, boost::shared_ptr<carla::client::Actor> actor, carla::client::World& world_);
    ~CarlaLidarPublisher(){
        lidar->Destroy();
  }
private:
    boost::shared_ptr<carla::client::BlueprintLibrary> blueprint_library = nullptr;
    boost::shared_ptr<carla::client::Actor> actor = nullptr; 
    carla::client::World& world_;
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
};