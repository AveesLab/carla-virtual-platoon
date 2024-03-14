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
namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
using namespace std::chrono_literals;
using namespace std::string_literals;



class CarlaRGBCameraPublisher : public rclcpp::Node {

public:
    CarlaRGBCameraPublisher(boost::shared_ptr<carla::client::BlueprintLibrary> blueprint_library, boost::shared_ptr<carla::client::Actor> actor, carla::client::World& world_, std::string name_);
    ~CarlaRGBCameraPublisher() {
    camera->Destroy();
    }
private:
    boost::shared_ptr<carla::client::BlueprintLibrary> blueprint_library = nullptr;
    boost::shared_ptr<carla::client::Actor> actor = nullptr; 
    carla::client::World& world_;
    void publishImage(const csd::Image &carla_image);
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    boost::shared_ptr<carla::client::Sensor> camera;
    boost::shared_ptr<carla::client::Actor> cam_actor;
    carla::geom::Transform camera_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> camera_bp;

    float rgbcam_x;
    float rgbcam_y;
    float rgbcam_z;
    float rgbcam_pitch;
    float rgbcam_yaw;
    float rgbcam_roll;
    std::string rgbcam_sensor_tick;
    std::string rgbcam_topic_name;
    std::string role_name_;
    
};