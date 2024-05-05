#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>


class CarlaRGBCameraPublisher : public rclcpp::Node {

public:
    CarlaRGBCameraPublisher(boost::shared_ptr<carla::client::Actor> actor);
    ~CarlaRGBCameraPublisher() {
        camera->Destroy();
    }
private:
    void publishImage(const csd::Image &carla_image);
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    boost::shared_ptr<carla::client::Sensor> camera;
    boost::shared_ptr<carla::client::Actor> cam_actor;
    carla::geom::Transform camera_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> camera_bp;
    void publishImage1(const csd::Image &carla_image);
    float rgbcam_x;
    float rgbcam_y;
    float rgbcam_z;
    float rgbcam_pitch;
    float rgbcam_yaw;
    float rgbcam_roll;
    std::string rgbcam_sensor_tick;
    std::string rgbcam_topic_name;
    std::string role_name_;
    std::string rgbcam_image_size_x;
    std::string rgbcam_image_size_y;
    std::string rgbcam_fov;
};