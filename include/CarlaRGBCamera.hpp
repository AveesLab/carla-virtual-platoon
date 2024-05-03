#include "shared_carlalib.h"



class CarlaRGBCameraPublisher : public rclcpp::Node {

public:
    CarlaRGBCameraPublisher(boost::shared_ptr<carla::client::Actor> actor, int num);
    ~CarlaRGBCameraPublisher() {
        camera->Destroy();
    }
private:
    boost::shared_ptr<carla::client::Actor> actor = nullptr; 
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
    
};