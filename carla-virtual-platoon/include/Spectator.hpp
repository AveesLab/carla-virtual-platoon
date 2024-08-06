#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/bool.hpp>

class Spectator : public rclcpp::Node {

public:
    Spectator(boost::shared_ptr<carla::client::Actor> actor);
    ~Spectator() {
        camera->Destroy();
    }
private:
int tmp;
boost::shared_ptr<carla::client::Actor> actor_;
    void publishImage(const csd::Image &carla_image);
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    boost::shared_ptr<carla::client::Sensor> camera;
    boost::shared_ptr<carla::client::Actor> cam_actor;
    carla::geom::Transform camera_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> camera_bp;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Att_subscriber_;
    void AttSubCallback(const std_msgs::msg::Bool::SharedPtr msg );
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
    bool spectator_;
};
