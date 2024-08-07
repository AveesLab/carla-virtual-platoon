#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/bool.hpp>

#include <boost/shared_ptr.hpp>

struct TimedImage {
    boost::shared_ptr<csd::Image> image;
    int timestamp;
};



class CameraPublisher : public rclcpp::Node {

public:
    CameraPublisher(boost::shared_ptr<carla::client::Actor> actor);
    ~CameraPublisher() {
        camera->Destroy();
    }
private:

    int tmp;
    boost::shared_ptr<carla::client::Actor> actor_;
//    void publishImage(const csd::Image &carla_image);
    void publishImage(const csd::Image &carla_image, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher);
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr LaneImagePublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr WaitLanePublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr WaitVelPublisher;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
    std::vector<boost::shared_ptr<cc::Sensor>> camera_sensors; 


    boost::shared_ptr<carla::client::Sensor> camera;
    boost::shared_ptr<carla::client::Actor> camera_actor;
    carla::geom::Transform camera_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> camera_bp;

    void publishImage1(const csd::Image &carla_image);

    int num_cameras_;
    bool sync_ = false;
    bool sync_with_delay = false;
    int tick_cnt = 0;
    int velocity_planner_period = 30;
    int velocity_planner_delay = 100;
    int path_planner_period = 30;
    int path_planner_delay = 100;
    std::vector<std::queue<TimedImage>> velocity_image_queue;
    std::queue<TimedImage> path_image_queue;
    void GetDelayParameter();
    int lcm_period;
    int gcd(int a, int b);
    int lcm(int a, int b);
    void wait_for_velocity(bool ch_);
    void wait_for_lane(bool ch_);

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
