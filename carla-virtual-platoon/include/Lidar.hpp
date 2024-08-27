#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>
#include <boost/shared_ptr.hpp>

struct TimedLidar {
    boost::shared_ptr<csd::LidarMeasurement> lidar;
    int timestamp;

    TimedLidar(boost::shared_ptr<csd::LidarMeasurement> lidar_data, int ts) : lidar(lidar_data), timestamp(ts) {}
};

class LidarPublisher : public rclcpp::Node {

public:
    LidarPublisher(boost::shared_ptr<carla::client::Actor> actor);
    ~LidarPublisher(){
        for(auto lid  :lidar_sensors) {
            lid->Destroy();
        }
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
    std::mutex mutex_;
    bool sync_ = false;
    bool sync_with_delay = false;
    void GetDelayParameter();
    int gcd(int a, int b);
    int lcm(int a, int b);
    int lcm_period;
    int tick_cnt = 0;
    int cnt = 0;
    int velocity_planner_period = 30;
    int velocity_planner_delay = 100;
    std::vector<std::queue<TimedLidar>> velocity_lidar_queue;

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
