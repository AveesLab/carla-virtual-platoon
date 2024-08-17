#include "shared_carlalib.h"
#include <boost/make_shared.hpp>
#include <rclcpp/qos.hpp>
#include <boost/shared_ptr.hpp>

struct TimedRadar {
    boost::shared_ptr<csd::RadarMeasurement> radar;
    int timestamp;

    TimedRadar(boost::shared_ptr<csd::RadarMeasurement> radar_data, int ts) : radar(radar_data), timestamp(ts) {}
};

class RadarPublisher : public rclcpp::Node {

public:
    RadarPublisher(boost::shared_ptr<carla::client::Actor> actor);
    ~RadarPublisher(){
        for(auto rad : radar_sensors) {
            rad->Destroy();
        }
  }
private:
    void publishRadarData(const boost::shared_ptr<csd::RadarMeasurement> &radar_data, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher);
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> publishers_;
    std::vector<boost::shared_ptr<cc::Sensor>> radar_sensors;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;

    boost::shared_ptr<carla::client::Sensor> radar;
    boost::shared_ptr<carla::client::Actor> radar_actor;
    carla::geom::Transform radar_transform;
    boost::shared_ptr<carla::client::ActorBlueprint> radar_bp;
    std::mutex mutex_;
    bool sync_ = false;
    bool sync_with_delay = false;
    void GetDelayParameter();
    int gcd(int a, int b);
    int lcm(int a, int b);
    int lcm_period;
    int tick_cnt = 0;
    int velocity_planner_period = 30;
    int velocity_planner_delay = 100;
    std::vector<std::queue<TimedRadar>> velocity_radar_queue;
    int cnt = 0;

    int num_radars_;
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
