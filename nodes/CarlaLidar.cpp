#include "CarlaLidar.hpp"

#include <boost/make_shared.hpp>

CarlaLidarPublisher::CarlaLidarPublisher(boost::shared_ptr<carla::client::BlueprintLibrary> blueprint_library, boost::shared_ptr<carla::client::Actor> actor,carla::client::World& world_)
    : Node("carla_lidar_publisher", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)),world_(world_) {

    rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    custom_qos.best_effort();

    this->blueprint_library = blueprint_library;
    this->actor = actor;


    lidar_bp = boost::shared_ptr<carla::client::ActorBlueprint>(
        const_cast<carla::client::ActorBlueprint*>(blueprint_library->Find("sensor.lidar.ray_cast"))
);

    
    this->get_parameter_or("lidar/x",lidar_x,2.3f);
    this->get_parameter_or("lidar/y",lidar_y,0.0f);
    this->get_parameter_or("lidar/z",lidar_z,1.0f);
    this->get_parameter_or("lidar/pitch",lidar_pitch, 0.0f);
    this->get_parameter_or("lidar/yaw",lidar_yaw,0.0f);
    this->get_parameter_or("lidar/roll",lidar_roll,0.0f);
    this->get_parameter_or("lidar/sensor_tick",lidar_sensor_tick,std::string("0.1f"));
    this->get_parameter_or("lidar/horizontal_fov",lidar_horizontal_fov,std::string("30.0f"));
    this->get_parameter_or("lidar/upper_fov",lidar_upper_fov,std::string("0.0f"));
    this->get_parameter_or("lidar/lower_fov",lidar_lower_fov,std::string("0.0f"));
    this->get_parameter_or("lidar/points_per_second",lidar_points_per_second,std::string("30000"));
    this->get_parameter_or("lidar/range",lidar_range,std::string("30.0f"));
    this->get_parameter_or("lidar_topic_name",lidar_topic_name,std::string("LV/carla/lidar"));
    

    lidar_bp->SetAttribute("sensor_tick", lidar_sensor_tick);
     std::cerr << lidar_horizontal_fov  << std::endl;
    lidar_bp->SetAttribute("horizontal_fov", lidar_horizontal_fov);
    std::cerr << lidar_points_per_second  << std::endl;
    lidar_bp->SetAttribute("points_per_second", lidar_points_per_second);
    lidar_bp->SetAttribute("upper_fov", lidar_upper_fov);
    lidar_bp->SetAttribute("lower_fov", lidar_lower_fov);
    lidar_bp->SetAttribute("range", lidar_range);
    lidar_bp->SetAttribute("channels","1");
    
    assert(lidar_bp != nullptr);
     publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic_name, custom_qos);
    lidar_transform = cg::Transform{
        cg::Location{lidar_x, lidar_y, lidar_z},   // x, y, z.
        cg::Rotation{lidar_pitch, lidar_yaw, lidar_roll}}; // pitch, yaw, roll.
    lidar_actor = world_.SpawnActor(*lidar_bp, lidar_transform, actor.get());
    lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);

    lidar->Listen([this](auto data) {
        auto lidar_data = boost::static_pointer_cast<carla::sensor::data::LidarMeasurement>(data);
        assert(lidar_data != nullptr);
        publishLidarData(lidar_data);
  });
}




void CarlaLidarPublisher::publishLidarData(const boost::shared_ptr<csd::LidarMeasurement> &carla_lidar_measurement)
    {
    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    point_cloud_msg.header.stamp = this->now();
    point_cloud_msg.header.frame_id = "lidar_frame";

    // Define fields
    point_cloud_msg.fields.resize(4);
    point_cloud_msg.fields[0].name = "x";
    point_cloud_msg.fields[0].offset = 0;
    point_cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg.fields[0].count = 1;

    point_cloud_msg.fields[1].name = "y";
    point_cloud_msg.fields[1].offset = 4;
    point_cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg.fields[1].count = 1;

    point_cloud_msg.fields[2].name = "z";
    point_cloud_msg.fields[2].offset = 8;
    point_cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg.fields[2].count = 1;

    point_cloud_msg.fields[3].name = "intensity";
    point_cloud_msg.fields[3].offset = 12;
    point_cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg.fields[3].count = 1;

    // Initialize a vector to hold the LiDAR data
    std::vector<float> lidar_data;
    // Assuming each detection is 4 floats (x, y, z, intensity)
    lidar_data.reserve(carla_lidar_measurement->size() * 4);

    // Iterate over each LiDAR detection
    for (const auto& detection : *carla_lidar_measurement) {
        if (detection.intensity < 0.96) {
          //  continue;
        }
        // Extract and process each point's data (x, y, z, intensity)
        lidar_data.push_back(detection.point.x);
        lidar_data.push_back(-detection.point.y); // Inverting Y-axis
        lidar_data.push_back(detection.point.z);
        lidar_data.push_back(detection.intensity);
     //   std::cerr << detection.intensity << std::endl;
    }

    // Assemble PointCloud2 message
    point_cloud_msg.height = 1;
    point_cloud_msg.width = lidar_data.size() / 4;
    point_cloud_msg.is_dense = false;
    point_cloud_msg.is_bigendian = false;
    point_cloud_msg.point_step = sizeof(float) * 4;
    point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;

    point_cloud_msg.data.resize(point_cloud_msg.row_step);
    memcpy(&point_cloud_msg.data[0], lidar_data.data(), lidar_data.size() * sizeof(float));

    publisher_->publish(point_cloud_msg);
 
    }

