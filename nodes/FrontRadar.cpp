#include "FrontRadar.hpp"

FrontRadarPublisher::FrontRadarPublisher(boost::shared_ptr<carla::client::Actor> actor)
    : Node("front_radar_node", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)) {

    rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    custom_qos.best_effort();

    this->get_parameter_or("radar/x",radar_x,2.3f);
    this->get_parameter_or("radar/y",radar_y,0.0f);
    this->get_parameter_or("radar/z",radar_z,1.5f);
    this->get_parameter_or("radar/pitch",radar_pitch, 0.0f);
    this->get_parameter_or("radar/yaw",radar_yaw,0.0f);
    this->get_parameter_or("radar/roll",radar_roll,0.0f);
    this->get_parameter_or("radar/sensor_tick",radar_sensor_tick,std::string("0.1f"));
    this->get_parameter_or("radar/horizontal_fov",radar_horizontal_fov,std::string("40.0f"));
    this->get_parameter_or("radar/vertical_fov",radar_vertical_fov,std::string("30.0f"));
    this->get_parameter_or("radar/points_per_second",radar_points_per_second,std::string("8000"));
    this->get_parameter_or("radar/range",radar_range,std::string("300.0f"));
    this->get_parameter_or("radar_topic_name",radar_topic_name,std::string("carla/radar"));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(radar_topic_name, custom_qos);

    radar_bp = boost::shared_ptr<carla::client::ActorBlueprint>( const_cast<carla::client::ActorBlueprint*>(blueprint_library->Find("sensor.other.radar")));
    assert(radar_bp != nullptr);
    radar_bp->SetAttribute("sensor_tick", radar_sensor_tick);
    radar_bp->SetAttribute("horizontal_fov", radar_horizontal_fov);
    radar_bp->SetAttribute("points_per_second", radar_points_per_second);
    radar_bp->SetAttribute("vertical_fov", radar_vertical_fov);
    radar_bp->SetAttribute("range", radar_range);

    radar_transform = cg::Transform{cg::Location{radar_x, radar_y, radar_z}, cg::Rotation{radar_pitch, radar_yaw, radar_roll}}; // pitch, yaw, roll.
    radar_actor = world->SpawnActor(*radar_bp, radar_transform, actor.get());
    radar = boost::static_pointer_cast<cc::Sensor>(radar_actor);

    radar->Listen([this](auto data) {
        auto radar_data = boost::static_pointer_cast<carla::sensor::data::RadarMeasurement>(data);
        assert(radar_data != nullptr);
        publishRadarData(radar_data);
    });

    //left-side radar
    this->get_parameter_or("left_radar/x",left_radar_x,2.3f);
    this->get_parameter_or("left_radar/y",left_radar_y,0.0f);
    this->get_parameter_or("left_radar/z",left_radar_z,1.5f);
    this->get_parameter_or("left_radar/pitch",left_radar_pitch, 0.0f);
    this->get_parameter_or("left_radar/yaw",left_radar_yaw,0.0f);
    this->get_parameter_or("left_radar/roll",left_radar_roll,0.0f);
    this->get_parameter_or("left_radar/sensor_tick",left_radar_sensor_tick,std::string("0.1f"));
    this->get_parameter_or("left_radar/horizontal_fov",left_radar_horizontal_fov,std::string("40.0f"));
    this->get_parameter_or("left_radar/vertical_fov",left_radar_vertical_fov,std::string("30.0f"));
    this->get_parameter_or("left_radar/points_per_second",left_radar_points_per_second,std::string("8000"));
    this->get_parameter_or("left_radar/range",left_radar_range,std::string("300.0f"));
    this->get_parameter_or("left_radar_topic_name",left_radar_topic_name,std::string("carla/radar"));

    left_radar_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(left_radar_topic_name, custom_qos);

    left_radar_bp = boost::shared_ptr<carla::client::ActorBlueprint>( const_cast<carla::client::ActorBlueprint*>(blueprint_library->Find("sensor.other.radar")));
    assert(left_radar_bp != nullptr);
    left_radar_bp->SetAttribute("sensor_tick", left_radar_sensor_tick);
    left_radar_bp->SetAttribute("horizontal_fov", left_radar_horizontal_fov);
    left_radar_bp->SetAttribute("points_per_second", left_radar_points_per_second);
    left_radar_bp->SetAttribute("vertical_fov", left_radar_vertical_fov);
    left_radar_bp->SetAttribute("range", left_radar_range);

    left_radar_transform = cg::Transform{cg::Location{left_radar_x, left_radar_y, left_radar_z}, cg::Rotation{left_radar_pitch, left_radar_yaw, left_radar_roll}}; // pitch, yaw, roll.
    left_radar_actor = world->SpawnActor(*left_radar_bp, left_radar_transform, actor.get());
    left_radar = boost::static_pointer_cast<cc::Sensor>(left_radar_actor);

    left_radar->Listen([this](auto data) {
        auto radar_data = boost::static_pointer_cast<carla::sensor::data::RadarMeasurement>(data);
        assert(radar_data != nullptr);
        publishLeftRadarData(radar_data);
    });

    //right-side radar
    this->get_parameter_or("right_radar/x",right_radar_x,2.3f);
    this->get_parameter_or("right_radar/y",right_radar_y,0.0f);
    this->get_parameter_or("right_radar/z",right_radar_z,1.5f);
    this->get_parameter_or("right_radar/pitch",right_radar_pitch, 0.0f);
    this->get_parameter_or("right_radar/yaw",right_radar_yaw,0.0f);
    this->get_parameter_or("right_radar/roll",right_radar_roll,0.0f);
    this->get_parameter_or("right_radar/sensor_tick",right_radar_sensor_tick,std::string("0.1f"));
    this->get_parameter_or("right_radar/horizontal_fov",right_radar_horizontal_fov,std::string("40.0f"));
    this->get_parameter_or("right_radar/vertical_fov",right_radar_vertical_fov,std::string("30.0f"));
    this->get_parameter_or("right_radar/points_per_second",right_radar_points_per_second,std::string("8000"));
    this->get_parameter_or("right_radar/range",right_radar_range,std::string("300.0f"));
    this->get_parameter_or("right_radar_topic_name",right_radar_topic_name,std::string("carla/radar"));

    right_radar_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(right_radar_topic_name, custom_qos);

    right_radar_bp = boost::shared_ptr<carla::client::ActorBlueprint>( const_cast<carla::client::ActorBlueprint*>(blueprint_library->Find("sensor.other.radar")));
    assert(right_radar_bp != nullptr);
    right_radar_bp->SetAttribute("sensor_tick", right_radar_sensor_tick);
    right_radar_bp->SetAttribute("horizontal_fov", right_radar_horizontal_fov);
    right_radar_bp->SetAttribute("points_per_second", right_radar_points_per_second);
    right_radar_bp->SetAttribute("vertical_fov", right_radar_vertical_fov);
    right_radar_bp->SetAttribute("range", right_radar_range);

    right_radar_transform = cg::Transform{cg::Location{right_radar_x, right_radar_y, right_radar_z}, cg::Rotation{right_radar_pitch, right_radar_yaw, right_radar_roll}}; // pitch, yaw, roll.
    right_radar_actor = world->SpawnActor(*right_radar_bp, right_radar_transform, actor.get());
    right_radar = boost::static_pointer_cast<cc::Sensor>(right_radar_actor);

    right_radar->Listen([this](auto data) {
        auto radar_data = boost::static_pointer_cast<carla::sensor::data::RadarMeasurement>(data);
        assert(radar_data != nullptr);
        publishRightRadarData(radar_data);
    });
}
void FrontRadarPublisher::publishRightRadarData(const boost::shared_ptr<csd::RadarMeasurement> &carla_radar_measurement) {
    sensor_msgs::msg::PointCloud2 radar_msg;
    radar_msg.header.stamp = this->now();
    radar_msg.header.frame_id = "laser";

    radar_msg.height = 1;
    radar_msg.width = carla_radar_measurement->GetDetectionAmount();
    radar_msg.is_dense = false;
    radar_msg.is_bigendian = false;

    std::vector<sensor_msgs::msg::PointField> fields(7);

    // Define PointFields for x, y, z, Range, Velocity, AzimuthAngle, and ElevationAngle
    fields[0].name = "x";
    fields[0].offset = 0;
    fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[0].count = 1;

    fields[1].name = "y";
    fields[1].offset = 4;
    fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[1].count = 1;

    fields[2].name = "z";
    fields[2].offset = 8;
    fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[2].count = 1;

    fields[3].name = "Range";
    fields[3].offset = 12;
    fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[3].count = 1;

    fields[4].name = "Velocity";
    fields[4].offset = 16;
    fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[4].count = 1;

    fields[5].name = "AzimuthAngle";
    fields[5].offset = 20;
    fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[5].count = 1;

    fields[6].name = "ElevationAngle";
    fields[6].offset = 24;
    fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[6].count = 1;

    radar_msg.fields = fields;

    radar_msg.point_step = sizeof(float) * fields.size();
    radar_msg.row_step = radar_msg.point_step * radar_msg.width;

    std::vector<uint8_t> data(radar_msg.row_step * radar_msg.height);
  
    size_t offset = 0;
   
    for (size_t i = 0; i < carla_radar_measurement->GetDetectionAmount(); ++i) {
      const auto &detection = carla_radar_measurement->at(i);
      float x = detection.depth * std::cos(detection.azimuth) * std::cos(-detection.altitude);
      float y = detection.depth * std::sin(-detection.azimuth) * std::cos(detection.altitude);
      float z = detection.depth * std::sin(detection.altitude);

      float range = detection.depth;
      float velocity = detection.velocity;
      float azimuth_angle = detection.azimuth;
      float elevation_angle = detection.altitude;
      //RCLCPP_INFO(this->get_logger(), "Radar vale: %f %f %f %f %f", x,y,z,range,velocity);
      memcpy(&data[offset + fields[0].offset], &x, sizeof(float));
      memcpy(&data[offset + fields[1].offset], &y, sizeof(float));
      memcpy(&data[offset + fields[2].offset], &z, sizeof(float));
      memcpy(&data[offset + fields[3].offset], &range, sizeof(float));
      memcpy(&data[offset + fields[4].offset], &velocity, sizeof(float));
      memcpy(&data[offset + fields[5].offset], &azimuth_angle, sizeof(float));
      memcpy(&data[offset + fields[6].offset], &elevation_angle, sizeof(float));
 
      offset += radar_msg.point_step;  
    }
    radar_msg.data = data;
    right_radar_publisher_->publish(radar_msg);
 
}

void FrontRadarPublisher::publishLeftRadarData(const boost::shared_ptr<csd::RadarMeasurement> &carla_radar_measurement) {
    sensor_msgs::msg::PointCloud2 radar_msg;
    radar_msg.header.stamp = this->now();
    radar_msg.header.frame_id = "laser";

    radar_msg.height = 1;
    radar_msg.width = carla_radar_measurement->GetDetectionAmount();
    radar_msg.is_dense = false;
    radar_msg.is_bigendian = false;

    std::vector<sensor_msgs::msg::PointField> fields(7);

    // Define PointFields for x, y, z, Range, Velocity, AzimuthAngle, and ElevationAngle
    fields[0].name = "x";
    fields[0].offset = 0;
    fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[0].count = 1;

    fields[1].name = "y";
    fields[1].offset = 4;
    fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[1].count = 1;

    fields[2].name = "z";
    fields[2].offset = 8;
    fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[2].count = 1;

    fields[3].name = "Range";
    fields[3].offset = 12;
    fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[3].count = 1;

    fields[4].name = "Velocity";
    fields[4].offset = 16;
    fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[4].count = 1;

    fields[5].name = "AzimuthAngle";
    fields[5].offset = 20;
    fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[5].count = 1;

    fields[6].name = "ElevationAngle";
    fields[6].offset = 24;
    fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[6].count = 1;

    radar_msg.fields = fields;

    radar_msg.point_step = sizeof(float) * fields.size();
    radar_msg.row_step = radar_msg.point_step * radar_msg.width;

    std::vector<uint8_t> data(radar_msg.row_step * radar_msg.height);
  
    size_t offset = 0;
   
    for (size_t i = 0; i < carla_radar_measurement->GetDetectionAmount(); ++i) {
      const auto &detection = carla_radar_measurement->at(i);
      float x = detection.depth * std::cos(detection.azimuth) * std::cos(-detection.altitude);
      float y = detection.depth * std::sin(-detection.azimuth) * std::cos(detection.altitude);
      float z = detection.depth * std::sin(detection.altitude);

      float range = detection.depth;
      float velocity = detection.velocity;
      float azimuth_angle = detection.azimuth;
      float elevation_angle = detection.altitude;
      //RCLCPP_INFO(this->get_logger(), "Radar vale: %f %f %f %f %f", x,y,z,range,velocity);
      memcpy(&data[offset + fields[0].offset], &x, sizeof(float));
      memcpy(&data[offset + fields[1].offset], &y, sizeof(float));
      memcpy(&data[offset + fields[2].offset], &z, sizeof(float));
      memcpy(&data[offset + fields[3].offset], &range, sizeof(float));
      memcpy(&data[offset + fields[4].offset], &velocity, sizeof(float));
      memcpy(&data[offset + fields[5].offset], &azimuth_angle, sizeof(float));
      memcpy(&data[offset + fields[6].offset], &elevation_angle, sizeof(float));
 
      offset += radar_msg.point_step;  
    }
    radar_msg.data = data;
    left_radar_publisher_->publish(radar_msg);
 
}

void FrontRadarPublisher::publishRadarData(const boost::shared_ptr<csd::RadarMeasurement> &carla_radar_measurement) {
    sensor_msgs::msg::PointCloud2 radar_msg;
    radar_msg.header.stamp = this->now();
    radar_msg.header.frame_id = "laser";

    radar_msg.height = 1;
    radar_msg.width = carla_radar_measurement->GetDetectionAmount();
    radar_msg.is_dense = false;
    radar_msg.is_bigendian = false;

    std::vector<sensor_msgs::msg::PointField> fields(7);

    // Define PointFields for x, y, z, Range, Velocity, AzimuthAngle, and ElevationAngle
    fields[0].name = "x";
    fields[0].offset = 0;
    fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[0].count = 1;

    fields[1].name = "y";
    fields[1].offset = 4;
    fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[1].count = 1;

    fields[2].name = "z";
    fields[2].offset = 8;
    fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[2].count = 1;

    fields[3].name = "Range";
    fields[3].offset = 12;
    fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[3].count = 1;

    fields[4].name = "Velocity";
    fields[4].offset = 16;
    fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[4].count = 1;

    fields[5].name = "AzimuthAngle";
    fields[5].offset = 20;
    fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[5].count = 1;

    fields[6].name = "ElevationAngle";
    fields[6].offset = 24;
    fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[6].count = 1;

    radar_msg.fields = fields;

    radar_msg.point_step = sizeof(float) * fields.size();
    radar_msg.row_step = radar_msg.point_step * radar_msg.width;

    std::vector<uint8_t> data(radar_msg.row_step * radar_msg.height);
  
    size_t offset = 0;
   
    for (size_t i = 0; i < carla_radar_measurement->GetDetectionAmount(); ++i) {
      const auto &detection = carla_radar_measurement->at(i);
      float x = detection.depth * std::cos(detection.azimuth) * std::cos(-detection.altitude);
      float y = detection.depth * std::sin(-detection.azimuth) * std::cos(detection.altitude);
      float z = detection.depth * std::sin(detection.altitude);

      float range = detection.depth;
      float velocity = detection.velocity;
      float azimuth_angle = detection.azimuth;
      float elevation_angle = detection.altitude;
      //RCLCPP_INFO(this->get_logger(), "Radar vale: %f %f %f %f %f", x,y,z,range,velocity);
      memcpy(&data[offset + fields[0].offset], &x, sizeof(float));
      memcpy(&data[offset + fields[1].offset], &y, sizeof(float));
      memcpy(&data[offset + fields[2].offset], &z, sizeof(float));
      memcpy(&data[offset + fields[3].offset], &range, sizeof(float));
      memcpy(&data[offset + fields[4].offset], &velocity, sizeof(float));
      memcpy(&data[offset + fields[5].offset], &azimuth_angle, sizeof(float));
      memcpy(&data[offset + fields[6].offset], &elevation_angle, sizeof(float));
 
      offset += radar_msg.point_step;  
    }
    radar_msg.data = data;
    publisher_->publish(radar_msg);
 
    }

