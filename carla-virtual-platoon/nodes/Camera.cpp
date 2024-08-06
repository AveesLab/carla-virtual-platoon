#include "Camera.hpp"

CameraPublisher::CameraPublisher(boost::shared_ptr<carla::client::Actor> actor)
    : Node("camera_node"+ std::to_string(actor->GetId()), rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)) {

    rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    actor_ = actor;
    this->get_parameter_or("add_sensor/camera_number", num_cameras_, 2);
    this->get_parameter_or("carla/sync", sync_ , true);
    //this->getparameter_or("carla/sync_with_delay", sync_with_delay, true);

    for(int i=0; i<num_cameras_; ++i){
      std::string index = std::to_string(i);

      this->get_parameter_or("camera" + index + "/x", rgbcam_x,2.0f);
      this->get_parameter_or("camera" + index + "/y", rgbcam_y,0.0f);
      this->get_parameter_or("camera" + index + "/z", rgbcam_z,3.5f);
      this->get_parameter_or("camera" + index + "/pitch", rgbcam_pitch, -15.0f);
      this->get_parameter_or("camera" + index + "/yaw", rgbcam_yaw,0.0f);
      this->get_parameter_or("camera" + index + "/roll", rgbcam_roll,0.0f);
      this->get_parameter_or("camera" + index + "/sensor_tick", rgbcam_sensor_tick,std::string("0.033333f"));
      this->get_parameter_or("camera" + index + "/image_size_x", rgbcam_image_size_x,std::string("640"));
      this->get_parameter_or("camera" + index + "/image_size_y", rgbcam_image_size_y,std::string("480"));
      this->get_parameter_or("camera" + index + "/fov", rgbcam_fov,std::string("90.0f"));
      this->get_parameter_or("camera" + index + "/topic_name", rgbcam_topic_name ,std::string("carla/image_raw" + index));
      if(sync_) rgbcam_sensor_tick = "0.0f";
    
      auto publisher = this->create_publisher<sensor_msgs::msg::Image>(rgbcam_topic_name, custom_qos);
      publishers_.push_back(publisher);
      
      auto camera_bp = blueprint_library->Find("sensor.camera.rgb");
      assert(camera_bp != nullptr);

      // Create a modifiable copy of the camera blueprint
      auto camera_bp_modifiable = *camera_bp;

      camera_bp_modifiable.SetAttribute("sensor_tick", rgbcam_sensor_tick);
      camera_bp_modifiable.SetAttribute("image_size_x", "640");
      camera_bp_modifiable.SetAttribute("image_size_y", "480");
      camera_bp_modifiable.SetAttribute("fov", "90.0f");
      camera_bp_modifiable.SetAttribute("lens_flare_intensity", "0.0f");

      auto camera_transform = cg::Transform{cg::Location{rgbcam_x, rgbcam_y, rgbcam_z}, cg::Rotation{rgbcam_pitch, rgbcam_yaw, rgbcam_roll}};
      auto camera_actor = world->SpawnActor(camera_bp_modifiable, camera_transform, actor.get());
      auto camera = boost::static_pointer_cast<cc::Sensor>(camera_actor);

      camera_sensors.push_back(camera);
  
      camera->Listen([this, i](auto data) {
          auto image = boost::static_pointer_cast<csd::Image>(data);
          assert(image != nullptr);
          publishImage(*image, publishers_[i]);
      });
    }
}

void CameraPublisher::publishImage(const csd::Image &carla_image, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher) {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
  
    // Set the header
     msg->header.stamp = this->now();
   
    msg->header.frame_id = "1"; // Set appropriate frame ID

    // Set image properties
    msg->height = carla_image.GetHeight();
    msg->width = carla_image.GetWidth();
    msg->encoding = "rgb8"; // Assuming the image is in RGB8 format
    msg->is_bigendian = false;
    msg->step = carla_image.GetWidth() * 3; // 3 bytes per pixel for RGB8 encoding

    // Allocate memory for ROS message data
    msg->data.resize(msg->step * msg->height);

    // Copy image data
    const auto* raw_data = reinterpret_cast<const uint8_t*>(carla_image.data());
    for (int i = 0; i < msg->height * msg->width; ++i) {
        // Skip alpha channel by offsetting the raw data index
        int raw_index = i * 4; // 4 bytes per pixel (BGRA)
        int msg_index = i * 3; // 3 bytes per pixel (RGB)
        msg->data[msg_index] = raw_data[raw_index + 2];     // Red (BGR -> RGB)
        msg->data[msg_index + 1] = raw_data[raw_index + 1]; // Green
        msg->data[msg_index + 2] = raw_data[raw_index];     // Blue
    }
    std::cerr << "pub image" << std::endl;
    // Publish the message
    publisher->publish(*msg);
}

