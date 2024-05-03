#include "CarlaRGBCamera.hpp"
#include <rclcpp/qos.hpp>
#include <boost/make_shared.hpp>


#include <opencv2/opencv.hpp> // OpenCV 헤더 파일 추가
#include <sensor_msgs/msg/image.hpp> // ROS2 이미지 메시지 헤더


static void SaveSemSegImageToDisk(const csd::Image &image) {
  using namespace carla::image;  
  char buffer[9u];
  std::snprintf(buffer, sizeof(buffer), "%08zu", image.GetFrame());
  auto filename = "_images/"s + buffer + ".png";  
  auto view = ImageView::MakeView(image);
  ImageIO::WriteView(filename, view);
}
CarlaRGBCameraPublisher::CarlaRGBCameraPublisher(boost::shared_ptr<carla::client::Actor> actor,int num)
    : Node("carla_camera_publisher", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)) {

    rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    custom_qos.best_effort();
    
    this->get_parameter_or("rgbcam/x",rgbcam_x,2.0f);
    this->get_parameter_or("rgbcam/y",rgbcam_y,0.0f);
    this->get_parameter_or("rgbcam/z",rgbcam_z,3.5f);
    this->get_parameter_or("rgbcam/pitch",rgbcam_pitch, -15.0f);
    this->get_parameter_or("rgbcam/yaw",rgbcam_yaw,0.0f);
    this->get_parameter_or("rgbcam/roll",rgbcam_roll,0.0f);
    this->get_parameter_or("rgbcam/sensor_tick",rgbcam_sensor_tick,std::string("0.033333f"));
    this->get_parameter_or("rgbcam_topic_name",rgbcam_topic_name,std::string("/carla/image_raw"));
    

    rgbcam_topic_name = "/truck" + std::to_string(num) + rgbcam_topic_name;
    this->actor = actor;
  
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(rgbcam_topic_name, custom_qos);

    camera_bp = boost::shared_ptr<carla::client::ActorBlueprint>(const_cast<carla::client::ActorBlueprint*>(blueprint_library->Find("sensor.camera.rgb")));
    
    camera_bp->SetAttribute("sensor_tick", rgbcam_sensor_tick);
    camera_bp->SetAttribute("image_size_x","640");
    camera_bp->SetAttribute("image_size_y","480");
    camera_bp->SetAttribute("fov", "90.0");
    assert(camera_bp != nullptr);

    camera_transform = cg::Transform{ cg::Location{rgbcam_x, rgbcam_y, rgbcam_z}, cg::Rotation{rgbcam_pitch, rgbcam_yaw, rgbcam_roll}}; // pitch, yaw, roll.
    cam_actor = world->SpawnActor(*camera_bp, camera_transform, actor.get());
    camera = boost::static_pointer_cast<cc::Sensor>(cam_actor);

    camera->Listen([this](auto data) {
        auto image = boost::static_pointer_cast<csd::Image>(data);
        assert(image != nullptr);
        publishImage(*image);
    });
}




void CarlaRGBCameraPublisher::publishImage(const csd::Image &carla_image) {
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

    // Publish the message
    publisher_->publish(*msg);
}

