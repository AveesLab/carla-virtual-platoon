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
CarlaRGBCameraPublisher::CarlaRGBCameraPublisher(boost::shared_ptr<carla::client::BlueprintLibrary> blueprint_library, boost::shared_ptr<carla::client::Actor> actor,carla::client::World& world_, std::string name_)
    : Node("carla_camera_publisher", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)),world_(world_) {

    rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    custom_qos.best_effort();
    if(name_.compare("/FV1/") == 0 || name_.compare("/FV2/") == 0  || name_.compare("/LV/") == 0) {
    this->role_name_ = name_;
    this->get_parameter_or("rgbcam/x",rgbcam_x,2.0f);
    this->get_parameter_or("rgbcam/y",rgbcam_y,0.0f);
    this->get_parameter_or("rgbcam/z",rgbcam_z,3.5f);
    this->get_parameter_or("rgbcam/pitch",rgbcam_pitch, -15.0f);
    this->get_parameter_or("rgbcam/yaw",rgbcam_yaw,0.0f);
    this->get_parameter_or("rgbcam/roll",rgbcam_roll,0.0f);
    this->get_parameter_or("rgbcam/sensor_tick",rgbcam_sensor_tick,std::string("0.033333f"));
    this->get_parameter_or("rgbcam_topic_name",rgbcam_topic_name,std::string("LV/carla/camasdasdera"));
    

    rgbcam_topic_name = role_name_ + rgbcam_topic_name;
    this->blueprint_library = blueprint_library;
    this->actor = actor;
  
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(rgbcam_topic_name, custom_qos);

    camera_bp = boost::shared_ptr<carla::client::ActorBlueprint>(
    const_cast<carla::client::ActorBlueprint*>(blueprint_library->Find("sensor.camera.rgb"))
);
    
    camera_bp->SetAttribute("sensor_tick", rgbcam_sensor_tick);
    camera_bp->SetAttribute("image_size_x","640");
    camera_bp->SetAttribute("image_size_y","480");
    camera_bp->SetAttribute("fov", "90.0");
    //camera_bp->SetAttribute("enable_postprocess_effects","false");
    assert(camera_bp != nullptr);

    camera_transform = cg::Transform{
    cg::Location{rgbcam_x, rgbcam_y, rgbcam_z},   // x, y, z.
    cg::Rotation{rgbcam_pitch, rgbcam_yaw, rgbcam_roll}}; // pitch, yaw, roll.
    cam_actor = world_.SpawnActor(*camera_bp, camera_transform, actor.get());
    camera = boost::static_pointer_cast<cc::Sensor>(cam_actor);

    camera->Listen([this](auto data) {
    auto image = boost::static_pointer_cast<csd::Image>(data);
    assert(image != nullptr);
    publishImage(*image);
  });

    }
    else {
    this->role_name_  = name_;
    this->get_parameter_or("rgbcam/x",rgbcam_x,-5.0f);
    this->get_parameter_or("rgbcam/y",rgbcam_y,-30.0f);
    this->get_parameter_or("rgbcam/z",rgbcam_z,30.0f);
    this->get_parameter_or("rgbcam/pitch",rgbcam_pitch, -35.0f);
    this->get_parameter_or("rgbcam/yaw",rgbcam_yaw,90.0f);
    this->get_parameter_or("rgbcam/roll",rgbcam_roll,0.0f);
    this->get_parameter_or("rgbcam/sensor_tick",rgbcam_sensor_tick,std::string("0.0f"));
    this->get_parameter_or("rgbcam_topic_name",rgbcam_topic_name,std::string("LV/carla/camasdasdera"));
    

    rgbcam_topic_name = role_name_ + rgbcam_topic_name;
    this->blueprint_library = blueprint_library;
    this->actor = actor;
  
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(rgbcam_topic_name, custom_qos);

    camera_bp = boost::shared_ptr<carla::client::ActorBlueprint>(
    const_cast<carla::client::ActorBlueprint*>(blueprint_library->Find("sensor.camera.rgb"))
);
    
    camera_bp->SetAttribute("sensor_tick", "0.0f");
    camera_bp->SetAttribute("image_size_x","1280");
    camera_bp->SetAttribute("image_size_y","600");
    camera_bp->SetAttribute("fov", "90.0");
    assert(camera_bp != nullptr);

    camera_transform = cg::Transform{
    cg::Location{rgbcam_x, rgbcam_y, rgbcam_z},   // x, y, z.
    cg::Rotation{rgbcam_pitch, rgbcam_yaw, rgbcam_roll}}; // pitch, yaw, roll.
    cam_actor = world_.SpawnActor(*camera_bp, camera_transform, actor.get());
    camera = boost::static_pointer_cast<cc::Sensor>(cam_actor);

    camera->Listen([this](auto data) {
    auto image = boost::static_pointer_cast<csd::Image>(data);
    assert(image != nullptr);
    publishImage1(*image);
  });

    }













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



void CarlaRGBCameraPublisher::publishImage1(const csd::Image &carla_image) {
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
    
    // 여기서부터 이미지 표시를 위한 코드
    // 이미지 데이터를 cv::Mat 객체로 변환
    cv::Mat cv_image(msg->height, msg->width, CV_8UC3, msg->data.data());
  cv::Mat cv_image_bgr;
    cv::cvtColor(cv_image, cv_image_bgr, cv::COLOR_RGB2BGR);
    // 이미지 표시
    cv::imshow("CARLA RGB Image", cv_image_bgr);
    cv::waitKey(1); // OpenCV 이벤트 처리 대기, 1ms 동안 대기

    // Publish the message
    //publisher_->publish(*msg);
}