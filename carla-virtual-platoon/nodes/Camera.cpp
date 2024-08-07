#include "Camera.hpp"

CameraPublisher::CameraPublisher(boost::shared_ptr<carla::client::Actor> actor)
    : Node("camera_node"+ std::to_string(actor->GetId()), rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)) {

    rclcpp::QoS custom_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    actor_ = actor;
    this->get_parameter_or("add_sensor/camera_number", num_cameras_, 2);
    this->get_parameter_or("carla/sync", sync_ , false);
    this->get_parameter_or("carla/sync_with_delay", sync_with_delay, false);
    
    if(sync_with_delay) {
        GetDelayParameter();
        LaneImagePublisher = this->create_publisher<sensor_msgs::msg::Image>("lane_camera", custom_qos);
        WaitLanePublisher = this->create_publisher<std_msgs::msg::Bool>("wait_lane", custom_qos);
        WaitVelPublisher = this->create_publisher<std_msgs::msg::Bool>("wait_vel", custom_qos);
    }

    velocity_image_queue.resize(num_cameras_);
    publishers_.resize(num_cameras_);

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
        if(sync_ || sync_with_delay) rgbcam_sensor_tick = "0.0f";
    
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

            static int prev_tick_cnt = 0;

            if(sync_with_delay) {
               
                if (!velocity_image_queue[i].empty() && (tick_cnt == 0 ? prev_tick_cnt - velocity_image_queue[i].front().timestamp >= velocity_planner_delay : tick_cnt - velocity_image_queue[i].front().timestamp >= velocity_planner_delay)) {
                    auto image_ = velocity_image_queue[i].front().image;
                    velocity_image_queue[i].pop();
                    publishImage(*image_, publishers_[i]);
                    wait_for_velocity(true);
                }
                else {
                    wait_for_velocity(false);
                }
                
                

               
                if (i == 0 && !path_image_queue.empty() && (tick_cnt == 0 ? prev_tick_cnt - path_image_queue.front().timestamp >= path_planner_delay : tick_cnt - path_image_queue.front().timestamp >= path_planner_delay)) {
                    auto image_ = path_image_queue.front().image;
                    path_image_queue.pop();
                    publishImage(*image_, LaneImagePublisher);
                    wait_for_velocity(true);
                }
                else {
                    wait_for_lane(false);
                }
                

                if(tick_cnt % velocity_planner_period == 0) {
                    velocity_image_queue[i].push(TimedImage{image, tick_cnt});
                }

                if(i == 0 && tick_cnt % path_planner_period == 0) {
                    path_image_queue.push(TimedImage{image, tick_cnt});
                }

                tick_cnt += 10;

                if(tick_cnt >= lcm_period) {
                    prev_tick_cnt = tick_cnt; // Save the current tick count before resetting
                    tick_cnt = 0;
                }

            } else {
                publishImage(*image, publishers_[i]);
            }
        });


    }
}

void CameraPublisher::wait_for_lane(bool ch_) {
    std_msgs::msg::Bool msg;
    msg.data = ch_;
    WaitLanePublisher->publish(msg);
}

void CameraPublisher::wait_for_velocity(bool ch_) {
    std_msgs::msg::Bool msg;
    msg.data = ch_;
    WaitVelPublisher->publish(msg);
}

void CameraPublisher::GetDelayParameter() {
    this->get_parameter_or("period/velocity_planner", velocity_planner_period, 30);
    this->get_parameter_or("delay/velocity_planner", velocity_planner_delay , 100);
    this->get_parameter_or("period/path_planner", path_planner_period, 30);
    this->get_parameter_or("delay/path_planner", path_planner_delay , 100);

    int lcm_velocity = lcm(velocity_planner_period, velocity_planner_delay);
    int lcm_path = lcm(path_planner_period, path_planner_delay);
    int lcm_all = lcm(lcm_velocity, lcm_path);

    lcm_period = lcm_all; 
}

int CameraPublisher::gcd(int a, int b) {
    while (b != 0) {
        int temp = b;
        b = a % b;
        a = temp;
    }
    return a;
}

int CameraPublisher::lcm(int a, int b) {
    return (a * b) / gcd(a, b);
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

