#pragma once

#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <carla/geom/Vector3D.h>
#include <std_msgs/msg/float32.hpp>
#include <carla/client/Vehicle.h>
#include <carla/rpc/VehicleControl.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/RadarMeasurement.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
using namespace std::chrono_literals;
using namespace std::string_literals;

extern cc::Client *client;
extern cc::World *world;
extern carla::SharedPtr<carla::client::BlueprintLibrary> blueprint_library;

#define EXPECT_TRUE(pred) if (!(pred)) { throw std::runtime_error(#pred); }