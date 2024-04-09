cmake_minimum_required(VERSION 3.8)
project(ros2_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Ocr2lrc.msg"
  "msg/Lrc2ocr.msg"
  "msg/Lrc2xav.msg"
  "msg/Xav2lrc.msg"
  "msg/Lane.msg"
  "msg/Obj2xav.msg"
  "msg/Lane2xav.msg"
  "msg/Xav2lane.msg"
  "msg/Cmd2xav.msg"
  "msg/Xav2cmd.msg"
  "msg/Boundingbox.msg"
  "msg/Yoloflag.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
 ${msg_files}
 DEPENDENCIES geometry_msgs std_msgs 
)

install(TARGETS
        DESTINATION lib/${PROJECT_NAME})



ament_package()
