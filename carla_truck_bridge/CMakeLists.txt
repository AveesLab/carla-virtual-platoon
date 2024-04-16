cmake_minimum_required(VERSION 3.8)
project(carla_truck_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wl,-Bstatic -lrpc -lboost_filesystem -Wl,-Bdynamic -lpng -ltiff -ljpeg -lRecast -lDetour -lDetourCrowd)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED)
find_package(TIFF REQUIRED)
find_package(ros2_msg REQUIRED)

find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(QT_PROJECT_SOURCES
  # src	
  src/controller/main.cpp
  src/controller/controller.cpp
  src/controller/ros2node.cpp
  src/controller/controller.ui
  src/controller/lvthread.cpp
  src/controller/fv1thread.cpp
  src/controller/fv2thread.cpp
 
  # include
  src/controller/controller.h
  src/controller/ros2node.hpp
  src/controller/lvthread.h
  src/controller/fv1thread.h
  src/controller/fv2thread.h
)

set(QT_PROJECT_SOURCES_DEPENDENCIES
  rclcpp 
  std_msgs 
  Qt5Core 
  Qt5Widgets 
  ament_index_cpp 
  ros2_msg
  cv_bridge
)

set(ament_dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  Boost
)


##############################################################################################
include(dependency.cmake)

add_executable(LV_bridge
    src/LV.cpp
    nodes/CarlaRGBCamera.cpp
    nodes/CarlaRadar.cpp
    nodes/CarlaLidar.cpp
    nodes/CarlaVehicle.cpp
    nodes/CarlaIMU.cpp
)
add_executable(FV1_bridge
    src/FV1.cpp
    nodes/CarlaRGBCamera.cpp
    nodes/CarlaRadar.cpp
    nodes/CarlaLidar.cpp
    nodes/CarlaVehicle.cpp
    nodes/CarlaIMU.cpp
)

add_executable(FV2_bridge
    src/FV2.cpp
    nodes/CarlaRGBCamera.cpp
    nodes/CarlaRadar.cpp
    nodes/CarlaLidar.cpp
    nodes/CarlaVehicle.cpp
    nodes/CarlaIMU.cpp
)
##############################################################################################
add_executable(controller ${QT_PROJECT_SOURCES})
ament_target_dependencies(controller ${QT_PROJECT_SOURCES_DEPENDENCIES})

target_link_libraries(controller
  Qt5::Core
  Qt5::Quick
  Qt5::Widgets
)
##############################################################################################
ament_target_dependencies(LV_bridge ${ament_dependencies} )

target_include_directories(LV_bridge PUBLIC
    ${libCarla_INCLUDE_DIR}
    ${DEP_INCLUDE_DIR}
)
target_include_directories(LV_bridge PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_directories(LV_bridge PUBLIC
    ${libCarla_LIB_DIR}
)

target_link_libraries(LV_bridge ${libCarla_LIBS} ${Boost_LIBRARIES} ${TIFF_LIBRARIES} jpeg png
)
##############################################################################################
ament_target_dependencies(FV1_bridge ${ament_dependencies} )

target_include_directories(FV1_bridge PUBLIC
    ${libCarla_INCLUDE_DIR}
    ${DEP_INCLUDE_DIR}
)
target_include_directories(FV1_bridge PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_directories(FV1_bridge PUBLIC
    ${libCarla_LIB_DIR}
)

target_link_libraries(FV1_bridge ${libCarla_LIBS} ${Boost_LIBRARIES} ${TIFF_LIBRARIES} jpeg png
)
##############################################################################################
ament_target_dependencies(FV2_bridge ${ament_dependencies} )

target_include_directories(FV2_bridge PUBLIC
    ${libCarla_INCLUDE_DIR}
    ${DEP_INCLUDE_DIR}
)
target_include_directories(FV2_bridge PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_directories(FV2_bridge PUBLIC
    ${libCarla_LIB_DIR}
)

target_link_libraries(FV2_bridge ${libCarla_LIBS} ${Boost_LIBRARIES} ${TIFF_LIBRARIES} jpeg png
)
##############################################################################################
install(TARGETS
LV_bridge 
FV1_bridge
FV2_bridge
controller
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY libcarla-install/lib/
  DESTINATION lib
)

ament_package()
