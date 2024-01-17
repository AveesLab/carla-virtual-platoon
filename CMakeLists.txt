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

set(ament_dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  Boost
)


include(dependency.cmake)

add_executable(LV_bridge
    src/LV.cpp
    nodes/CarlaRGBCamera.cpp
    nodes/CarlaRadar.cpp
)
add_executable(FV1_bridge
    src/FV1.cpp
    nodes/CarlaRGBCamera.cpp
    nodes/CarlaRadar.cpp
)


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

install(TARGETS
LV_bridge 
FV1_bridge
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