#include "shared_carlalib.h"
#include "CarlaRGBCamera.hpp"
#include "CarlaRadar.hpp"
#include "CarlaLidar.hpp"
#include "CarlaVehicle.hpp"

/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
}/// Save a semantic segmentation image to disk converting to CityScapes palette.

  
  static auto ParseArguments() {
 
  using ResultType = std::tuple<std::string, uint16_t>;
  return ResultType{"localhost", 2000u};
}



int main(int argc, const char *argv[]) {
    std::string host;
    uint16_t port;
    std::tie(host, port) = ParseArguments();    
    std::mt19937_64 rng((std::random_device())());    
    auto client = cc::Client(host, port);
    client.SetTimeout(40s);  
    
    std::cout << "Client API version : " << client.GetClientVersion() << '\n';
    std::cout << "Server API version : " << client.GetServerVersion() << '\n';  

    // Geta Town04.
    auto world = client.GetWorld();    
    auto blueprint_library = world.GetBlueprintLibrary(); 
    //Get a Trailer
    auto trailer = blueprint_library->Filter("trailer");
    auto blueprint_trailer = RandomChoice(*trailer, rng);
    // Get a Truck blueprint.
    auto vehicles = blueprint_library->Filter("dafxf");
    auto blueprint = RandomChoice(*vehicles, rng);  
    blueprint.SetAttribute("role_name", "LV");
    // Find a valid spawn point for trailer.
    auto map = world.GetMap();
    auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);

    transform.location.x = -270.990f; // 300 , 303(11m)
    transform.location.y = 30.0f;
    transform.location.z = 2.0f;
    transform.rotation.roll = 0.0f;
    transform.rotation.pitch = 0.0f;
    transform.rotation.yaw= 0.22f;
    
    // Spawn the trailer
    auto actor_trailer = world.SpawnActor(blueprint_trailer, transform);
    std::cout << "Spawned " << actor_trailer->GetDisplayId() << '\n';
    auto trailer_ = boost::static_pointer_cast<cc::Vehicle>(actor_trailer);    

    // Find a valid spawn point for truck
    transform.location += 5.2f * transform.GetForwardVector();    

    // Spawn the truck
    auto actor = world.SpawnActor(blueprint, transform);
    std::cout << "Spawned " << actor->GetDisplayId() << '\n';
    auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);    
    
    // Move spectator so we can see the vehicle from the simulator window.
    auto spectator = world.GetSpectator();
    transform.location += 32.0f * transform.GetForwardVector();
    transform.location.z += 2.0f;
    transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -15.0f;
    spectator->SetTransform(transform);  


    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<CarlaRGBCameraPublisher>(blueprint_library,actor,world);
    //auto node_radar = std::make_shared<CarlaRadarPublisher>(blueprint_library,actor,world);
    auto node_lidar = std::make_shared<CarlaLidarPublisher>(blueprint_library,actor,world);
    auto node_vehicle = std::make_shared<CarlaVehicleController>(vehicle);
    //executor.add_node(node);
    executor.add_node(node_lidar);
    executor.add_node(node_vehicle);
     // Set autopilot
//    std::this_thread::sleep_for(10s);
//    vehicle->SetAutopilot(true);   
//    carla::geom::Vector3D target_velocity(10.4444,0 , 0); 
//    vehicle->EnableConstantVelocity(target_velocity);
    executor.spin();

    rclcpp::shutdown(); 

    // Remove actors from the simulation.
    vehicle->Destroy();
    trailer_->Destroy();

    std::cout << "Actors destroyed." << std::endl;  

    return 0;
}
