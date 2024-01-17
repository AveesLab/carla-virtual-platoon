#include "shared_carlalib.h"
#include "CarlaRGBCamera.hpp"
#include "CarlaRadar.hpp"


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

    // Get world
    auto world = client.GetWorld();    
    auto blueprint_library = world.GetBlueprintLibrary(); 

    //Get a Trailer
    auto trailer_fv1 = blueprint_library->Filter("trailer");
    auto blueprint_trailer_fv1 = RandomChoice(*trailer_fv1, rng);
    // Get a Truck blueprint.
    auto vehicles_fv1 = blueprint_library->Filter("dafxf");
    auto blueprint_fv1 = RandomChoice(*vehicles_fv1, rng);  
    auto map = world.GetMap();
    auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
    transform.location.x = 11.90f;
    transform.location.y = 13.32f;
    transform.location.z = 1.0f;
    transform.rotation.roll = 0.0f;
    transform.rotation.pitch = 0.0f;
    transform.rotation.yaw= -90.22f;

    // Spawn the trailer
    auto actor_trailer_fv1 = world.SpawnActor(blueprint_trailer_fv1, transform);
    std::cout << "Spawned " << actor_trailer_fv1->GetDisplayId() << '\n';
    auto trailer_fv1_ = boost::static_pointer_cast<cc::Vehicle>(actor_trailer_fv1);    

    // Find a valid spawn point for truck
    transform.location += 5.2f * transform.GetForwardVector();    

    // Spawn the truck
    auto actor_fv1 = world.SpawnActor(blueprint_fv1, transform);
    std::cout << "Spawned " << actor_fv1->GetDisplayId() << '\n';
    auto vehicle_fv1 = boost::static_pointer_cast<cc::Vehicle>(actor_fv1);    

    // Set autopilot
   // vehicle_fv1->SetAutopilot(true);  
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node_fv1 = std::make_shared<CarlaRGBCameraPublisher>(blueprint_library,actor_fv1,world);
    auto node_radar_fv1 = std::make_shared<CarlaRadarPublisher>(blueprint_library,actor_fv1,world);
    executor.add_node(node_fv1);
    executor.add_node(node_radar_fv1);

    executor.spin();
    rclcpp::shutdown(); 
    // Remove actors from the simulation.
      
    vehicle_fv1->Destroy();
    trailer_fv1_->Destroy();
  


    std::cout << "Actors destroyed." << std::endl;  

    return 0;
}