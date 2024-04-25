#include "shared_carlalib.h"
#include "CarlaRGBCamera.hpp"
#include "CarlaRadar.hpp"
#include "CarlaLidar.hpp"
#include "CarlaVehicle.hpp"
#include <carla/rpc/WeatherParameters.h>

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
    carla::rpc::WeatherParameters weather;
    weather.cloudiness = 0.0f;
    weather.dust_storm = 0.0f;
    weather.fog_density = 0.0f;
    weather.fog_falloff = 0.0f;
    weather.mie_scattering_scale = 0.0f;
    weather.precipitation =0.0f;
    weather.precipitation_deposits =0.0f;
    weather.rayleigh_scattering_scale = 0.0f;
    weather.scattering_intensity =0.0f;
    weather.sun_altitude_angle =0.0f;
    weather.sun_azimuth_angle =0.0f;
    weather.wetness=0.0f;
    weather.wind_intensity=0.0f;
    world.SetWeather(weather.Default);






    auto blueprint_library = world.GetBlueprintLibrary(); 

    //Get a Trailer
    auto trailer_fv1 = blueprint_library->Filter("trailer");
    auto blueprint_trailer_fv1 = RandomChoice(*trailer_fv1, rng);
    // Get a Truck blueprint.
    auto vehicles_fv1 = blueprint_library->Filter("dafxf");
    auto blueprint_fv1 = RandomChoice(*vehicles_fv1, rng);  
    blueprint_fv1.SetAttribute("role_name", "FV1");
    auto map = world.GetMap();
    auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
    
    
    
    transform.location.x = -300.990f; // 300 , 303(11m)
    transform.location.y = 30.0f;
   transform.location.z = 2.0f;
    transform.rotation.roll = 0.0f;
    transform.rotation.pitch = 0.0f;
    transform.rotation.yaw= 0.22f;

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
    
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node_fv1 = std::make_shared<CarlaRGBCameraPublisher>(blueprint_library,actor_fv1,world,"/FV1/");
    auto node_radar_fv1 = std::make_shared<CarlaRadarPublisher>(blueprint_library,actor_fv1,world,"/FV1/");
    auto node_fv1_spec = std::make_shared<CarlaRGBCameraPublisher>(blueprint_library,actor_fv1,world,"/tmptmptmp/");
  //  auto node_lidar_fv1 = std::make_shared<CarlaLidarPublisher>(blueprint_library,actor_fv1,world);
    auto node_vehicle = std::make_shared<CarlaVehicleController>(vehicle_fv1);

    executor.add_node(node_fv1_spec);
    executor.add_node(node_fv1);
    executor.add_node(node_radar_fv1);
   // executor.add_node(node_lidar_fv1);
    executor.add_node(node_vehicle);

    /*
    std::this_thread::sleep_for(20s);
    vehicle_fv1->SetAutopilot(true);   
    std::this_thread::sleep_for(5s);
    vehicle_fv1->SetAutopilot(false);
    carla::geom::Vector3D target_velocity(11.11,0 , 0); // X 축을 따라 20 m/s 속도
    vehicle_fv1->EnableConstantVelocity(target_velocity);
    */
    executor.spin();
    rclcpp::shutdown(); 
    // Remove actors from the simulation.
      
    vehicle_fv1->Destroy();
    trailer_fv1_->Destroy();
  


    std::cout << "Actors destroyed." << std::endl;  

    return 0;
}
