#include "shared_carlalib.h"
#include "CarlaLocation.hpp"
#include "FrontCamera.hpp"
#include "FrontLidar.hpp"
#include "TruckControl.hpp"
#include "TruckStatus.hpp"
#include <thread>
#include <unistd.h>
#include <sys/wait.h>

std::string host = "localhost";
uint16_t port = 2000u;
cc::Client *client;
cc::World *world;
carla::SharedPtr<carla::client::BlueprintLibrary> blueprint_library;


/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
}

carla::geom::Location GetTruckLocation(int truck_num, std::string map_name) {
    float x = 0.0f , y = 0.0f ,z =0.0f;
    if (truckLocations.find(map_name) != truckLocations.end() && truckLocations[map_name].find(truck_num) != truckLocations[map_name].end()) {
        x = truckLocations[map_name][truck_num][0];
        y = truckLocations[map_name][truck_num][1];
        z = truckLocations[map_name][truck_num][2];
    } else {
        throw std::runtime_error("Invalid map name or truck index for location.");
    }

    return carla::geom::Location(x,y,z);
}

carla::geom::Rotation GetTruckRotation(int truck_num, std::string map_name) {
    float pitch = 0.0f , yaw = 0.0f ,roll =0.0f;
    if (truckRotations.find(map_name) != truckRotations.end() && truckRotations[map_name].find(truck_num) != truckRotations[map_name].end()) {
        pitch = truckRotations[map_name][truck_num][0];
        yaw = truckRotations[map_name][truck_num][1];
        roll = truckRotations[map_name][truck_num][2];
    } else {
        throw std::runtime_error("Invalid map name or truck index for rotation.");
    }
    return carla::geom::Rotation(pitch,yaw,roll);
}

void connect_to_carla(int truck_num) {
    //Connecting to CARLA server;
    client = new cc::Client(host, port);
    client->SetTimeout(40s);
    std::cerr << truck_num << " Truck connected to CARLA server" << '\n';
    world = new cc::World(client->GetWorld());
    blueprint_library = world->GetBlueprintLibrary();
}

void generate_truck(int truck_num, std::string map_name) {
    std::mt19937_64 rng((std::random_device())());  

    //Get a Trailer
    auto trailer = blueprint_library->Filter("trailer");
    auto blueprint_trailer = RandomChoice(*trailer, rng);
    blueprint_trailer.SetAttribute("role_name", "trailer" + std::to_string(truck_num));

    // Get a Truck blueprint.
    auto truck = blueprint_library->Filter("dafxf");
    auto blueprint_truck = RandomChoice(*truck, rng);  
    blueprint_truck.SetAttribute("role_name", "truck" + std::to_string(truck_num));

    carla::geom::Location TruckLocation = GetTruckLocation(truck_num,map_name);
    carla::geom::Rotation TruckRotation = GetTruckRotation(truck_num,map_name);
    carla::geom::Transform transform(TruckLocation,TruckRotation);
    
    // Spawn the trailer
    auto actor_trailer = world->SpawnActor(blueprint_trailer, transform);
    std::cout << "Spawned " << actor_trailer->GetDisplayId() << '\n';
    auto vehicle_trailer = boost::static_pointer_cast<cc::Vehicle>(actor_trailer);   

    // Spawn the truck
    transform.location += 5.2f * transform.GetForwardVector(); 
    auto actor_truck = world->SpawnActor(blueprint_truck, transform);
    std::cout << "Spawned " << actor_truck->GetDisplayId() << '\n';
    auto vehicle_truck = boost::static_pointer_cast<cc::Vehicle>(actor_truck);


    rclcpp::executors::MultiThreadedExecutor executor; 
    auto node_camera = std::make_shared<FrontCameraPublisher>(actor_truck);
    //auto node_radar = std::make_shared<FrontRadarPublisher>(actor_truck);
    auto node_lidar = std::make_shared<FrontLidarPublisher>(actor_truck);
    auto node_control = std::make_shared<TruckControl>(vehicle_truck);
    auto node_status = std::make_shared<TruckStatusPublisher>(vehicle_truck);

    executor.add_node(node_camera);
    //executor.add_node(node_radar);
    executor.add_node(node_lidar);
    executor.add_node(node_control);
    executor.add_node(node_status);

    executor.spin(); 



    vehicle_truck->Destroy();
    vehicle_trailer->Destroy();     
}


int main(int argc, char *argv[]) {
    try {
        rclcpp::init(argc, argv);

        int truck_num = 1; // Default value
        std::string map_name = "IHP"; //Default map

        std::string prefix_truck_id("--truck_id=");
        std::string prefix_map_name("--map=");
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg.find(prefix_truck_id) == 0) {
                truck_num = std::atoi(arg.substr(prefix_truck_id.length()).c_str());
            }
            else if (arg.find(prefix_map_name) == 0) {
                map_name = arg.substr(prefix_map_name.length());
            }
        }

        if(truck_num == 0 ) {
            std::cout << "Truck Number : " << truck_num << std::endl;
            std::cout << "Map Name : " << map_name << std::endl;            
        }

        connect_to_carla(truck_num);
        generate_truck(truck_num,map_name);

        rclcpp::shutdown();
    } 
    catch (const std::exception& e) {
        std::cerr << "Unhandled Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
