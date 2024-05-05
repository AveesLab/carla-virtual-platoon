#include "shared_carlalib.h"
#include "CarlaLocation.hpp"
#include "CarlaRGBCamera.hpp"
#include "CarlaLidar.hpp"
#include "CarlaVehicle.hpp"
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

carla::geom::Location GetTruckLocation(int num) {

    float x = IHP_loc[num][0];
    float y = IHP_loc[num][1];
    float z = IHP_loc[num][2];
    return carla::geom::Location(x,y,z);
}

carla::geom::Rotation GetTruckRotation(int num) {

    float pitch = IHP_rot[num][0];
    float yaw = IHP_rot[num][1];
    float roll = IHP_rot[num][2];
    return carla::geom::Rotation(pitch,yaw,roll);
}

void connect_to_carla() {
    //Connecting to CARLA server;
    client = new cc::Client(host, port);
    client->SetTimeout(40s);
    std::cerr << "Trucks connected to CARLA server" << '\n';
    world = new cc::World(client->GetWorld());
    blueprint_library = world->GetBlueprintLibrary();
}

void generate_truck(int TruckNum) {
    std::mt19937_64 rng((std::random_device())());  

    //Get a Trailer
    auto trailer = blueprint_library->Filter("trailer");
    auto blueprint_trailer = RandomChoice(*trailer, rng);
    // Get a Truck blueprint.
    auto truck = blueprint_library->Filter("dafxf");
    auto blueprint_truck = RandomChoice(*truck, rng);  

    carla::geom::Location TruckLocation = GetTruckLocation(TruckNum);
    carla::geom::Rotation TruckRotation = GetTruckRotation(TruckNum);
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
    auto node_camera = std::make_shared<CarlaRGBCameraPublisher>(actor_truck);
    //auto node_radar = std::make_shared<CarlaRadarPublisher>(actor_truck);
    auto node_lidar = std::make_shared<CarlaLidarPublisher>(actor_truck);
    auto node_vehicle = std::make_shared<CarlaVehicleController>(vehicle_truck);

    executor.add_node(node_camera);
    //executor.add_node(node_radar);
    executor.add_node(node_lidar);
    executor.add_node(node_vehicle);

    executor.spin(); 



    vehicle_truck->Destroy();
    vehicle_trailer->Destroy();     
}


int main(int argc, char *argv[]) {
    try {
        rclcpp::init(argc, argv);

        int NumTrucks = 1; // Default value

        std::string prefix("--truck_id=");
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg.find(prefix) == 0) {
                NumTrucks = std::atoi(arg.substr(prefix.length()).c_str());
            }
        }
        std::cout << "Truck Number : " << NumTrucks << std::endl;
        connect_to_carla();
        generate_truck(NumTrucks);

        rclcpp::shutdown();
    } 
    catch (const std::exception& e) {
        std::cerr << "Unhandled Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}