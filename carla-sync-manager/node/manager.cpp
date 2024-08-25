#include "manager.hpp"
const float epsilon = 0.0001f; 
/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
}


SyncManager::SyncManager()
    : Node("sync_manager_node"), registration_(10,false), sync_throttle(10,false), sync_steer(10,false) {

            client = new cc::Client(host, port);
            world = new cc::World(client->GetWorld());

            TruckSizeSubscriber_ = this->create_subscription<std_msgs::msg::Int32>("/numtruckss", 10, std::bind(&SyncManager::TruckSizeSubCallback, this, std::placeholders::_1));
            RegistrationSubscriber_ = this->create_subscription<std_msgs::msg::Int32>("/registration", 10, std::bind(&SyncManager::RegistrationSubCallback, this, std::placeholders::_1));
            SyncThrottleSubscriber_ = this->create_subscription<std_msgs::msg::Int32>("/sync_throttle", 10, std::bind(&SyncManager::SyncThrottleSubCallback, this, std::placeholders::_1));
            SyncSteerSubscriber_ = this->create_subscription<std_msgs::msg::Int32>("/sync_steer", 10, std::bind(&SyncManager::SyncSteerSubCallback, this, std::placeholders::_1));
            ShutdownPublisher_ = this->create_publisher<std_msgs::msg::String>("/shutdown_topic",10);
            isNodeRunning_ = true;

            settings = world->GetSettings();   
            settings.synchronous_mode = true; // sync_mode 
            settings.fixed_delta_seconds = 0.01f; // FPS
            world->ApplySettings(settings,time_);


            manager_Thread = std::thread(&SyncManager::managerInThread, this);
                std::cerr << "init finish" << std::endl;
           }


SyncManager::~SyncManager(void) 
{
    isNodeRunning_ = false;
    manager_Thread.join();
    settings.synchronous_mode = false;
    world->ApplySettings(settings,time_);
    sleep(5);
    auto actor_list = world->GetActors();
    for (size_t i = 0; i < truck_ids.size(); ++i) {
        unsigned int actor_id = truck_ids[i];
        unsigned int trail_id = trailer_ids[i];
        auto actor = actor_list->Find(actor_id);
        auto trail_actor = actor_list->Find(trail_id);
        if (actor != nullptr) {
            auto vehicle = boost::dynamic_pointer_cast<cc::Vehicle>(actor);
            if (vehicle != nullptr) {
                vehicle->Destroy();
            }
        }

        if(trail_actor != nullptr) {
            auto vehicle = boost::dynamic_pointer_cast<cc::Vehicle>(trail_actor);
            if (vehicle != nullptr) {
                vehicle->Destroy();
            }
        }
    }
    std::cerr << "pub shutdown" << std::endl;
}           


void SyncManager::TruckSizeSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    std::cerr << "TruckSizeSubCallback : "<< msg->data << std::endl;
    unique_lock<mutex> lock(mutex_);
    this->size = msg->data;
    truck_ids.resize(this->size);
    trailer_ids.resize(this->size);
}

double SyncManager::GetDistanceBetweenActors(ActorPtr current, ActorPtr target) {
    Transform target_transform = target->GetTransform();
    Transform current_transform = current->GetTransform();

    double extent_sum_x = 0.0;

    if (boost::dynamic_pointer_cast<cc::Vehicle>(target) || boost::dynamic_pointer_cast<cc::Walker>(target)) {
        //std::cerr << target->GetBoundingBox().extent.x <<"   " << current->GetBoundingBox().extent.x << std::endl;
        extent_sum_x = target->GetBoundingBox().extent.x + current->GetBoundingBox().extent.x;
    }

    //double distance = target_transform.location.Distance(current_transform.location);
    double distance = std::sqrt(std::pow(current_transform.location.x-target_transform.location.x,2));
    //std::cerr << "truck1" << current_transform.location.x << "trailor0" << target_transform.location.x << std::endl;
    distance -= extent_sum_x;
    //std::cerr << distance << std::endl;
    distance = std::max(0.0, distance) - 4.86f;

    return distance;
}

void SyncManager::recordData() {
    struct timeval currentTime;
    double diff_time;
    std::string log_path_ = "/home/nvidia/ros2_ws/logfiles/";

    auto actor_list = world->GetActors();

    gettimeofday(&currentTime, nullptr);
    //double sim_time = currentTime.tv_sec + (currentTime.tv_usec / 1e6);

    for (size_t i = 0; i < truck_ids.size(); ++i) {
        unsigned int actor_id = truck_ids[i];
        auto actor = actor_list->Find(actor_id);
        if (actor != nullptr) {
            auto vehicle = boost::dynamic_pointer_cast<cc::Vehicle>(actor);
            if (vehicle != nullptr) {
                // Get velocity and acceleration
                Vector3D velocity = vehicle->GetVelocity();
                Vector3D acceleration = vehicle->GetAcceleration();

                // Calculate the length of the velocity and acceleration vectors
                double velocity_length = std::sqrt(std::pow(velocity.x,2)  + std::pow(velocity.y,2) + std::pow(velocity.z,2)); 
                double acceleration_length = acceleration.x;

                // Placeholder for distance and flags (to be replaced with actual values)
                float distance_ = 0.0f;
                if (i > 0 && i - 1 < trailer_ids.size()) {
                    auto trailer_actor = actor_list->Find(trailer_ids[i - 1]);
                    if (trailer_actor != nullptr) {
                        distance_ = GetDistanceBetweenActors(actor, trailer_actor);
                        //std::cerr << distance_ << std::endl; 
                    }
                }
                float cut_in_flag = 0.0f;
                float sotif_flag = 0.0f;

                // Generate filename using the index in v_id
                std::string filename = "truck" + to_string(i + 1) + ".csv";
                std::string file_path = log_path_ + filename;

                // Check if file exists, if not, create and write header
                std::ifstream read_file(file_path);
                std::ofstream write_file;
                if (read_file.fail()) { 
                    write_file.open(file_path);
                    write_file << "Time, ActorID, Velocity, Acceleration, Distance, cut_in_flag, sotif_flag" << std::endl;
                }
                read_file.close();

                // Format and write data
                char buf[256] = {0x00,};
                sprintf(buf, "%.2f, %u, %.3f, %.3f, %.3f, %.3f, %.3f", 
                        sim_time, actor_id, velocity_length, acceleration_length, distance_, cut_in_flag, sotif_flag);

                write_file.open(file_path, std::ios::out | std::ios::app);
                write_file << buf << std::endl;
                write_file.close();
            }
        }
    }
}


void SyncManager::RegistrationSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    unique_lock<mutex> lock(mutex_);
    std::cerr << "RegistrationSubCallback : "<< msg->data << std::endl;
    registration_[msg->data] = true;
}


void SyncManager::SyncThrottleSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    unique_lock<mutex> lock(mutex_);
    //std::cerr << "throttle " << std::endl;
    sync_throttle[msg->data] = true;
}

void SyncManager::SyncSteerSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    unique_lock<mutex> lock(mutex_);
    //std::cerr << "steer" << std::endl;
    sync_steer[msg->data] = true;
}

bool SyncManager::check_register() {
    if(size == 0 ) return false;
    if(registered) return true;
    unique_lock<mutex> lock(mutex_);
    for(int i = 0; i<size; i++) {
        if(registration_[i] == false) return false;
    }
    //std::cerr << "tick for register " << std::endl;
    if(cnt == 0) {
        world->Tick(time_);    
        cnt = 1;
        for(int i = 0; i<size; i++) {
            if(registration_[i] == true) registration_[i] = false;
        }
        return false;
    }
    else if(cnt == 1) {
        registered = true;
        std::cerr << "All registered" << std::endl;
        world->Tick(time_);  
        return true;
    }
}

void SyncManager::FindAllTruck() {
    for(int i =0; i<this->size;i++) {
        std::string truck_name = "truck" + std::to_string(i);
        std::string trailer_name = "trailer" + std::to_string(i);
        auto actor_list = world->GetActors();
        for (auto iter = actor_list->begin(); iter != actor_list->end(); ++iter) {
            ActorPtr actor = *iter;
            ActorId actor_id = actor->GetId();
            if (actor->GetTypeId().front() == 'v') {
              
                for (auto&& attribute: actor->GetAttributes()) {
                  if (attribute.GetValue() == truck_name) {
                      unsigned int truck_id = actor_id;
                      truck_ids[i] = truck_id;
                  }
                  else if (attribute.GetValue() == trailer_name) {
                      unsigned int trailer_id = actor_id;
                      trailer_ids[i] = trailer_id; 
                  }
                }
            }
        } 

    }


}

bool SyncManager::sync_received() {
    unique_lock<mutex> lock(mutex_);
    // sync_throttle 배열의 모든 원소가 true인지 확인
    for (int i = 0; i < size; i++) {
        if (sync_throttle[i] == false) {
            return false; // 하나라도 false면 false 반환
        }
    }

    // sync_steer 배열의 모든 원소가 true인지 확인
    for (int i = 0; i < size; i++) {
        if (sync_steer[i] == false) {
            return false; // 하나라도 false면 false 반환
        }
    }


    //all received
    for(int i = 0; i<size; i++) {
        sync_throttle[i] = false;
    }
    for(int i = 0; i<size; i++) {
        sync_steer[i] = false;
    }    

    return true;


}

void SyncManager::managerInThread()
{
    while(isNodeRunning_) {
        if(check_register()) {
            if(sync_received() && first) {
                std::cerr << "tick" << sim_time<<std::endl;
                recordData();
                if(sim_time - 100.0f > 0.0f) {
                    isNodeRunning_ = false;
                    settings.synchronous_mode = false;
                    world->ApplySettings(settings,time_);
                    sleep(2);
                    std_msgs::msg::String msg;
                    msg.data = "down";
                    ShutdownPublisher_->publish(msg);
                    rclcpp::shutdown();
                }
                world->Tick(time_);
                sim_time += 0.01f;
            }
            if(!first) {
                FindAllTruck();
                first = true;
            }
        }



    }
}