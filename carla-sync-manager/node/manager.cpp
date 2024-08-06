#include "manager.hpp"

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
}           


void SyncManager::TruckSizeSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    std::cerr << "TruckSizeSubCallback : "<< msg->data << std::endl;
    unique_lock<mutex> lock(mutex_);
    this->size = msg->data;
}


void SyncManager::RegistrationSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    unique_lock<mutex> lock(mutex_);
    std::cerr << "RegistrationSubCallback : "<< msg->data << std::endl;
    registration_[msg->data] = true;
}


void SyncManager::SyncThrottleSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    unique_lock<mutex> lock(mutex_);
    std::cerr << "throttle " << std::endl;
    sync_throttle[msg->data] = true;
}

void SyncManager::SyncSteerSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    unique_lock<mutex> lock(mutex_);
    std::cerr << "steer" << std::endl;
    sync_steer[msg->data] = true;
}

bool SyncManager::check_register() {
    if(size == 0 ) return false;
    if(registered) return true;
    unique_lock<mutex> lock(mutex_);
    for(int i = 0; i<size; i++) {
        if(registration_[i] == false) return false;
    }
    std::cerr << "tick for register " << std::endl;
    world->Tick(time_);    
    if(cnt == 0) {
        cnt = 1;
        for(int i = 0; i<size; i++) {
            if(registration_[i] == true) registration_[i] = false;
        }
        return false;
    }
    else if(cnt == 1) {
        registered = true;
        std::cerr << "All registered" << std::endl;
        return true;
    }
}

bool SyncManager::sync_received() {
    unique_lock<mutex> lock(mutex_);
    for(int i = 0; i<size; i++) {
        if(sync_throttle[i] == false) {
            //std::cerr << "throttle " << i << std::endl;
            return false;
        }
    }
    for(int i = 0; i<size; i++) {
        if(sync_steer[i] == false) {
            //std::cerr << "steer " << i << std::endl;
            return false;
        }
    }
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
                std::cerr << "tick" << std::endl;
                world->Tick(time_);
            }
            if(!first) {
                first = true;
            }
        }



    }
}