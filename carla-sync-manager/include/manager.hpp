#include <iostream>
#include <fstream>
#include <string>
#include <sys/time.h>
#include <vector>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pthread.h>
#include <thread>
#include <shared_carlalib.h>
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/int32.hpp"
using namespace std;
class SyncManager : public rclcpp::Node {

public:
    SyncManager();
    ~SyncManager();
private:
    std::string host = "localhost";
    uint16_t port = 2000u;
    cc::Client* client;
    cc::World* world;
    carla::time_duration time_;
    carla::rpc::EpisodeSettings settings;
    bool first = false;
    bool first_check = false;
    mutex mutex_;
    bool isNodeRunning_ = false;
    bool registered = false;
    int size = 0;
    int cnt = 0;
    std::vector<bool> registration_;
    std::vector<bool> sync_throttle;
    std::vector<bool> sync_steer;
    std::thread manager_Thread;
    void managerInThread();


    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr TruckSizeSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr RegistrationSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr SyncSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr SyncThrottleSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr SyncSteerSubscriber_;


    //callback
    void TruckSizeSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void RegistrationSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void SyncThrottleSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void SyncSteerSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    bool check_register();
    bool sync_received();
};