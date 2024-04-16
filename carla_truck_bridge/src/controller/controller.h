#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QMainWindow>
#include <QApplication>
#include <QMutex>
#include <QDebug>
#include <QLocale>
#include <QTranslator>
#include <QString>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

//C++
#include <iostream>
#include <pthread.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <boost/thread/thread.hpp>
#include <vector>
#include <sys/time.h>
#include <condition_variable>
#include <functional>
#include <memory>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>

#include "lvthread.h"
#include "fv1thread.h"
#include "fv2thread.h"
#include "ros2node.hpp"
#include "ros2_msg/msg/cmd2xav.hpp"
#include "ros2_msg/msg/xav2cmd.hpp"

typedef ros2_msg::msg::Xav2cmd CmdData;

QT_BEGIN_NAMESPACE
namespace Ui { class Controller; }
QT_END_NAMESPACE

using namespace std::chrono_literals;

class Controller : public QMainWindow
{
    Q_OBJECT

public:
    Controller(const std::shared_ptr<Ros2Node>& ros2_node, QWidget *parent = nullptr);
    ~Controller();
    void sendData(int value_vel, int value_dist, int to);

    static QMutex lv_mutex_;
    static QMutex fv1_mutex_;
    static QMutex fv2_mutex_;

    static CmdData lv_data_;
    static CmdData fv1_data_;
    static CmdData fv2_data_;

    static int cnt;
    
    bool Emergency_flag = false;

    int MinVel;
    int MaxVel;
    int MinDist;
    int MaxDist;

    bool both_lc_flag = false;
//    bool wait_flag = false; 
    bool lv_wait_flag = false; 
    bool fv1_wait_flag = false; 
    bool fv2_wait_flag = false;

    bool lv_lc_complete = false;
    bool fv1_lc_complete = false;
    bool fv2_lc_complete = false;

    bool LV_lc_right = false;
    bool LV_lc_left = false;
    bool FV1_lc_right = false;
    bool FV1_lc_left = false;
    bool FV2_lc_right = false;
    bool FV2_lc_left = false;
    
    bool LV_Rear = false;
    bool FV1_Rear = false;
    bool FV2_Rear = false;

    // 1
    float lv_cur_dist_ = 0.0f;
    float fv1_cur_dist_ = 0.0f;
    float fv2_cur_dist_ = 0.0f;

    // 2
    float lv_est_dist_ = 0.0f;
    float fv1_est_dist_ = 0.0f;
    float fv2_est_dist_ = 0.0f;

    // 3
    float lv_r_est_dist_ = 0.0f;
    float fv1_r_est_dist_ = 0.0f;
    float fv2_r_est_dist_ = 0.0f;

    // 4
    float lv_est_vel_ = 0.0f;
    float fv1_est_vel_ = 0.0f;
    float fv2_est_vel_ = 0.0f;

    // 5
    float lv_r_est_vel_ = 0.0f;
    float fv1_r_est_vel_ = 0.0f;
    float fv2_r_est_vel_ = 0.0f;

    // 6
    int lv_bbox_ready_ = 3; //isbboxObject? 1:Yes,  2:No, 3:No_Msg
    int fv1_bbox_ready_ = 3;
    int fv2_bbox_ready_ = 3;

    // 7
    int lv_r_bbox_ready_ = 3;
    int fv1_r_bbox_ready_ = 3;
    int fv2_r_bbox_ready_ = 3;

private slots:
    void requestData(ros2_msg::msg::Cmd2xav cmd_data);

    void updateData(CmdData cmd_data);

    void replyData();

    void on_MVelSlider_valueChanged(int value);

    void on_MDistSlider_valueChanged(int value);

    void on_LVVelSlider_valueChanged(int value);

    void on_LVDistSlider_valueChanged(int value);

    void on_pushButton_clicked(bool checked);

    void on_LVBox_activated(int index);

    void on_FV1Box_activated(int index);

    void on_FV2Box_activated(int index);

    void on_Send_clicked();

    void on_FV2_Left_LC_toggled(bool checked);

    void on_FV2_Right_LC_toggled(bool checked);

    void on_FV1_Right_LC_toggled(bool checked);

    void on_FV1_Left_LC_toggled(bool checked);

    void on_LV_Right_LC_toggled(bool checked);

    void on_LV_Left_LC_toggled(bool checked);
    
    void on_LV_Rear_toggled(bool checked);

    void on_FV1_Rear_toggled(bool checked);

    void on_FV2_Rear_toggled(bool checked);


signals:
    void send(CmdData cmd_data);

private:
    Ui::Controller *ui;

    const std::shared_ptr<Ros2Node> ros2_node;

    //Publisher 
    rclcpp::Publisher<ros2_msg::msg::Cmd2xav>::SharedPtr XavPublisher_;

    //Subscriber 
    rclcpp::Subscription<ros2_msg::msg::Xav2cmd>::SharedPtr LVSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::Xav2cmd>::SharedPtr FV1Subscriber_;
    rclcpp::Subscription<ros2_msg::msg::Xav2cmd>::SharedPtr FV2Subscriber_;

    //Callback Func
    void XavSubCallback(const ros2_msg::msg::Xav2cmd &msg);

    cv::Mat display_Map(CmdData cmd_data);
    LVThread* lv_thread_;
    FV1Thread* fv1_thread_;
    FV2Thread* fv2_thread_;

    struct timeval startTime_;
    double time_ = 0.0;
    double req_time_ = 0.0;
    std::string log_path_ = "/home/avees/logfiles/";
    void recordData(struct timeval *time);
};
#endif // CONTROLLER_H
