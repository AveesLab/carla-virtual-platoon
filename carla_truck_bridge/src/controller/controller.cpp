#include "controller.h"
#include "ui_controller.h"

QMutex Controller::lv_mutex_;
QMutex Controller::fv1_mutex_;
QMutex Controller::fv2_mutex_;

CmdData Controller::lv_data_;
CmdData Controller::fv1_data_;
CmdData Controller::fv2_data_;

Controller::Controller(const std::shared_ptr<Ros2Node>& ros2_node, QWidget *parent)
  : QMainWindow(parent)
  , ui(new Ui::Controller)
  , ros2_node(ros2_node)
{
  ui->setupUi(this);

  std::string LVSubTopicName;
  int LVSubQueueSize;
  std::string FV1SubTopicName;
  int FV1SubQueueSize;
  std::string FV2SubTopicName;
  int FV2SubQueueSize;

  std::string XavPubTopicName;
  int XavPubQueueSize;

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  ros2_node->get_parameter_or("subscribers/lv_xavier_to_cmd/topic", LVSubTopicName, std::string("/LV/xav2cmd_msg"));
  ros2_node->get_parameter_or("subscribers/lv_xavier_to_cmd/queue_size", LVSubQueueSize, 10);
  ros2_node->get_parameter_or("subscribers/fv1_xavier_to_cmd/topic", FV1SubTopicName, std::string("/FV1/xav2cmd_msg"));
  ros2_node->get_parameter_or("subscribers/fv1_xavier_to_cmd/queue_size", FV1SubQueueSize, 1);
  ros2_node->get_parameter_or("subscribers/fv2_xavier_to_cmd/topic", FV2SubTopicName, std::string("/FV2/xav2cmd_msg"));
  ros2_node->get_parameter_or("subscribers/fv2_xavier_to_cmd/queue_size", FV2SubQueueSize, 1);

  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  ros2_node->get_parameter_or("publishers/cmd_to_xavier/topic", XavPubTopicName, std::string("/cmd2xav_msg"));
  ros2_node->get_parameter_or("publishers/cmd_to_xav/queue_size", XavPubQueueSize, 10);

  /******************/
  /* Ros Qos Option */
  /******************/
  rclcpp::QoS XavPubQos(XavPubQueueSize);
  XavPubQos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  XavPubQos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  rclcpp::QoS XavSubQos(LVSubQueueSize); // LV, FV1, FV2 same
  XavSubQos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  XavSubQos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  /************************/
  /* Ros Topic Subscriber */
  /************************/
  LVSubscriber_ = ros2_node->create_subscription<ros2_msg::msg::Xav2cmd>(LVSubTopicName, XavSubQos, std::bind(&Controller::XavSubCallback, this, std::placeholders::_1));
  FV1Subscriber_ = ros2_node->create_subscription<ros2_msg::msg::Xav2cmd>(FV1SubTopicName, XavSubQos, std::bind(&Controller::XavSubCallback, this, std::placeholders::_1));
  FV2Subscriber_ = ros2_node->create_subscription<ros2_msg::msg::Xav2cmd>(FV2SubTopicName, XavSubQos, std::bind(&Controller::XavSubCallback, this, std::placeholders::_1));

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  XavPublisher_ = ros2_node->create_publisher<ros2_msg::msg::Cmd2xav>(XavPubTopicName, XavPubQos);

  /**********************************/
  /* Control & Communication Thread */
  /**********************************/
  lv_thread_ = new LVThread(this);
  fv1_thread_ = new FV1Thread(this);
  fv2_thread_ = new FV2Thread(this);

  qRegisterMetaType<CmdData>("CmdData");

  //    connect(lv_thread_, SIGNAL(request(CmdData)), this, SLOT(requestData(CmdData)), Qt::DirectConnection);
  connect(lv_thread_, SIGNAL(setValue(CmdData)),this,SLOT(updateData(CmdData)), Qt::AutoConnection);
  //    connect(fv1_thread_, SIGNAL(request(CmdData)), this, SLOT(requestData(CmdData)), Qt::DirectConnection);
  connect(fv1_thread_, SIGNAL(setValue(CmdData)),this,SLOT(updateData(CmdData)), Qt::AutoConnection);
  //    connect(fv2_thread_, SIGNAL(request(CmdData)), this, SLOT(requestData(CmdData)), Qt::DirectConnection);
  connect(fv2_thread_, SIGNAL(setValue(CmdData)),this,SLOT(updateData(CmdData)), Qt::AutoConnection);

  lv_thread_->start();
  fv1_thread_->start();
  fv2_thread_->start();

  gettimeofday(&startTime_, NULL);

  int DefaultVel = 0; // cm/s
  MinVel = 0; // cm/s
  MaxVel = 9000; // cm/s
  int DefaultDist = 1400; // cm
  MinDist = 300; // cm
  MaxDist = 2000; // cm

  /* Setup Velocity Slider */
  ui->MVelSlider->setMaximum(MaxVel);
  ui->LVVelSlider->setMaximum(MaxVel);
  ui->FV1VelSlider->setMaximum(MaxVel);
  ui->FV2VelSlider->setMaximum(MaxVel);
  ui->MVelSlider->setMinimum(MinVel);
  ui->LVVelSlider->setMinimum(MinVel);
  ui->FV1VelSlider->setMinimum(MinVel);
  ui->FV2VelSlider->setMinimum(MinVel);
  ui->MVelSlider->setValue(DefaultVel);
  ui->LVVelSlider->setValue(DefaultVel);
  ui->FV1VelSlider->setValue(DefaultVel);
  ui->FV2VelSlider->setValue(DefaultVel);

  /* Setup Distance Slider */
  ui->MDistSlider->setMaximum(MaxDist);
  ui->LVDistSlider->setMaximum(MaxDist);
  ui->FV1DistSlider->setMaximum(MaxDist);
  ui->FV2DistSlider->setMaximum(MaxDist);
  ui->MDistSlider->setMinimum(MinDist);
  ui->LVDistSlider->setMinimum(MinDist);
  ui->FV1DistSlider->setMinimum(MinDist);
  ui->FV2DistSlider->setMinimum(MinDist);
  ui->MDistSlider->setValue(DefaultDist);
  ui->LVDistSlider->setValue(DefaultDist);
  ui->FV1DistSlider->setValue(DefaultDist);
  ui->FV2DistSlider->setValue(DefaultDist);

  /* Setup Velocity & Distance Bar */
  ui->LVVelBar->setMaximum(MaxVel);
  ui->FV1VelBar->setMaximum(MaxVel);
  ui->FV2VelBar->setMaximum(MaxVel);
  ui->LVVelBar->setMinimum(MinVel);
  ui->FV1VelBar->setMinimum(MinVel);
  ui->FV2VelBar->setMinimum(MinVel);
  ui->LVDistBar->setMaximum(MaxDist);
  ui->FV1DistBar->setMaximum(MaxDist);
  ui->FV2DistBar->setMaximum(MaxDist);
  ui->LVDistBar->setMinimum(MinDist);
  ui->FV1DistBar->setMinimum(MinDist);
  ui->FV2DistBar->setMinimum(MinDist);

  /* Setup Mode */
  ui->LVBox->setCurrentIndex(1);
  ui->FV1Box->setCurrentIndex(2);
  ui->FV2Box->setCurrentIndex(3);

  ros2_msg::msg::Cmd2xav default_data;
  float default_dist = DefaultDist/100.0f;
  default_data.src_index = 20;
  default_data.tar_index = 0;
  default_data.tar_vel = DefaultVel;
  default_data.tar_dist = default_dist;
  requestData(default_data);
}

Controller::~Controller()
{
  delete lv_thread_;
  delete fv1_thread_;
  delete fv2_thread_;
  delete ui;
}

void Controller::XavSubCallback(const ros2_msg::msg::Xav2cmd &msg)
{
  if(msg.src_index == 0){  //LV 
    lv_mutex_.lock();
    lv_data_ = msg;
    lv_mutex_.unlock();
  }
  else if(msg.src_index == 1){  //FV1 
    fv1_mutex_.lock();
    fv1_data_ = msg;
    fv1_mutex_.unlock();
  }
  else if(msg.src_index == 2){  //FV2 
    fv2_mutex_.lock();
    fv2_data_ = msg;
    fv2_mutex_.unlock();
  }
}

void Controller::recordData(struct timeval *startTime){
  struct timeval currentTime;
  char file_name[] = "CC_log00.csv";
  static char file[128] = {0x00, };
  char buf[256] = {0x00,};
  static bool flag = false;
  std::ifstream read_file;
  std::ofstream write_file;
  if(!flag){
    for(int i = 0; i < 100; i++){
      file_name[6] = i/10 + '0';  //ASCI
      file_name[7] = i%10 + '0';
      sprintf(file, "%s%s", log_path_.c_str(), file_name);
      read_file.open(file);
      if(read_file.fail()){  //Check if the file exists
        read_file.close();
        write_file.open(file);
        break;
      }
      read_file.close();
    }
    //write_file << "Time[s],Predict,Target,Current,Saturation,Estimate,Alpha" << endl;
    write_file << "Time,Request_time,Cur_dist" << std::endl; //seconds
    flag = true;
  }
  else{
    lv_mutex_.lock();
    gettimeofday(&currentTime, NULL);
    time_ = ((currentTime.tv_sec - startTime->tv_sec)) + ((currentTime.tv_usec - startTime->tv_usec)/1000000.0);
    //sprintf(buf, "%.3e,%.3f,%.3f,%.3f,%.3f,%.3f,%d", time_, est_vel_, tar_vel_, cur_vel_, sat_vel_, fabs(cur_vel_ - hat_vel_), alpha_);
    write_file.open(file, std::ios::out | std::ios::app);
    write_file << buf << std::endl;
    lv_mutex_.unlock();
  }
  write_file.close();
}

void Controller::requestData(ros2_msg::msg::Cmd2xav cmd_data)
{
  ros2_msg::msg::Cmd2xav send_data = cmd_data;
  struct timeval startTime, endTime;

  if (send_data.tar_index == 0){ // LV
    lv_mutex_.lock();
    gettimeofday(&startTime, NULL);
    XavPublisher_->publish(send_data);
    gettimeofday(&endTime, NULL);
    req_time_ = ((endTime.tv_sec - startTime.tv_sec)* 1000.0) + ((endTime.tv_usec - startTime.tv_usec)/1000.0);
    lv_mutex_.unlock();
    //recordData(&startTime_);
  }
  else if (send_data.tar_index == 1){ // FV1
    fv1_mutex_.lock();
    gettimeofday(&startTime, NULL);
    XavPublisher_->publish(send_data);
    gettimeofday(&endTime, NULL);
    req_time_ = ((endTime.tv_sec - startTime.tv_sec)* 1000.0) + ((endTime.tv_usec - startTime.tv_usec)/1000.0);
    fv1_mutex_.unlock();
    //recordData(&startTime_);
  }
  else if (send_data.tar_index == 2){ // FV2
    fv2_mutex_.lock();
    gettimeofday(&startTime, NULL);
    XavPublisher_->publish(send_data);
    gettimeofday(&endTime, NULL);
    req_time_ = ((endTime.tv_sec - startTime.tv_sec)* 1000.0) + ((endTime.tv_usec - startTime.tv_usec)/1000.0);
    fv2_mutex_.unlock();
    //recordData(&startTime_);
  }
  else {  // LV, FV1, FV2 
    XavPublisher_->publish(send_data);
  }
}

void Controller::replyData()
{
  ros2_msg::msg::Cmd2xav cmd_data;
  float tar_vel, tar_dist;

  int value_vel = ui->LVVelSlider->value();
  int value_dist = ui->LVDistSlider->value();

  if(value_vel >= 10) {
    tar_vel = value_vel/100.0f;
  }
  else {
    tar_vel = 0;
  }
  tar_dist = value_dist/100.0f;

  // All: LV, FV1, FV2
  cmd_data.src_index = 20;
  cmd_data.tar_index = 140; 
  cmd_data.tar_vel = tar_vel;
  cmd_data.tar_dist = tar_dist;

  cmd_data.lv_lc_left = LV_lc_left;
  cmd_data.fv1_lc_left = FV1_lc_left;
  cmd_data.fv2_lc_left = FV2_lc_left;

  cmd_data.lv_lc_right = LV_lc_right;
  cmd_data.fv1_lc_right = FV1_lc_right;
  cmd_data.fv2_lc_right = FV2_lc_right;

  cmd_data.lv_cur_dist = lv_cur_dist_;
  cmd_data.fv1_cur_dist = fv1_cur_dist_;
  cmd_data.fv2_cur_dist = fv2_cur_dist_;

  cmd_data.lv_est_vel = lv_est_vel_;
  cmd_data.fv1_est_vel = fv1_est_vel_;
  cmd_data.fv2_est_vel = fv2_est_vel_;

  cmd_data.lv_est_dist = lv_est_dist_;
  cmd_data.fv1_est_dist = fv1_est_dist_;
  cmd_data.fv2_est_dist = fv2_est_dist_;

  cmd_data.lv_r_est_dist = lv_r_est_dist_;
  cmd_data.fv1_r_est_dist = fv1_r_est_dist_;
  cmd_data.fv2_r_est_dist = fv2_r_est_dist_;

  cmd_data.lv_bbox_ready = lv_bbox_ready_;
  cmd_data.fv1_bbox_ready = fv1_bbox_ready_;
  cmd_data.fv2_bbox_ready = fv2_bbox_ready_;

  cmd_data.lv_r_bbox_ready = lv_r_bbox_ready_;
  cmd_data.fv1_r_bbox_ready = fv1_r_bbox_ready_;
  cmd_data.fv2_r_bbox_ready = fv2_r_bbox_ready_;

  requestData(cmd_data);
}

void Controller::updateData(CmdData cmd_data)
{
  CmdData tmp;
  int vel, dist;
  float deci = 100.0f;
  tmp = cmd_data;
  vel = tmp.cur_vel*deci; // cm/s
  dist = tmp.cur_dist*deci; // cm

  if(tmp.tar_index == 20){
    /******/
    /* LV */
    /******/
    if(tmp.src_index == 0)
    {
      ui->LVCurVel->setText(QString::number(vel/deci));
      if(vel > MaxVel) {
        vel = MaxVel;
      }
      ui->LVVelBar->setValue(vel);
      ui->LVCurDist->setText(QString::number(dist/deci));
      if(dist > MaxDist) {
        dist = MaxDist;
      }
      ui->LVDistBar->setValue(dist);
      cv::Mat frame;
      display_Map(tmp).copyTo(frame);
      ui->LV_MAP->setPixmap(QPixmap::fromImage(QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888)));

      if (lv_wait_flag == false && (tmp.lc_right_flag == true || tmp.lc_left_flag == true)) 
        lv_wait_flag = true;

      if (lv_wait_flag == true) {
        if(LV_lc_right == true) {
          if(tmp.lc_right_flag == false) {
            lv_wait_flag = false;
            lv_lc_complete = true;
            ui->LV_Right_LC->toggle(); 
          } 
        }
        if(LV_lc_left == true) {
          if(tmp.lc_left_flag == false) {
            lv_wait_flag = false;
            lv_lc_complete = true;
            ui->LV_Left_LC->toggle(); 
          } 
        }
      }

      lv_cur_dist_ = dist/deci;
      lv_est_dist_ = tmp.est_dist;
      lv_r_est_dist_ = tmp.r_est_dist;
      lv_r_est_vel_ = tmp.r_est_vel;
      lv_bbox_ready_ = tmp.bbox_ready;
      lv_r_bbox_ready_ = tmp.r_bbox_ready;
    }
    /*******/
    /* FV1 */
    /*******/
    else if (tmp.src_index == 1)
    {
      ui->FV1CurVel->setText(QString::number(vel/deci));
      if(vel > MaxVel) {
        vel = MaxVel;
      }
      ui->FV1VelBar->setValue(vel);
      ui->FV1CurDist->setText(QString::number(dist/deci));
      if(dist > MaxDist) {
        dist = MaxDist;
      }
      ui->FV1DistBar->setValue(dist);
      cv::Mat frame;
      display_Map(tmp).copyTo(frame);
      ui->FV1_MAP->setPixmap(QPixmap::fromImage(QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888)));

      if (fv1_wait_flag == false && (tmp.lc_right_flag == true || tmp.lc_left_flag == true)) 
        fv1_wait_flag = true;

      if (fv1_wait_flag == true) {
        if(FV1_lc_right == true) {
          if(tmp.lc_right_flag == false) {
            fv1_wait_flag = false;
            fv1_lc_complete = true;
            ui->FV1_Right_LC->toggle(); 
            ui->LV_Right_LC->toggle(); // FV1 -> LV flag on 
          } 
        }
        if(FV1_lc_left == true) {
          if(tmp.lc_left_flag == false) {
            fv1_wait_flag = false;
            fv1_lc_complete = true;
            ui->FV1_Left_LC->toggle(); 
            ui->LV_Left_LC->toggle(); // FV1 -> LV flag on 
          } 
        }
      }

      fv1_cur_dist_ = dist/deci;
      fv1_est_dist_ = tmp.est_dist;
      fv1_r_est_dist_ = tmp.r_est_dist;
      fv1_est_vel_ = tmp.est_vel;
      fv1_r_est_vel_ = tmp.r_est_vel;
      fv1_bbox_ready_ = tmp.bbox_ready;
      fv1_r_bbox_ready_ = tmp.r_bbox_ready;
    }
    /*******/
    /* FV2 */
    /*******/
    else if (tmp.src_index == 2)
    {
      ui->FV2CurVel->setText(QString::number(vel/deci));
      if(vel > MaxVel) {
        vel = MaxVel;
      }
      ui->FV2VelBar->setValue(vel);
      ui->FV2CurDist->setText(QString::number(dist/deci));
      if(dist > MaxDist) {
        dist = MaxDist;
      }
      ui->FV2DistBar->setValue(dist);
      cv::Mat frame;
      display_Map(tmp).copyTo(frame);
      ui->FV2_MAP->setPixmap(QPixmap::fromImage(QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888)));

      if (fv2_wait_flag == false && (tmp.lc_right_flag == true || tmp.lc_left_flag == true)) 
        fv2_wait_flag = true;

      if (fv2_wait_flag == true) {
        if(FV2_lc_right == true) { // Controller state -> lc_right flag on 
          if(tmp.lc_right_flag == false) { // FV2 state -> FV2 LC complete
            fv2_wait_flag = false;
            fv2_lc_complete = true;
            ui->FV2_Right_LC->toggle(); 
            ui->FV1_Right_LC->toggle(); // FV2 -> FV1 flag on 
          } 
        }
        if(FV2_lc_left == true) {
          if(tmp.lc_left_flag == false) {
            fv2_wait_flag = false;
            fv2_lc_complete = true;
            ui->FV2_Left_LC->toggle(); 
            ui->FV1_Left_LC->toggle(); // FV2 -> FV1 flag on
          } 
        }
      }
      
      fv2_cur_dist_ = dist/deci;
      fv2_est_dist_ = tmp.est_dist;
      fv2_r_est_dist_ = tmp.r_est_dist;
      fv2_est_vel_ = tmp.est_vel;
      fv2_r_est_vel_ = tmp.r_est_vel;
      fv2_bbox_ready_ = tmp.bbox_ready;
      fv2_r_bbox_ready_ = tmp.r_bbox_ready;
    }
  }
//  if(lv_lc_complete && fv1_lc_complete && fv2_lc_complete) both_lc_flag = false;
  
  if(LV_lc_left || FV1_lc_left || FV2_lc_left || LV_lc_right || FV1_lc_right || FV2_lc_right){
    replyData();
  }
}

cv::Mat Controller::display_Map(CmdData value)
{
  int width = 350;
  int height = 350;
  cv::Mat map_frame = cv::Mat::zeros(cv::Size(width,height), CV_8UC3);
  int check_dist = 50;
  for(int i=0;i<height;i+=check_dist)
    cv::line(map_frame, cv::Point(0,i), cv::Point(width,i),cv::Scalar::all(100));
  for(int i=0;i<width;i+=check_dist)
    cv::line(map_frame, cv::Point(i,0), cv::Point(i,height),cv::Scalar::all(100));

  int centerY = height-100, centerX=width/2;
  cv::rectangle(map_frame, cv::Rect(cv::Point(centerX-9, centerY+50), cv::Point(centerX+9, centerY)), cv::Scalar(20,20,255), -1);

  std::vector<cv::Point> RpointList;
  std::vector<cv::Point> LpointList;
  std::vector<cv::Point> CpointList;
  int cam_height = 480, cam_width = 640; // pixel
  float cam_y_dist = 98, cam_x_dist = 77; // cm
  if(LV_Rear == true || FV1_Rear == true || FV2_Rear == true) {
    value.coef = value.rear_coef;
  }
  for(int i = -550; i < cam_height*1.2; i++){
    cv::Point temp_point;
    temp_point.y = i*cam_y_dist/cam_height-26;
    temp_point.y += (centerY-cam_y_dist);
    temp_point.x = (value.coef[0].a*pow(i,2) + value.coef[0].b*i + value.coef[0].c)*cam_x_dist/cam_width;
    temp_point.x += centerX-cam_x_dist/2;
    RpointList.push_back(temp_point);
    temp_point.x = (value.coef[1].a*pow(i,2) + value.coef[1].b*i + value.coef[1].c)*cam_x_dist/cam_width;
    temp_point.x += centerX-cam_x_dist/2;
    LpointList.push_back(temp_point);
    temp_point.x = (value.coef[2].a*pow(i,2) + value.coef[2].b*i + value.coef[2].c)*cam_x_dist/cam_width;
    temp_point.x += centerX-cam_x_dist/2;
    CpointList.push_back(temp_point);
  }
  const cv::Point* right_points_point = (const cv::Point*) cv::Mat(RpointList).data;
  int right_points_number = cv::Mat(RpointList).rows;
  const cv::Point* left_points_point = (const cv::Point*) cv::Mat(LpointList).data;
  int left_points_number = cv::Mat(LpointList).rows;
  const cv::Point* center_points_point = (const cv::Point*) cv::Mat(CpointList).data;
  int center_points_number = cv::Mat(CpointList).rows;

  cv::polylines(map_frame, &right_points_point, &right_points_number, 1, false, cv::Scalar::all(255), 2);
  cv::polylines(map_frame, &left_points_point, &left_points_number, 1, false, cv::Scalar::all(255), 2);
  cv::polylines(map_frame, &center_points_point, &center_points_number, 1, false, cv::Scalar(200,255,200), 2);

  float r = value.cur_dist*100; // cm
  float theta = value.cur_angle*M_PI/180; // rad
  if(r < 250) {
    int X = r*sin(theta)+centerX;
    int Y = -r*cos(theta)+centerY;
    cv::circle(map_frame,cv::Point(X,Y), 5, cv::Scalar(50,50,255), -1);
  }

  //cv::line(map_frame, cv::Point(width/2 - check_dist,124 + value.roi_dist*100/490),cv::Point(width/2 + check_dist,124+value.roi_dist*100/490), cv::Scalar(255,100,100),3);

  cv::Mat swap_frame;
  cv::cvtColor(map_frame, swap_frame, cv::COLOR_BGR2RGB);
  return swap_frame;
}


void Controller::on_MVelSlider_valueChanged(int value)
{
  ui->MTarVel->setText(QString::number(value/100.0)); // m/s
  ui->LVVelSlider->setValue(value);
  ui->FV1VelSlider->setValue(value);
  ui->FV2VelSlider->setValue(value);
}

void Controller::on_MDistSlider_valueChanged(int value)
{
  ui->MTarDist->setText(QString::number(value/100.0)); // m
  ui->LVDistSlider->setValue(value);
  ui->FV1DistSlider->setValue(value);
  ui->FV2DistSlider->setValue(value);
}

void Controller::on_LVVelSlider_valueChanged(int value)
{
  int value_vel, value_dist;
  float tar_vel = 0.0f, tar_dist = 0.0f;

  value_vel = value;
  value_dist = ui->LVDistSlider->value();

  ui->LVVelSlider->setValue(value_vel);
  ui->FV1VelSlider->setValue(value_vel);
  ui->FV2VelSlider->setValue(value_vel);
  ui->LVTarVel->setText(QString::number(value/100.0)); // m/s
  ui->FV1TarVel->setText(QString::number(value/100.0)); // m/s
  ui->FV2TarVel->setText(QString::number(value/100.0)); // m/s
}

void Controller::on_LVDistSlider_valueChanged(int value)
{
  int value_vel, value_dist;
  float tar_vel = 0.0f, tar_dist = 0.0f;

  value_vel = ui->LVVelSlider->value();
  value_dist = value;

  ui->LVDistSlider->setValue(value_dist);
  ui->FV1DistSlider->setValue(value_dist);
  ui->FV2DistSlider->setValue(value_dist);
  ui->LVTarDist->setText(QString::number(value/100.0)); // m
  ui->FV1TarDist->setText(QString::number(value/100.0)); // m
  ui->FV2TarDist->setText(QString::number(value/100.0)); // m
}

void Controller::on_pushButton_clicked(bool checked)  //Emergency stop
{
  ros2_msg::msg::Cmd2xav cmd_data;
  float LV_dist = ui->LVDistSlider->value()/100.0f;
  ui->MVelSlider->setValue(0);
  ui->LVVelSlider->setValue(0);
  ui->FV1VelSlider->setValue(0);
  ui->FV2VelSlider->setValue(0);
  Emergency_flag = true;

  cmd_data.src_index = 20;
  cmd_data.tar_index = 0;
  cmd_data.tar_vel = 0;
  cmd_data.tar_dist = LV_dist;
  cmd_data.emergency_flag = Emergency_flag;
  requestData(cmd_data);
}

void Controller::on_LVBox_activated(int index)
{
  if(index == 0)
  {
    ui->LVVelSlider->setEnabled(true);
    ui->LVDistSlider->setEnabled(true);
  }
  else if(index == 1)
  {
    ui->LVVelSlider->setEnabled(true);
    ui->LVDistSlider->setEnabled(true);
  }
  else if(index == 2)
  {
    ui->LVVelSlider->setEnabled(false);
    ui->LVDistSlider->setEnabled(true);
  }
  else if(index == 3)
  {
    ui->LVVelSlider->setEnabled(false);
    ui->LVDistSlider->setEnabled(true);
  }
}


void Controller::on_FV1Box_activated(int index)
{
  if(index == 0)
  {
    ui->FV1VelSlider->setEnabled(true);
    ui->FV1DistSlider->setEnabled(true);
  }
  else if(index == 1)
  {
    ui->FV1VelSlider->setEnabled(true);
    ui->FV1DistSlider->setEnabled(true);
  }
  else if(index == 2)
  {
    ui->FV1VelSlider->setEnabled(false);
    ui->FV1DistSlider->setEnabled(true);
  }
  else if(index == 3)
  {
    ui->FV1VelSlider->setEnabled(false);
    ui->FV1DistSlider->setEnabled(true);
  }
}


void Controller::on_FV2Box_activated(int index)
{
  if(index == 0)
  {
    ui->FV2VelSlider->setEnabled(true);
    ui->FV2DistSlider->setEnabled(true);
  }
  else if(index == 1)
  {
    ui->FV2VelSlider->setEnabled(true);
    ui->FV2DistSlider->setEnabled(true);
  }
  else if(index == 2)
  {
    ui->FV2VelSlider->setEnabled(false);
    ui->FV2DistSlider->setEnabled(true);
  }
  else if(index == 3)
  {
    ui->FV2VelSlider->setEnabled(false);
    ui->FV2DistSlider->setEnabled(true);
  }
}

void Controller::on_Send_clicked()
{
  ros2_msg::msg::Cmd2xav cmd_data;
  float tar_vel, tar_dist;

  int vel = ui->MTarVel->text().split(" ")[0].toFloat()*100;
  ui->MVelSlider->setValue(vel);
  ui->LVVelSlider->setValue(vel);
  ui->FV1VelSlider->setValue(vel);
  ui->FV2VelSlider->setValue(vel);
  int dist = ui->MTarDist->text().split(" ")[0].toFloat()*100;
  ui->MDistSlider->setValue(dist);
  ui->LVDistSlider->setValue(dist);
  ui->FV1DistSlider->setValue(dist);
  ui->FV2DistSlider->setValue(dist);

  if(ui->LVVelSlider->value() >= 10) {
    tar_vel = ui->LVVelSlider->value()/100.0f;
  }
  else {
    tar_vel = 0;
  }
  tar_dist = ui->LVDistSlider->value()/100.0f;
  cmd_data.src_index = 20;
  cmd_data.tar_index = 0;
  cmd_data.tar_vel = tar_vel;
  cmd_data.tar_dist = tar_dist;
  qDebug() << "Send button";

  requestData(cmd_data);
}

void Controller::on_LV_Right_LC_toggled(bool checked)
{
  ros2_msg::msg::Cmd2xav cmd_data;
  float tar_vel, tar_dist;

  int value_vel = ui->LVVelSlider->value();
  int value_dist = ui->LVDistSlider->value();
  LV_lc_right = checked;

  if(value_vel >= 10) {
    tar_vel = value_vel/100.0f;
  }
  else {
    tar_vel = 0;
  }
  tar_dist = value_dist/100.0f;

  cmd_data.src_index = 20;
  cmd_data.tar_index = 0;
  cmd_data.tar_vel = tar_vel;
  cmd_data.tar_dist = tar_dist;
  cmd_data.lv_lc_right = LV_lc_right;
  qDebug() << "LV_lc_right: " << LV_lc_right;

  requestData(cmd_data);
}

void Controller::on_LV_Left_LC_toggled(bool checked)
{
  ros2_msg::msg::Cmd2xav cmd_data;
  float tar_vel, tar_dist;

  int value_vel = ui->LVVelSlider->value();
  int value_dist = ui->LVDistSlider->value();
  LV_lc_left = checked;

  if(value_vel >= 10) {
    tar_vel = value_vel/100.0f;
  }
  else {
    tar_vel = 0;
  }
  tar_dist = value_dist/100.0f;

  cmd_data.src_index = 20;
  cmd_data.tar_index = 0;
  cmd_data.tar_vel = tar_vel;
  cmd_data.tar_dist = tar_dist;
  cmd_data.lv_lc_left = LV_lc_left;
  qDebug() << "LV_lc_left: " << LV_lc_left;

  requestData(cmd_data);
}

void Controller::on_FV1_Right_LC_toggled(bool checked)
{
  ros2_msg::msg::Cmd2xav cmd_data;
  float tar_vel, tar_dist;

  int value_vel = ui->FV1VelSlider->value();
  int value_dist = ui->FV1DistSlider->value();
  FV1_lc_right = checked;

  if(value_vel >= 10) {
    tar_vel = value_vel/100.0f;
  }
  else {
    tar_vel = 0;
  }
  tar_dist = value_dist/100.0f;

  cmd_data.src_index = 20;
  cmd_data.tar_index = 1;
  cmd_data.tar_vel = tar_vel;
  cmd_data.tar_dist = tar_dist;
  cmd_data.fv1_lc_right = FV1_lc_right;
  qDebug() << "FV1_lc_right: " << FV1_lc_right;

  requestData(cmd_data);
}

void Controller::on_FV1_Left_LC_toggled(bool checked)
{
  ros2_msg::msg::Cmd2xav cmd_data;
  float tar_vel, tar_dist;

  int value_vel = ui->FV1VelSlider->value();
  int value_dist = ui->FV1DistSlider->value();
  FV1_lc_left = checked;

  if(value_vel >= 10) {
    tar_vel = value_vel/100.0f;
  }
  else {
    tar_vel = 0;
  }
  tar_dist = value_dist/100.0f;

  cmd_data.src_index = 20;
  cmd_data.tar_index = 1;
  cmd_data.tar_vel = tar_vel;
  cmd_data.tar_dist = tar_dist;
  cmd_data.fv1_lc_left = FV1_lc_left;
  qDebug() << "FV1_lc_left: " << FV1_lc_left;

  requestData(cmd_data);
}

void Controller::on_FV2_Left_LC_toggled(bool checked)
{
  ros2_msg::msg::Cmd2xav cmd_data;
  float tar_vel, tar_dist;

  int value_vel = ui->FV2VelSlider->value();
  int value_dist = ui->FV2DistSlider->value();
  FV2_lc_left = checked;

  if(value_vel >= 10) {
    tar_vel = value_vel/100.0f;
  }
  else {
    tar_vel = 0;
  }
  tar_dist = value_dist/100.0f;

  cmd_data.src_index = 20;
  cmd_data.tar_index = 2;
  cmd_data.tar_vel = tar_vel;
  cmd_data.tar_dist = tar_dist;
  cmd_data.fv2_lc_left = FV2_lc_left;
  qDebug() << "FV2_lc_left: " << FV2_lc_left;

//  if(both_lc_flag == false) {
//    ui->FV1_Left_LC->toggle(); 
//    ui->LV_Left_LC->toggle();  
//    both_lc_flag = true;
//  }
  requestData(cmd_data);
}

void Controller::on_FV2_Right_LC_toggled(bool checked)
{
  ros2_msg::msg::Cmd2xav cmd_data;
  float tar_vel, tar_dist;

  int value_vel = ui->FV2VelSlider->value();
  int value_dist = ui->FV2DistSlider->value();
  FV2_lc_right = checked;

  if(value_vel >= 10) {
    tar_vel = value_vel/100.0f;
  }
  else {
    tar_vel = 0;
  }
  tar_dist = value_dist/100.0f;

  cmd_data.src_index = 20;
  cmd_data.tar_index = 2;
  cmd_data.tar_vel = tar_vel;
  cmd_data.tar_dist = tar_dist;
  cmd_data.fv2_lc_right = FV2_lc_right;
  qDebug() << "FV2_lc_right: " << FV2_lc_right;

//  if(both_lc_flag == false) {
//    ui->FV1_Right_LC->toggle(); 
//    ui->LV_Right_LC->toggle();  
//    both_lc_flag = true;
//  }

  requestData(cmd_data);
}

void Controller::on_LV_Rear_toggled(bool checked)
{
  LV_Rear = checked;
  qDebug() << "LV_Rear";

}

void Controller::on_FV1_Rear_toggled(bool checked)
{
  FV1_Rear = checked;
  qDebug() << "FV1_Rear";

}

void Controller::on_FV2_Rear_toggled(bool checked)
{
  FV2_Rear = checked;
  qDebug() << "FV2_Rear";

}

