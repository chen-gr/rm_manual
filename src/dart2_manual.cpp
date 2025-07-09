//
// Created by chen_gr on 2025/3/9.
//

#include "rm_manual/dart2_manual.h"

namespace rm_manual
{
Dart2Manual::Dart2Manual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : ManualBase(nh, nh_referee)
{
  XmlRpc::XmlRpcValue dart_list, targets, launch_id, rotate_positions;
  nh.getParam("launch_id", launch_id);
  nh.getParam("dart_list", dart_list);
  nh.getParam("targets", targets);
  nh.getParam("rotate_positions", rotate_positions);
  getList(dart_list, targets, launch_id, rotate_positions);
  yaw_outpost_ = target_position_["outpost"][0];
  range_outpost_ = target_position_["outpost"][1];
  yaw_base_ = target_position_["base"][0];
  range_base_ = target_position_["base"][1];

  ros::NodeHandle nh_yaw = ros::NodeHandle(nh, "yaw");
  yaw_sender_ = new rm_common::JointPointCommandSender(nh_yaw, joint_state_);
  ros::NodeHandle nh_trigger = ros::NodeHandle(nh, "trigger");
  trigger_sender_ = new rm_common::JointPointCommandSender(nh_trigger, joint_state_);
  ros::NodeHandle nh_rotate = ros::NodeHandle(nh, "rotate");
  rotate_sender_ = new rm_common::JointPointCommandSender(nh_rotate, joint_state_);

  nh_trigger.getParam("trigger_home_command", trigger_home_command_);
  nh_trigger.getParam("trigger_work_command", trigger_work_command_);
  nh_trigger.getParam("trigger_confirm_home", trigger_confirm_home_);
  nh_trigger.getParam("trigger_confirm_work", trigger_confirm_work_);

  ros::NodeHandle nh_clamp = ros::NodeHandle(nh, "clamp_positions");
  nh_clamp.getParam("clamp_left_position", clamp_left_position_);
  nh_clamp.getParam("clamp_mid_position", clamp_mid_position_);
  nh_clamp.getParam("clamp_right_position", clamp_right_position_);
  nh_clamp.getParam("release_position", release_position_);
  nh_clamp.getParam("clamp_finish_position", clamp_finish_position_);

  ros::NodeHandle nh_belt_left = ros::NodeHandle(nh, "belt_left");
  ros::NodeHandle nh_belt_right = ros::NodeHandle(nh, "belt_right");
  belt_left_sender_ = new rm_common::JointPointCommandSender(nh_belt_left, joint_state_);
  belt_right_sender_ = new rm_common::JointPointCommandSender(nh_belt_right, joint_state_);
  nh_belt_left.getParam("belt_left_max", belt_left_max_);
  nh_belt_right.getParam("belt_right_max", belt_right_max_);
  nh_belt_left.getParam("belt_left_min", belt_left_min_);
  nh_belt_right.getParam("belt_right_min", belt_right_min_);
  nh_belt_left.getParam("belt_left_place", belt_left_place_);
  nh_belt_right.getParam("belt_right_place", belt_right_place_);
  nh_belt_left.getParam("belt_left_placed", belt_left_placed_);
  nh_belt_right.getParam("belt_right_placed", belt_right_placed_);
  nh_belt_left.getParam("upward_vel", upward_vel_);
  nh_belt_left.getParam("downward_vel", downward_vel_);

  ros::NodeHandle nh_range = ros::NodeHandle(nh, "range");
  range_sender_ = new rm_common::JointPointCommandSender(nh_range, joint_state_);

  ros::NodeHandle nh_clamp_left = ros::NodeHandle(nh, "clamp_left");
  ros::NodeHandle nh_clamp_mid = ros::NodeHandle(nh, "clamp_mid");
  ros::NodeHandle nh_clamp_right = ros::NodeHandle(nh, "clamp_right");
  clamp_left_sender_ = new rm_common::JointPointCommandSender(nh_clamp_left,joint_state_);
  clamp_mid_sender_ = new rm_common::JointPointCommandSender(nh_clamp_mid,joint_state_);
  clamp_right_sender_ = new rm_common::JointPointCommandSender(nh_clamp_right,joint_state_);
  clamp_position_.insert(std::make_pair(clamp_left_sender_, clamp_left_position_));
  clamp_position_.insert(std::make_pair(clamp_mid_sender_, clamp_mid_position_));
  clamp_position_.insert(std::make_pair(clamp_right_sender_, clamp_right_position_));

  XmlRpc::XmlRpcValue shooter_rpc_value, gimbal_rpc_value,clamp_rpc_value;
  nh.getParam("shooter_calibration", shooter_rpc_value);
  shooter_calibration_ = new rm_common::CalibrationQueue(shooter_rpc_value, nh, controller_manager_);
  nh.getParam("gimbal_calibration", gimbal_rpc_value);
  gimbal_calibration_ = new rm_common::CalibrationQueue(gimbal_rpc_value, nh, controller_manager_);
  nh.getParam("clamp_calibration",clamp_rpc_value);
  clamp_calibration_ = new rm_common::CalibrationQueue(clamp_rpc_value, nh, controller_manager_);

  ros::NodeHandle nh_camera = ros::NodeHandle(nh, "camera");
  nh_camera.getParam("camera_x_offset", camera_x_offset_);
  nh_camera.getParam("camera_y_offset", camera_y_offset_);
  nh_camera.getParam("long_camera_p_x", long_camera_p_x_);
  nh_camera.getParam("long_camera_p_y", long_camera_p_y_);
  nh_camera.getParam("short_camera_p_x", short_camera_p_x_);
  nh_camera.getParam("long_camera_x_threshold", long_camera_x_threshold_);
  nh_camera.getParam("long_camera_detach_threshold", retarget_threshold);

  left_switch_up_event_.setActiveHigh(boost::bind(&Dart2Manual::leftSwitchUpOn, this));
  left_switch_mid_event_.setActiveHigh(boost::bind(&Dart2Manual::leftSwitchMidOn, this));
  left_switch_down_event_.setActiveHigh(boost::bind(&Dart2Manual::leftSwitchDownOn, this));
  right_switch_down_event_.setActiveHigh(boost::bind(&Dart2Manual::rightSwitchDownOn, this));
  right_switch_mid_event_.setRising(boost::bind(&Dart2Manual::rightSwitchMidRise, this));
  right_switch_up_event_.setRising(boost::bind(&Dart2Manual::rightSwitchUpRise, this));
  wheel_clockwise_event_.setRising(boost::bind(&Dart2Manual::wheelClockwise, this));
  wheel_anticlockwise_event_.setRising(boost::bind(&Dart2Manual::wheelAntiClockwise, this));
  dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/rm_ecat_hw/dbus", 10, &Dart2Manual::dbusDataCallback, this);
  dart_client_cmd_sub_ = nh_referee.subscribe<rm_msgs::DartClientCmd>("dart_client_cmd_data", 10,
                                                                      &Dart2Manual::dartClientCmdCallback, this);
  game_robot_hp_sub_ =
      nh_referee.subscribe<rm_msgs::GameRobotHp>("game_robot_hp", 10, &Dart2Manual::gameRobotHpCallback, this);
  game_status_sub_ =
      nh_referee.subscribe<rm_msgs::GameStatus>("game_status", 10, &Dart2Manual::gameStatusCallback, this);
  short_camera_data_sub_ =
      nh.subscribe<rm_msgs::Dart>("/short_dart_camera_distance", 10, &Dart2Manual::shortCameraDataCallback, this);
  long_camera_data_sub_ =
      nh.subscribe<rm_msgs::Dart>("/dart_camera_distance", 10, &Dart2Manual::longCameraDataCallback, this);
}

void Dart2Manual::getList(const XmlRpc::XmlRpcValue& darts, const XmlRpc::XmlRpcValue& targets,
                          const XmlRpc::XmlRpcValue& launch_id, const XmlRpc::XmlRpcValue& rotate_positions)
{
  for (const auto& dart : darts)
  {
    ROS_ASSERT(dart.second.hasMember("param") and dart.second.hasMember("id"));
    ROS_ASSERT(dart.second["param"].getType() == XmlRpc::XmlRpcValue::TypeArray and
               dart.second["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    for (int i = 0; i < 4; ++i)
    {
      if (dart.second["id"] == launch_id[i])
      {
        Dart dart_info;
        dart_info.outpost_offset_ = static_cast<double>(dart.second["param"][0]);
        dart_info.outpost_range_ = static_cast<double>(dart.second["param"][1]);
        dart_info.base_offset_ = static_cast<double>(dart.second["param"][2]);
        dart_info.base_range_ = static_cast<double>(dart.second["param"][3]);
        dart_list_.insert(std::make_pair(i, dart_info));
      }
    }
  }
  for (const auto& target : targets)
  {
    ROS_ASSERT(target.second.hasMember("position"));
    ROS_ASSERT(target.second["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::vector<double> position(2);
    position[0] = static_cast<double>(target.second["position"][0]);
    position[1] = static_cast<double>(target.second["position"][1]);
    target_position_.insert(std::make_pair(target.first, position));
  }
  ROS_ASSERT(rotate_positions.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < rotate_positions.size(); i++)
  {
    auto& position = rotate_positions[i];
    ROS_ASSERT(position.hasMember("angle"));
    auto& angle_val = position["angle"];
    ROS_ASSERT(angle_val.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(angle_val.size() >= 2);  // 确保有至少两个元素
    auto angle0 = static_cast<double>(angle_val[0]);
    auto angle1 = static_cast<double>(angle_val[1]);
    rotate_place_position_.push_back(angle0);
    rotate_back_position_.push_back(angle1);
  }
  for (int i = 0; i < static_cast<int>(rotate_place_position_.size()); ++i)
  {
    std::cout <<"place i:"<<i<< "  num:"<< rotate_place_position_[i] << std::endl;
    std::cout << "back i:"<<i << "  num:"<< rotate_back_position_[i] << std::endl;
  }
}

void Dart2Manual::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
{
  ManualBase::gameRobotStatusCallback(data);
  robot_id_ = data->robot_id;
  if (!clamp_calibrate_)
  {
    clamp_calibration_ -> reset();
    clamp_calibrate_ = true;
  }
}

void Dart2Manual::remoteControlTurnOn()
{
  ManualBase::remoteControlTurnOn();
  gimbal_calibration_->stopController();
  gimbal_calibration_->reset();
  shooter_calibration_->stopController();
  shooter_calibration_->reset();
}

void Dart2Manual::updateLaunchMode()
{
  if (launch_mode_ != last_launch_mode_)
  {
    switch (launch_mode_)
    {
      case INIT:
        init();
        break;
      case PULLDOWN:
        pullDown();
        break;
      case ROTATE_PLACE:
        rotatePlace();
        break;
      case LOADING:
        loading();
        break;
      case ROTATE_BACK:
        rotateBack();
        break;
      case LOADED:
        loaded();
        break;
      case ENGAGE:
        engage();
        break;
      case PULLUP:
        pullUp();
        break;
      case READY:
        ready();
        break;
      case PUSH:
        push();
        break;
      default:
        ROS_WARN("Invalid mode.");
        break;
    }
  }
  last_launch_mode_ = launch_mode_;
}
void Dart2Manual::init()
{
  ROS_INFO("Enter INIT");
  shooter_calibration_->reset();
  belt_left_sender_->setPoint(0.0);
  belt_right_sender_->setPoint(0.0);
  trigger_sender_->setPoint(trigger_home_command_);
  rotate_sender_ -> setPoint(rotate_back_position_[dart_fired_num_]);
  if (last_launch_mode_ == PUSH)
    long_camera_x_after_push_ = long_camera_x_;
  auto_aim_state_ = NONE;
  if (dart_fired_num_ > 3)
    dart_fired_num_ = 0;
  last_init_time_ = ros::Time::now();
}
void Dart2Manual::pullDown()
{
  ROS_INFO("Enter PULLDOWN");
  trigger_sender_->setPoint(trigger_work_command_);
  belt_left_sender_->setPoint(downward_vel_);
  belt_right_sender_->setPoint(downward_vel_);
}
void Dart2Manual::rotatePlace()
{
  ROS_INFO("Enter ROTATE_PLACE");
  belt_left_sender_->setPoint(0.0);
  belt_right_sender_->setPoint(0.0);
  rotate_sender_ -> setPoint(rotate_place_position_[has_fired_num_]);
  confirm_place_ = true;
  last_rotate_time_ = ros::Time::now();
}

void Dart2Manual::loading()
{
  ROS_INFO("Enter LOADING");
  setGripperAction(dart_fired_num_);
  confirm_place_ = false;
  last_loading_time_ = ros::Time::now();
}

void Dart2Manual::rotateBack()
{
  ROS_INFO("Enter ROTATE_BACK");
  belt_left_sender_->setPoint(0.0);
  belt_right_sender_->setPoint(0.0);
  rotate_sender_ -> setPoint(rotate_back_position_[has_fired_num_]);
  confirm_back_ = true;
  last_rotate_time_ = ros::Time::now();
}

void Dart2Manual::loaded()
{
  ROS_INFO("Enter LOADED");
  belt_left_sender_->setPoint(downward_vel_);
  belt_right_sender_->setPoint(downward_vel_);
  confirm_back_ = false;
}

void Dart2Manual::engage()
{
  ROS_INFO("Enter ENGAGE");
  trigger_sender_->setPoint(trigger_home_command_);
  belt_left_sender_->setPoint(0.0);
  belt_right_sender_->setPoint(0.0);
  last_engage_time_ = ros::Time::now();
}

void Dart2Manual::pullUp()
{
  ROS_INFO("Enter PULLUP");
  //rotate_sender_ -> setPoint(rotate_back_position_[has_fired_num_]);
  belt_right_sender_->setPoint(upward_vel_);
  belt_left_sender_->setPoint(upward_vel_);
}
void Dart2Manual::ready()
{
  ROS_INFO("Enter READY");
  belt_right_sender_->setPoint(0.0);
  belt_left_sender_->setPoint(0.0);
  last_ready_time_ = ros::Time::now();
}
void Dart2Manual::push()
{
  ROS_INFO("Enter PUSH");
  trigger_sender_->setPoint(trigger_work_command_);
  dart_fired_num_++;
  ROS_INFO("Launch dart num:%d", dart_fired_num_);
  last_push_time_ = ros::Time::now();
  is_long_camera_aim_ = false;
}

void Dart2Manual::updateAutoAimState()
{
  switch (auto_aim_state_)
  {
    case AIM:
      if (last_auto_aim_state_ != auto_aim_state_)
        ROS_INFO("Enter AIM.");
      aim();
      break;
    case ADJUST:
      if (last_auto_aim_state_ != auto_aim_state_)
        ROS_INFO("Enter ADJUST.");
      adjust();
      break;
  }
  last_auto_aim_state_ = auto_aim_state_;
}

void Dart2Manual::aim()
{
  if (is_short_camera_found_ && !is_long_camera_found_)
  {
    short_camera_x_set_point_ += short_camera_x_ * short_camera_p_x_;
  }
  if (is_long_camera_found_)
  {
    long_camera_x_set_point_ += long_camera_x_ * long_camera_p_x_;
    long_camera_y_set_point_ = long_camera_y_ * long_camera_p_y_;
  }
  if (std::abs(long_camera_x_) <= long_camera_x_threshold_ && long_camera_x_ != 0 && yaw_velocity_ < 0.001)
  {
    ROS_INFO("The %d dart had aimed.Now x error:%lf, yaw position: %lf ", dart_fired_num_, long_camera_x_,
             joint_state_.position[yaw_sender_->getIndex()]);
    auto_aim_state_ = AIMED;
  }
}

void Dart2Manual::adjust()
{
  if (dart_fired_num_ == 0)
  {
    long_camera_x_set_point_ += dart_list_[dart_fired_num_].base_offset_;
    long_camera_y_set_point_ += dart_list_[dart_fired_num_].base_range_;
  }
  else
  {
    long_camera_x_set_point_ +=
        (dart_list_[dart_fired_num_].base_offset_ - dart_list_[dart_fired_num_ - 1].base_offset_);
    long_camera_y_set_point_ +=
        (dart_list_[dart_fired_num_].base_range_ - dart_list_[dart_fired_num_ - 1].base_range_);
  }
  if (yaw_velocity_ < 0.001)
  {
    ROS_INFO("The %d dart had adjusted.Now x error:%lf, yaw position: %lf ,range position: %lf", dart_fired_num_, long_camera_x_,
             joint_state_.position[yaw_sender_->getIndex()], joint_state_.position[range_sender_->getIndex()]);
    long_camera_x_before_push_ = long_camera_x_;
    auto_aim_state_ = ADJUSTED;
  }
}

void Dart2Manual::autoAim()
{
  camera_is_online_ = (ros::Time::now() - last_get_camera_data_time_ < ros::Duration(1.0));
  if (use_auto_aim_ && camera_is_online_)
  {
    if (dart_fired_num_ == 0)
      auto_aim_state_ = AIM;
    if (dart_fired_num_ != 0)
    {
      if (abs(long_camera_x_after_push_ - long_camera_x_before_push_) < retarget_threshold)
        auto_aim_state_ = ADJUST;
      else
        auto_aim_state_ = AIM;
    }
    if (auto_aim_state_ == AIMED)
      auto_aim_state_ = ADJUST;
  }
  else if (auto_aim_state_ != ADJUSTED)
    auto_aim_state_ = ADJUST;
}

void Dart2Manual::run()
{
  ManualBase::run();
  gimbal_calibration_->update(ros::Time::now());
  shooter_calibration_->update(ros::Time::now());
  clamp_calibration_->update(ros::Time::now());
  updateLaunchMode();
  updateAutoAimState();
  updateGripper();
}

void Dart2Manual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updateRc(dbus_data);
  if (std::abs(dbus_data->ch_r_x) > 0.1){
    move(yaw_sender_, dbus_data->ch_r_x);
  }
  if (std::abs(dbus_data->ch_r_y) > 0.1){
    move(range_sender_, dbus_data->ch_r_y);
  }
  operateGripper(dbus_data);
}

void Dart2Manual::sendCommand(const ros::Time& time)
{
  belt_left_sender_->sendCommand(time);
  belt_right_sender_->sendCommand(time);
  range_sender_->sendCommand(time);
  trigger_sender_->sendCommand(time);
  yaw_sender_->sendCommand(time);
  rotate_sender_->sendCommand(time);
  clamp_left_sender_->sendCommand(time);
  clamp_mid_sender_->sendCommand(time);
  clamp_right_sender_->sendCommand(time);
}

void Dart2Manual::checkReferee()
{
  ManualBase::checkReferee();
}

void Dart2Manual::move(rm_common::JointPointCommandSender* joint, double ch)
{
  if (!joint_state_.position.empty())
  {
    double position = joint_state_.position[joint->getIndex()];
    if (ch != 0.)
    {
      joint->setPoint(position - ch * scale_);
      if_stop_ = true;
    }
    if (ch == 0. && if_stop_)
    {
      joint->setPoint(joint_state_.position[joint->getIndex()]);
      if_stop_ = false;
    }
  }
}

void Dart2Manual::changeGripperState(rm_common::JointPointCommandSender* gripper, bool& is_release)
{
  if (is_release)
  {
    if (clamp_position_.find(gripper) != clamp_position_.end())
      gripper -> setPoint(clamp_position_.at(gripper));
    is_release = false;
  }
  else
  {
    gripper -> setPoint(release_position_);
    is_release = true;
  }
}

void Dart2Manual::changeAllGripperState()
{
  changeGripperState(clamp_left_sender_,left_clamp_is_release_);
  changeGripperState(clamp_mid_sender_,mid_clamp_is_release_);
  changeGripperState(clamp_right_sender_,right_clamp_is_release_);
}

void Dart2Manual::readyLaunchDart(int dart_fired_num)
{
  if (launch_mode_ == INIT && shooter_calibration_->isCalibrated() && range_velocity_ < 0.001 &&
      ros::Time::now() - last_init_time_ > ros::Duration(0.2) && last_init_time_ > last_push_time_)
    launch_mode_ = PULLDOWN;
  if (launch_mode_ == PULLDOWN && belt_left_position_ >= belt_left_place_ && belt_right_position_ >= belt_right_place_ &&
      dart_fired_num != 0)
    launch_mode_ = ROTATE_PLACE;
  if (launch_mode_ == ROTATE_PLACE && dart_fired_num != 0 && rotate_velocity_ < 0.01 && confirm_place_ &&
      ros::Time::now() - last_rotate_time_ > ros::Duration(0.6))
    launch_mode_ = LOADING;
  if (launch_mode_ == LOADING  && dart_fired_num != 0 && !confirm_place_ && ros::Time::now() - last_loading_time_ > ros::Duration(0.5))
    launch_mode_ = LOADED;
  if ((launch_mode_ == LOADED || launch_mode_ == PULLDOWN) && belt_left_position_ >= belt_left_max_ &&
      belt_right_position_ >= belt_right_max_)
    launch_mode_ = ENGAGE;
  if (launch_mode_ == ENGAGE && triggerIsHome() && ros::Time::now() - last_engage_time_ > ros::Duration(0.3))
    launch_mode_ = PULLUP;
  if (launch_mode_ == PULLUP && belt_left_position_ <= belt_left_min_ && belt_right_position_ <= belt_right_min_)
    launch_mode_ = READY;
}

void Dart2Manual::leftSwitchDownOn()
{
  launch_mode_ = INIT;
}

bool Dart2Manual::triggerIsWorked() const
{
  return trigger_position_ >= trigger_confirm_work_;
}

bool Dart2Manual::triggerIsHome() const
{
  return trigger_position_ <= trigger_confirm_home_;
}

void Dart2Manual::leftSwitchMidOn()
{
  switch (manual_state_)
  {
    case OUTPOST:
      yaw_sender_->setPoint(yaw_outpost_ + long_camera_x_set_point_ + short_camera_x_set_point_);
      range_sender_->setPoint(range_outpost_ + long_camera_y_set_point_);
      break;
    case BASE:
      yaw_sender_->setPoint(yaw_base_ + long_camera_x_set_point_ + short_camera_x_set_point_);
      range_sender_->setPoint(range_base_ + long_camera_y_set_point_);
      break;
  }
  readyLaunchDart(dart_fired_num_);
  if (launch_mode_ == READY)
  {
    autoAim();
  }
  //autoAim();
}

void Dart2Manual::leftSwitchUpOn()
{
  if (launch_mode_ == READY && (is_long_camera_aim_ || !is_short_camera_found_))
    launch_mode_ = PUSH;
}

void Dart2Manual::rightSwitchMidRise()
{
  ManualBase::rightSwitchMidRise();
  allow_dart_door_open_times_ = 0;
  move_state_ = NORMAL;
}

void Dart2Manual::rightSwitchDownOn()
{
  recordPosition(dbus_data_);
  if (dbus_data_.ch_l_y == 1.)
  {
    if (manual_state_ == OUTPOST)
    {
      yaw_sender_->setPoint(yaw_outpost_);
      range_sender_->setPoint(range_outpost_);
    }
    else if (manual_state_ == BASE)
    {
      yaw_sender_->setPoint(yaw_base_);
      range_sender_->setPoint(range_base_);
    }
  }
  if (dbus_data_.ch_l_y == -1.)
  {
    if (manual_state_ == OUTPOST)
    {
      yaw_sender_->setPoint(target_position_["outpost"][0]);
      range_sender_->setPoint(target_position_["outpost"][1]);
    }
    else if (manual_state_ == BASE)
    {
      yaw_sender_->setPoint(target_position_["base"][0]);
      range_sender_->setPoint(target_position_["base"][1]);
    }
  }
}

void Dart2Manual::rightSwitchUpRise()
{
  ManualBase::rightSwitchUpRise();
  launch_mode_ = INIT;
}

void Dart2Manual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updatePc(dbus_data);
  double velocity_threshold = 1.0;
  if (range_velocity_ < velocity_threshold && yaw_velocity_ < velocity_threshold)
    move_state_ = STOP;
  else
    move_state_ = MOVING;
  if (referee_is_online_)
  {
    switch (auto_state_)
    {
      case OUTPOST:
        // yaw_sender_->setPoint(yaw_outpost_ + dart_list_[dart_fired_num_].outpost_offset_);
        // b_sender_->setPoint(b_outpost_);
        yaw_sender_->setPoint(yaw_base_ + long_camera_x_set_point_ + short_camera_x_set_point_);
        range_sender_->setPoint(range_base_ + long_camera_y_set_point_);
        break;
      case BASE:
        yaw_sender_->setPoint(yaw_base_ + long_camera_x_set_point_ + short_camera_x_set_point_);
        range_sender_->setPoint(range_base_ + long_camera_y_set_point_);
        break;
    }
    if (dart_launch_opening_status_ != 1 && game_progress_ != rm_msgs::GameStatus::IN_BATTLE)
      autoAim();
    if (game_progress_ == rm_msgs::GameStatus::IN_BATTLE)
    {
      if (last_dart_door_status_ - dart_launch_opening_status_ ==
          rm_msgs::DartClientCmd::OPENING_OR_CLOSING - rm_msgs::DartClientCmd::OPENED)
      {
        has_fired_num_ = 0;
        ROS_INFO("has fired num: %d", has_fired_num_);
      }
      if (last_dart_door_status_ - dart_launch_opening_status_ ==
          rm_msgs::DartClientCmd::OPENED - rm_msgs::DartClientCmd::OPENING_OR_CLOSING)
      {
        allow_dart_door_open_times_--;
        ROS_INFO("allow dart_door open times_: %d", allow_dart_door_open_times_);
      }
      if (allow_dart_door_open_times_ > 0)
      {
        readyLaunchDart(dart_fired_num_);
        if (launch_mode_ == READY)
          autoAim();
        if (dart_launch_opening_status_ == rm_msgs::DartClientCmd::OPENED)
        {
          switch (move_state_)
          {
            case MOVING:
              break;
            case STOP:
              if (has_fired_num_ < 2)
              {
                readyLaunchDart(dart_fired_num_);
                if (launch_mode_ == READY && ros::Time::now() - last_ready_time_ > ros::Duration(0.3) &&
                    last_ready_time_ > last_push_time_ && auto_aim_state_ == ADJUSTED)
                {
                  launch_mode_ = PUSH;
                  has_fired_num_++;
                  ROS_INFO("has fired_num_=%d", has_fired_num_);
                }
                if (launch_mode_ == PUSH && last_push_time_ > last_ready_time_ &&
                    ros::Time::now() - last_push_time_ > ros::Duration(0.3))
                {
                  ROS_INFO("now time: %f; last push time: %f", ros::Time::now().toSec(), last_push_time_.toSec());
                  launch_mode_ = INIT;
                }
              }
              break;
          }
        }
      }
    }
    last_dart_door_status_ = dart_launch_opening_status_;
  }
}

void Dart2Manual::recordPosition(const rm_msgs::DbusData dbus_data)
{
  if (dbus_data.ch_r_y == 1.)
  {
    if (manual_state_ == OUTPOST)
    {
      yaw_outpost_ = joint_state_.position[yaw_sender_->getIndex()];
      range_outpost_ = joint_state_.position[range_sender_->getIndex()];
      ROS_INFO("Recorded outpost position.");
    }
    else if (manual_state_ == BASE)
    {
      yaw_base_ = joint_state_.position[yaw_sender_->getIndex()];
      range_base_ = joint_state_.position[range_sender_->getIndex()];
      ROS_INFO("Recorded base position.");
    }
  }
}

void Dart2Manual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  if (!joint_state_.name.empty())
  {
    belt_left_position_ = std::abs(joint_state_.position[belt_left_sender_->getIndex()]);
    belt_right_position_ = std::abs(joint_state_.position[belt_right_sender_->getIndex()]);
    trigger_position_ = joint_state_.position[trigger_sender_->getIndex()];
    yaw_velocity_ = std::abs(joint_state_.velocity[yaw_sender_->getIndex()]);
    range_velocity_ = std::abs(joint_state_.velocity[range_sender_->getIndex()]);
    rotate_velocity_ = std::abs(joint_state_.velocity[rotate_sender_->getIndex()]);
  }
  wheel_clockwise_event_.update(data->wheel == 1.0);
  wheel_anticlockwise_event_.update(data->wheel == -1.0);
  dbus_data_ = *data;
}

void Dart2Manual::updateAllowDartDoorOpenTimes()
{
  int elapsed_time = 420 - remain_time_;
  if (!triggered_30s_ && elapsed_time > 30)
  {
    allow_dart_door_open_times_++;
    triggered_30s_ = true;
    ROS_INFO("30s into the match. allow dart door open times: %d", allow_dart_door_open_times_);
  }
  if (!triggered_4min_ && elapsed_time > 240)
  {
    allow_dart_door_open_times_++;
    triggered_4min_ = true;
    if (launch_mode_ == PUSH)
      launch_mode_ = INIT;
    ROS_INFO("420s into the match. allow dart door open times: %d", allow_dart_door_open_times_);
  }
}

void Dart2Manual::updateGripper(){
  if (last_gripper_state_ != gripper_state_)
  {
    switch (gripper_state_) {
      case LEFT:
        ROS_INFO("Left gripper change state.");
        changeGripperState(clamp_left_sender_, left_clamp_is_release_);
        break;
      case MID:
        ROS_INFO("Mid gripper change state.");
        changeGripperState(clamp_mid_sender_, mid_clamp_is_release_);
        break;
      case RIGHT:
        ROS_INFO("Right gripper change state.");
        changeGripperState(clamp_right_sender_, right_clamp_is_release_);
        break;
      case ALL:
        ROS_INFO("All gripper change state.");
        changeAllGripperState();
        break;
    }
  }
  last_gripper_state_ = gripper_state_;
}

void Dart2Manual::gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
{
  ManualBase::gameStatusCallback(data);
  game_progress_ = data->game_progress;
  if (game_progress_ == rm_msgs::GameStatus::IN_BATTLE)
    remain_time_ = data->stage_remain_time;
  else
    remain_time_ = 420;
  updateAllowDartDoorOpenTimes();
}

void Dart2Manual::dartClientCmdCallback(const rm_msgs::DartClientCmd::ConstPtr& data)
{
  dart_launch_opening_status_ = data->dart_launch_opening_status;
}

void Dart2Manual::gameRobotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data)
{
  switch (robot_id_)
  {
    case rm_msgs::GameRobotStatus::RED_DART:
      outpost_hp_ = data->blue_outpost_hp;
      break;
    case rm_msgs::GameRobotStatus::BLUE_DART:
      outpost_hp_ = data->red_outpost_hp;
      break;
  }
  if (outpost_hp_ != 0)
    auto_state_ = OUTPOST;
  else
    auto_state_ = BASE;
}

void Dart2Manual::wheelClockwise()
{
  switch (move_state_)
  {
    case NORMAL:
      scale_ = scale_micro_;
      move_state_ = MICRO;
      ROS_INFO("Pitch and yaw : MICRO_MOVE_MODE");
      break;
    case MICRO:
      scale_ = 0.001;
      move_state_ = NORMAL;
      ROS_INFO("Pitch and yaw : NORMAL_MOVE_MODE");
      break;
  }
}

void Dart2Manual::wheelAntiClockwise()
{
  switch (manual_state_)
  {
    case OUTPOST:
      manual_state_ = BASE;
      ROS_INFO("Friction wheels : BASE_MODE");
      yaw_sender_->setPoint(yaw_base_);
      break;
    case BASE:
      manual_state_ = OUTPOST;
      ROS_INFO("Friction wheels : OUTPOST_MODE");
      yaw_sender_->setPoint(yaw_outpost_);
      break;
  }
}

void Dart2Manual::longCameraDataCallback(const rm_msgs::Dart::ConstPtr& data)
{
  is_long_camera_found_ = data->is_found;
  long_camera_x_ = data->distance;
  long_camera_y_ = data->height;
  last_get_camera_data_time_ = data->stamp;
  // ROS_INFO("time error:%f",(ros::Time::now() - data->stamp).toSec());
}

void Dart2Manual::shortCameraDataCallback(const rm_msgs::Dart::ConstPtr& data)
{
  is_short_camera_found_ = data->is_found;
  short_camera_x_ = data->distance;
  short_camera_y_ = data->height;
  // last_get_camera_data_time_ = data->stamp;
}

void Dart2Manual::setGripperAction(int dart_fired_num)
{
  switch (dart_fired_num)
  {
    case 0:
      break;
    case 1:
      clamp_left_sender_ -> setPoint(release_position_);
      left_clamp_is_release_ = true;
      break;
    case 2:
      clamp_mid_sender_ -> setPoint(release_position_);
      clamp_left_sender_ -> setPoint(clamp_finish_position_);
      mid_clamp_is_release_ = true;
      left_clamp_is_release_ = false;
      break;
    case 3:
      clamp_right_sender_ -> setPoint(release_position_);
      clamp_mid_sender_ -> setPoint(clamp_finish_position_);
      right_clamp_is_release_ = true;
      mid_clamp_is_release_ = false;
      break;
  }
}

void Dart2Manual::operateGripper(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  if (dbus_data->ch_l_y > 0.75)
  {
    gripper_state_ = MID;
    return;
  }
  if (dbus_data->ch_l_x > 0.75)
  {
    gripper_state_ = RIGHT;
    return;
  }
  if (dbus_data->ch_l_y < -0.75)
  {
    gripper_state_ = ALL;
    return;
  }
  if (dbus_data->ch_l_x < -0.75)
  {
    gripper_state_ = LEFT;
    return;
  }
  gripper_state_ = ZERO;
}

}  // namespace rm_manual
