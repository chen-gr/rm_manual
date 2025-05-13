//
// Created by chen_gr on 2025/3/9.
//

#include "rm_manual/dart2_manual.h"

namespace rm_manual
{
Dart2Manual::Dart2Manual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : ManualBase(nh, nh_referee)
{
  XmlRpc::XmlRpcValue dart_list,targets,launch_id,rotate_positions;
  nh.getParam("launch_id", launch_id);
  nh.getParam("dart_list", dart_list);
  nh.getParam("targets", targets);
  nh.getParam("rotate_positions", rotate_positions);
  getList(dart_list, targets, launch_id, rotate_positions);
  yaw_outpost_ = target_position_["outpost"][0];
  b_outpost_ = target_position_["outpost"][1];
  yaw_base_ = target_position_["base"][0];
  b_base_ = target_position_["base"][1];

  first_place_ = rotate_position_["first_position"][0];
  first_back_ = rotate_position_["first_position"][1];
  second_place_ = rotate_position_["second_position"][0];
  second_back_ = rotate_position_["second_position"][1];
  third_place_ = rotate_position_["third_position"][0];
  third_back_ = rotate_position_["third_position"][1];

  ros::NodeHandle nh_yaw = ros::NodeHandle(nh, "yaw");
  yaw_sender_ = new rm_common::JointPointCommandSender(nh_yaw, joint_state_);
  ros::NodeHandle nh_trigger = ros::NodeHandle(nh, "trigger");
  trigger_sender_ = new rm_common::JointPointCommandSender(nh_trigger, joint_state_);
  ros::NodeHandle nh_rotate = ros::NodeHandle(nh, "rotate");
  rotate_sender_ = new rm_common::JointPointCommandSender(nh_rotate, joint_state_);

  nh_trigger.getParam("trigger_home", trigger_home_);
  nh_trigger.getParam("trigger_work", trigger_work_);
  nh_trigger.getParam("trigger_confirm_home", trigger_confirm_home_);
  nh_trigger.getParam("trigger_confirm_work", trigger_confirm_work_);

  ros::NodeHandle nh_a_left = ros::NodeHandle(nh, "a_left");
  ros::NodeHandle nh_a_right = ros::NodeHandle(nh, "a_right");
  a_left_sender_ = new rm_common::JointPointCommandSender(nh_a_left, joint_state_);
  a_right_sender_ = new rm_common::JointPointCommandSender(nh_a_right, joint_state_);
  nh_a_left.getParam("a_left_max", a_left_max_);
  nh_a_right.getParam("a_right_max", a_right_max_);
  nh_a_left.getParam("a_left_min", a_left_min_);
  nh_a_right.getParam("a_right_min", a_right_min_);
  nh_a_left.getParam("a_left_place", a_left_place_);
  nh_a_right.getParam("a_right_place", a_right_place_);
  nh_a_left.getParam("a_left_placed", a_left_placed_);
  nh_a_right.getParam("a_right_placed", a_right_placed_);
  nh_a_left.getParam("upward_vel", upward_vel_);
  nh_a_left.getParam("downward_vel", downward_vel_);

  ros::NodeHandle nh_b = ros::NodeHandle(nh, "b");
  b_sender_ = new rm_common::JointPointCommandSender(nh_b,joint_state_);

  XmlRpc::XmlRpcValue shooter_rpc_value, gimbal_rpc_value;
  nh.getParam("shooter_calibration", shooter_rpc_value);
  shooter_calibration_ = new rm_common::CalibrationQueue(shooter_rpc_value, nh, controller_manager_);
  nh.getParam("gimbal_calibration", gimbal_rpc_value);
  gimbal_calibration_ = new rm_common::CalibrationQueue(gimbal_rpc_value, nh, controller_manager_);

  ros::NodeHandle nh_camera = ros::NodeHandle(nh, "camera");
  nh_camera.getParam("camera_x_offset", camera_x_offset_);
  nh_camera.getParam("camera_y_offset", camera_y_offset_);
  nh_camera.getParam("long_camera_p_x", long_camera_p_x_);
  nh_camera.getParam("long_camera_p_y", long_camera_p_y_);
  nh_camera.getParam("short_camera_p_x", short_camera_p_x_);
  nh_camera.getParam("long_camera_x_threshold", long_camera_x_threshold_);
  nh_camera.getParam("long_camera_detach_threshold", long_camera_detach_threshold_);

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
  short_camera_data_sub_ = nh.subscribe<rm_msgs::Dart>("/short_dart_camera_distance", 10,&Dart2Manual::shortCameraDataCallback, this);
  long_camera_data_sub_ = nh.subscribe<rm_msgs::Dart>("/dart_camera_distance", 10,&Dart2Manual::longCameraDataCallback, this);
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
        dart_info.outpost_tension_ = static_cast<double>(dart.second["param"][1]);
        dart_info.base_offset_ = static_cast<double>(dart.second["param"][2]);
        dart_info.base_tension_ = static_cast<double>(dart.second["param"][3]);
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
  for (const auto& rotate_position : rotate_positions)
  {
    ROS_ASSERT(rotate_position.second.hasMember("angle"));
    ROS_ASSERT(rotate_position.second["angle"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::vector<double> angle(2);
    angle[0] = static_cast<double>(rotate_position.second["angle"][0]);
    angle[1] = static_cast<double>(rotate_position.second["angle"][1]);
    rotate_position_.insert(std::make_pair(rotate_position.first, angle));
  }
}

void Dart2Manual::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
{
  ManualBase::gameRobotStatusCallback(data);
  robot_id_ = data->robot_id;
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
        rotate_place();
        break;
      case LOADING:
        loading();
        break;
      case ROTATE_BACK:
        rotate_back();
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
  a_left_sender_->setPoint(0.0);
  a_right_sender_->setPoint(0.0);
  trigger_sender_->setPoint(trigger_home_);
  if (dart_fired_num_ > 3)
    dart_fired_num_ = 0;
  if (dart_fired_num_ == 0)
    rotate_sender_->setPoint(1.04017);
  last_init_time_ = ros::Time::now();
}
void Dart2Manual::pullDown()
{
  ROS_INFO("Enter PULLDOWN");
  trigger_sender_->setPoint(trigger_work_);
  a_left_sender_->setPoint(downward_vel_);
  a_right_sender_->setPoint(downward_vel_);
}
void Dart2Manual::rotate_place()
{
  ROS_INFO("Enter ROTATE_PLACE");
  a_left_sender_->setPoint(0.0);
  a_right_sender_->setPoint(0.0);
  switch(dart_fired_num_)
  {
    case 1:
      rotate_sender_->setPoint(first_place_);
      break;
    case 2:
      rotate_sender_->setPoint(second_place_);
      break;
    case 3:
      rotate_sender_->setPoint(third_place_);
      break;
    default:
      ROS_INFO("Unknown fired num for place");
      break;
  }
  confirm_place_ = true;
  last_rotate_time_ = ros::Time::now();
}

void Dart2Manual::loading()
{
  ROS_INFO("Enter LOADING");
  a_left_sender_->setPoint(upward_vel_ + 1.2);
  a_right_sender_->setPoint(upward_vel_ + 1.2);
  confirm_place_ = false;
}

void Dart2Manual::rotate_back()
{
  ROS_INFO("Enter ROTATE_BACK");
  a_left_sender_->setPoint(0.0);
  a_right_sender_->setPoint(0.0);
  switch(dart_fired_num_)
  {
    case 1:
      rotate_sender_->setPoint(first_back_);
      break;
    case 2:
      rotate_sender_->setPoint(second_back_);
      break;
    case 3:
      rotate_sender_->setPoint(third_back_);
      break;
    default:
      ROS_INFO("Unknown fired num for back");
      break;
  }
  confirm_back_ = true;
  last_rotate_time_ = ros::Time::now();
}

void Dart2Manual::loaded()
{
  ROS_INFO("Enter LOADED");
  a_left_sender_->setPoint(downward_vel_);
  a_right_sender_->setPoint(downward_vel_);
  confirm_back_ = false;
}

void Dart2Manual::engage()
{
  ROS_INFO("Enter ENGAGE");
  trigger_sender_->setPoint(trigger_home_);
  a_left_sender_->setPoint(0.0);
  a_right_sender_->setPoint(0.0);
  last_engage_time_ = ros::Time::now();
}

void Dart2Manual::pullUp()
{
  ROS_INFO("Enter PULLUP");
  a_right_sender_->setPoint(upward_vel_);
  a_left_sender_->setPoint(upward_vel_);
}
void Dart2Manual::ready()
{
  ROS_INFO("Enter READY");
  a_right_sender_->setPoint(0.0);
  a_left_sender_->setPoint(0.0);
  last_ready_time_ = ros::Time::now();

}
void Dart2Manual::push()
{
  ROS_INFO("Enter PUSH");
  trigger_sender_->setPoint(trigger_work_);
  dart_fired_num_++;
  ROS_INFO("Launch dart num:%d",dart_fired_num_);
  last_push_time_ = ros::Time::now();
  is_long_camera_aim_ = false;
}
void Dart2Manual::run()
{
  ManualBase::run();
  gimbal_calibration_->update(ros::Time::now());
  shooter_calibration_->update(ros::Time::now());
  updateLaunchMode();
}

void Dart2Manual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updateRc(dbus_data);
  move(yaw_sender_, dbus_data->ch_l_x);
  move(b_sender_, dbus_data->ch_r_y);
}

void Dart2Manual::sendCommand(const ros::Time& time)
{
  a_left_sender_->sendCommand(time);
  a_right_sender_->sendCommand(time);
  b_sender_->sendCommand(time);
  trigger_sender_->sendCommand(time);
  yaw_sender_->sendCommand(time);
  rotate_sender_->sendCommand(time);
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

void Dart2Manual::readyLaunchDart(int dart_fired_num)
{
  if (launch_mode_ == INIT && shooter_calibration_->isCalibrated() && b_velocity_ < 0.001 && ros::Time::now() - last_init_time_ > ros::Duration(0.3) && last_init_time_ > last_push_time_)
    launch_mode_ = PULLDOWN;
  if (launch_mode_ == PULLDOWN && a_left_position_ >= a_left_place_ && a_right_position_ >= a_right_place_ && dart_fired_num !=0)
    launch_mode_ = ROTATE_PLACE;
  if (launch_mode_ == ROTATE_PLACE && dart_fired_num !=0 && rotate_velocity_ < 0.01 && confirm_place_ && ros::Time::now() - last_rotate_time_ > ros::Duration(0.9))
      launch_mode_ = LOADING;
  if (launch_mode_ == LOADING && a_left_position_ <= a_left_placed_ && a_right_position_ <= a_right_placed_ && dart_fired_num !=0)
    launch_mode_ = ROTATE_BACK;
  if (launch_mode_ == ROTATE_BACK && dart_fired_num !=0 && rotate_velocity_ < 0.01 && confirm_back_ && ros::Time::now() - last_rotate_time_ > ros::Duration(0.6))
      launch_mode_ = LOADED;
  if ((launch_mode_ == LOADED || launch_mode_ == PULLDOWN) && a_left_position_ >= a_left_max_ && a_right_position_ >= a_right_max_)
    launch_mode_ = ENGAGE;
  if (launch_mode_ == ENGAGE && triggerIsHome() && ros::Time::now() - last_engage_time_ > ros::Duration(0.5))
    launch_mode_ = PULLUP;
  if (launch_mode_ == PULLUP && a_left_position_ <= a_left_min_ && a_right_position_ <= a_right_min_)
    launch_mode_ = READY;
}


void Dart2Manual::leftSwitchDownOn()
{
  launch_mode_ = INIT;
}

bool Dart2Manual::triggerIsWorked() const
{
  return trigger_position_ <= trigger_confirm_work_;
}

bool Dart2Manual::triggerIsHome() const
{
  return trigger_position_ >= trigger_confirm_home_;
}
void Dart2Manual::leftSwitchMidOn()
{
  switch (manual_state_)
  {
    case OUTPOST:
      yaw_sender_->setPoint(yaw_outpost_ + long_camera_x_set_point_ + short_camera_x_set_point_);
      b_sender_->setPoint(b_outpost_ + long_camera_y_set_point_);
      break;
    case BASE:
      yaw_sender_->setPoint(yaw_base_  + long_camera_x_set_point_ + short_camera_x_set_point_);
      b_sender_->setPoint(b_base_  + long_camera_y_set_point_);
    break;
  }
  readyLaunchDart(dart_fired_num_);
  if(launch_mode_ == READY)
  {
      updateCameraData();
  }
 //updateCameraData();
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
      b_sender_->setPoint(b_outpost_);
    }
    else if (manual_state_ == BASE)
    {
      yaw_sender_->setPoint(yaw_base_);
      b_sender_->setPoint(b_base_);
    }
  }
  if (dbus_data_.ch_l_y == -1.)
  {
    if (manual_state_ == OUTPOST)
    {
      yaw_sender_->setPoint(target_position_["outpost"][0]);
      b_sender_->setPoint(target_position_["outpost"][1]);
    }
    else if (manual_state_ == BASE)
    {
      yaw_sender_->setPoint(target_position_["base"][0]);
      b_sender_->setPoint(target_position_["base"][1]);
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
  if (b_velocity_ < velocity_threshold && yaw_velocity_ < velocity_threshold)
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
        yaw_sender_->setPoint(yaw_base_  + long_camera_x_set_point_ + short_camera_x_set_point_);
        b_sender_->setPoint(b_base_  + long_camera_y_set_point_);
        break;
      case BASE:
        yaw_sender_->setPoint(yaw_base_ + long_camera_x_set_point_ + short_camera_x_set_point_);
        b_sender_->setPoint(b_base_ + long_camera_y_set_point_);
        break;
    }
    if (game_progress_ == rm_msgs::GameStatus::IN_BATTLE)
    {
      if (last_dart_door_status_ - dart_launch_opening_status_ ==
rm_msgs::DartClientCmd::OPENING_OR_CLOSING - rm_msgs::DartClientCmd::OPENED)
      {
        has_fired_num_=0;
        ROS_INFO("has fired num: %d",has_fired_num_);
      }
      if (last_dart_door_status_ - dart_launch_opening_status_ == rm_msgs::DartClientCmd::OPENED - rm_msgs::DartClientCmd::OPENING_OR_CLOSING)
      {
        allow_dart_door_open_times_ --;
        ROS_INFO("allow dart_door open times_: %d", allow_dart_door_open_times_);
      }
      if (allow_dart_door_open_times_ > 0)
      {
        readyLaunchDart(dart_fired_num_);
        // ROS_INFO("Dart2Manual::updatePc: launch ready");
        if (launch_mode_ == READY)
          updateCameraData();
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
                if (launch_mode_ == READY && ros::Time::now() - last_ready_time_ > ros::Duration(1.0) && last_ready_time_ > last_push_time_ && is_long_camera_aim_)
                {
                  launch_mode_ = PUSH;
                  has_fired_num_++;
                  ROS_INFO("has fired_num_=%d", has_fired_num_);
                }
                if (launch_mode_ == PUSH && last_push_time_ > last_ready_time_ && ros::Time::now() - last_push_time_ > ros::Duration(1.0))
                {
                  ROS_INFO("now time: %f; last push time: %f",ros::Time::now().toSec(),last_push_time_.toSec());
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
      b_outpost_ = joint_state_.position[b_sender_->getIndex()];
      ROS_INFO("Recorded outpost position.");
    }
    else if (manual_state_ == BASE)
    {
      yaw_base_ = joint_state_.position[yaw_sender_->getIndex()];
      b_base_ = joint_state_.position[b_sender_->getIndex()];
      ROS_INFO("Recorded base position.");
    }
  }
}

void Dart2Manual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  if (!joint_state_.name.empty())
  {
    a_left_position_= std::abs(joint_state_.position[a_left_sender_->getIndex()]);
    a_right_position_= std::abs(joint_state_.position[a_right_sender_->getIndex()]);
    trigger_position_= joint_state_.position[trigger_sender_->getIndex()];
    yaw_velocity_ = std::abs(joint_state_.velocity[yaw_sender_->getIndex()]);
    b_velocity_ = std::abs(joint_state_.velocity[b_sender_->getIndex()]);
    rotate_velocity_ = std::abs(joint_state_.velocity[rotate_sender_->getIndex()]);
  }
  wheel_clockwise_event_.update(data->wheel == 1.0);
  wheel_anticlockwise_event_.update(data->wheel == -1.0);
  dbus_data_ = *data;
}

std::string Dart2Manual::getOrdinalName(int dart_index) {
  switch (dart_index) {
    case 0: return "first";
    case 1: return "second";
    case 2: return "third";
    case 3: return "fourth";
    default:
      ROS_ERROR("invalid dart_index: %d", dart_index);
      return "init";
  }
}

void Dart2Manual::updateAllowDartDoorOpenTimes()
{
  int elapsed_time = 420 - remain_time_;
  if (!triggered_30s_ && elapsed_time > 30)
  {
    allow_dart_door_open_times_++;
    triggered_30s_ = true;
    ROS_INFO("30s into the match. allow dart door open times: %d",allow_dart_door_open_times_);
  }
  if (!triggered_4min_ && elapsed_time > 240)
  {
    allow_dart_door_open_times_++;
    triggered_4min_ = true;
    if (launch_mode_ == PUSH)
      launch_mode_ = INIT;
    ROS_INFO("420s into the match. allow dart door open times: %d",allow_dart_door_open_times_);
  }
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

void Dart2Manual::updateCameraData()
{
   if(abs(long_camera_x_ - last_camera_x) > abs(long_camera_x_))
   {
       camera_central_ = true;
       long_camera_x_set_point_ = last_long_camera_x_set_point - long_camera_x_ * long_camera_p_x_ * 100;
   }
  if (is_short_camera_found_ && !is_long_camera_found_)
  {
    short_camera_x_set_point_ += short_camera_x_ * short_camera_p_x_;
  }
  if (is_long_camera_found_ && !is_long_camera_aim_)
  {
    long_camera_x_set_point_ += long_camera_x_ * long_camera_p_x_;
    long_camera_y_set_point_ = long_camera_y_ * long_camera_p_y_;
    if (std::abs(long_camera_x_) <= long_camera_x_threshold_ && long_camera_x_ != 0 && yaw_velocity_ < 0.001) {
        is_long_camera_aim_ = true;
        is_adjust_ = false;
    }
    //ROS_INFO("long camera set point:%f",long_camera_x_set_point_);
  }
  if(!is_long_camera_found_)
  {
      is_long_camera_aim_= false;
      camera_central_ = false;
  }
  if (is_long_camera_aim_ && !is_adjust_)
  {
    //long_camera_x_set_point_ += camera_x_offset_;
   // long_camera_y_set_point_ += camera_y_offset_;
    long_camera_x_set_point_ += dart_list_[dart_fired_num_].base_offset_;
    long_camera_y_set_point_ += dart_list_[dart_fired_num_].base_tension_;
      camera_central_ = false;
      is_adjust_ = true;
      ROS_INFO("aim");
  }
    last_camera_x = long_camera_x_;
  last_long_camera_x_set_point = long_camera_x_set_point_;
}

void Dart2Manual::longCameraDataCallback(const rm_msgs::Dart::ConstPtr& data)
{
  is_long_camera_found_ = data->is_found;
  long_camera_x_ = data->distance;
  long_camera_y_ = data->height;
  //ROS_INFO("time error:%f",(ros::Time::now() - data->stamp).toSec());
}

void Dart2Manual::shortCameraDataCallback(const rm_msgs::Dart::ConstPtr& data)
{
  is_short_camera_found_ = data->is_found;
  short_camera_x_ = data->distance;
  short_camera_y_ = data->height;
}

} // namespace rm_manual