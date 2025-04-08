//
// Created by chen_gr on 2025/3/9.
//

#include "rm_manual/dart2_manual.h"

namespace rm_manual
{
Dart2Manual::Dart2Manual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : ManualBase(nh, nh_referee)
{
  XmlRpc::XmlRpcValue dart_list,targets,launch_id,arm_positions;
  nh.getParam("launch_id", launch_id);
  nh.getParam("dart_list", dart_list);
  nh.getParam("targets", targets);
  nh.getParam("arm_positions", arm_positions);
  getList(dart_list, targets, launch_id,arm_positions);

  ros::NodeHandle nh_yaw = ros::NodeHandle(nh, "yaw");
  yaw_sender_ = new rm_common::JointPointCommandSender(nh_yaw, joint_state_);

  ros::NodeHandle nh_trigger = ros::NodeHandle(nh, "trigger");
  trigger_sender_ = new rm_common::JointPointCommandSender(nh_trigger, joint_state_);
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
  nh_a_left.getParam("upward_vel", upward_vel_);
  nh_a_left.getParam("downward_vel", downward_vel_);

  ros::NodeHandle nh_b = ros::NodeHandle(nh, "b");
  b_sender_ = new rm_common::JointPointCommandSender(nh_b,joint_state_);

  XmlRpc::XmlRpcValue shooter_rpc_value, gimbal_rpc_value;
  nh.getParam("shooter_calibration", shooter_rpc_value);
  shooter_calibration_ = new rm_common::CalibrationQueue(shooter_rpc_value, nh, controller_manager_);
  nh.getParam("gimbal_calibration", gimbal_rpc_value);
  gimbal_calibration_ = new rm_common::CalibrationQueue(gimbal_rpc_value, nh, controller_manager_);

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
  arm_position_pub_ = nh.advertise<dart_msgs::armPosition>("/arm_position",1);
}

void Dart2Manual::getList(const XmlRpc::XmlRpcValue& darts, const XmlRpc::XmlRpcValue& targets,
                         const XmlRpc::XmlRpcValue& launch_id,const XmlRpc::XmlRpcValue& arm_positions)
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
        dart_info.outpost_b_ = static_cast<double>(dart.second["param"][1]);
        dart_info.base_offset_ = static_cast<double>(dart.second["param"][2]);
        dart_info.base_b_ = static_cast<double>(dart.second["param"][3]);
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
  for (const auto& arm_position : arm_positions)
  {
    ROS_ASSERT(arm_position.second.hasMember("position"));
    ROS_ASSERT(arm_position.second["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::vector<double> position(5);
    position[0] = static_cast<double>(arm_position.second["position"][0]);
    position[1] = static_cast<double>(arm_position.second["position"][1]);
    position[2] = static_cast<double>(arm_position.second["position"][2]);
    position[3] = static_cast<double>(arm_position.second["position"][3]);
    position[4] = static_cast<double>(arm_position.second["position"][4]);
    arm_position_.insert(std::make_pair(arm_position.first,position));
  }
}

void Dart2Manual::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
{
  ManualBase::gameRobotStatusCallback(data);
  robot_id_ = data->robot_id;
}

void Dart2Manual::gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
{
  ManualBase::gameStatusCallback(data);
  game_progress_ = data->game_progress;
}

void Dart2Manual::remoteControlTurnOn()
{
  ManualBase::remoteControlTurnOn();
  gimbal_calibration_->stopController();
  gimbal_calibration_->reset();
  shooter_calibration_->stopController();
  shooter_calibration_->reset();
}

void Dart2Manual::run()
{
  ManualBase::run();
  gimbal_calibration_->update(ros::Time::now());
  shooter_calibration_->update(ros::Time::now());
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
}

void Dart2Manual::checkReferee()
{
  ManualBase::checkReferee();
  arm_position_pub_.publish(arm_position_msg_data_);
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
  if (!ready_)
  {
    trigger_sender_->setPoint(0.02);
    a_left_sender_->setPoint(downward_vel_);
    a_right_sender_->setPoint(downward_vel_);
    if (!first_send_)
    {
      setArmPosition(arm_position_[getOrdinalName(dart_fired_num)]);
      first_send_ = true;
      last_send_time_ = ros::Time::now();
    }
    if (ros::Time::now() - last_send_time_ > ros::Duration(0.5) && !central_send_ )
    {
      setArmGripperPosition(arm_get_position_);
      last_send_time_ = ros::Time::now();
      central_send_ = true;
    }
    if (ros::Time::now() - last_send_time_ > ros::Duration(0.5) && central_send_)
    {
      setArmPosition(arm_position_["central"]);
    }
  }
  if (a_left_position_>= a_left_max_ && a_right_position_>= a_right_max_)
  {
    if (!ready_ && triggerIsWorked())
    {
      trigger_sender_->setPoint(0.7);
      ready_ = true;
    }
    a_left_sender_->setPoint(0.0);
    a_right_sender_->setPoint(0.0);
  }
  if (triggerIsHome())
  {
    setArmGripperPosition(arm_put_position_);
    a_right_sender_->setPoint(upward_vel_);
    a_left_sender_->setPoint(upward_vel_);
  }
  if (ready_ && a_left_position_<=a_left_min_ && a_right_position_<=a_right_min_)
  {
    setArmPosition(arm_position_["init"]);
    a_left_sender_->setPoint(0.0);
    a_right_sender_->setPoint(0.0);
    all_ready = true;
  }
}

void Dart2Manual::leftSwitchDownOn()
{
  trigger_sender_->setPoint(0.7);
  ready_ = false;
  dart_fired_num_ = 0;
  a_left_sender_->setPoint(0.0);
  a_right_sender_->setPoint(0.0);
  setArmPosition(arm_position_["init"]);
  first_send_ = false;
  central_send_ = false;
}

bool Dart2Manual::triggerIsWorked() const
{
  return trigger_position_ <= trigger_work_;
}

bool Dart2Manual::triggerIsHome() const
{
  return trigger_position_ >= trigger_home_;
}
void Dart2Manual::leftSwitchMidOn()
{
  switch (manual_state_)
  {
    case OUTPOST:
      yaw_sender_->setPoint(yaw_outpost_ + dart_list_[dart_fired_num_].outpost_offset_);
      b_sender_->setPoint(b_outpost_ + dart_list_[dart_fired_num_].outpost_offset_);
      break;
    case BASE:
      yaw_sender_->setPoint(yaw_base_ + dart_list_[dart_fired_num_].base_offset_);
      b_sender_->setPoint(b_base_ + dart_list_[dart_fired_num_].base_offset_);
    break;
  }
  readyLaunchDart(dart_fired_num_);
}

void Dart2Manual::leftSwitchUpOn()
{
  trigger_sender_->setPoint(0.02);

  setArmPosition(arm_position_["init"]);
  ready_ = false;
  first_send_ = false;
  central_send_ = false;

  dart_fired_num_++;
}

void Dart2Manual::rightSwitchMidRise()
{
  ManualBase::rightSwitchMidRise();
  dart_door_open_times_ = 0;
  initial_dart_fired_num_ = 0;
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
  yaw_sender_->setPoint(yaw_outpost_);
  b_sender_->setPoint(b_outpost_);
}

void Dart2Manual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updatePc(dbus_data);
  double velocity_threshold = 0.001;
  if (b_velocity_ < velocity_threshold && yaw_velocity_ < velocity_threshold)
    move_state_ = STOP;
  else
    move_state_ = MOVING;
  if (game_progress_ == rm_msgs::GameStatus::IN_BATTLE)
  {
    switch (auto_state_)
    {
      case OUTPOST:
        yaw_sender_->setPoint(yaw_outpost_ + dart_list_[dart_fired_num_].outpost_offset_);
      b_sender_->setPoint(b_outpost_);
      break;
      case BASE:
        yaw_sender_->setPoint(yaw_base_ + dart_list_[dart_fired_num_].base_offset_);
      b_sender_->setPoint(b_base_);
      break;
    }
    if (last_dart_door_status_ - dart_launch_opening_status_ ==
    rm_msgs::DartClientCmd::OPENING_OR_CLOSING - rm_msgs::DartClientCmd::OPENED)
    {
      dart_door_open_times_++;
      has_fired_num_=0;
    }
    if (move_state_ == STOP && launch_state_ != PUSH)
      launch_state_ = READY;
    else if (launch_state_ != PUSH && launch_state_ != READY)
      launch_state_ = NONE;
    if (dart_launch_opening_status_ == rm_msgs::DartClientCmd::OPENED)
    {
      switch (launch_state_)
      {
        case NONE:
          trigger_sender_->setPoint(0.7);
          a_left_sender_->setPoint(0.0);
          a_right_sender_->setPoint(0.0);
          setArmPosition(arm_position_["init"]);
          break;
        case READY:
          if (has_fired_num_ <2)
          {
            if (has_launch )
            {
              if (ros::Time::now()-last_launch_time_>ros::Duration(2.0))
              {
                readyLaunchDart(dart_fired_num_);
                if (all_ready)
                {
                  launch_state_ = PUSH;
                  has_launch = false;
                }
              }
            }
            else
            {
              readyLaunchDart(dart_fired_num_);
              if (all_ready)
              {
                launch_state_ = PUSH;
                has_launch = false;
              }
            }
          }
        break;
        case PUSH:
          trigger_sender_->setPoint(0.02);
          if (!has_launch)
            has_fired_num_++;
          has_launch = true;
          dart_fired_num_++;
          launch_state_ = READY;
          setArmPosition(arm_position_["init"]);
          all_ready = false;
          ready_ = false;
          first_send_ = false;
          central_send_ = false;
          break;
      }
    }
  }
}

void Dart2Manual::recordPosition(const rm_msgs::DbusData dbus_data)
{
  if (dbus_data.ch_r_y == 1.)
  {
    if (manual_state_ == OUTPOST)
    {
      yaw_outpost_ = joint_state_.position[yaw_sender_->getIndex()];
      b_outpost_=joint_state_.position[b_sender_->getIndex()];
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
    trigger_position_= std::abs(joint_state_.position[trigger_sender_->getIndex()]);
    yaw_velocity_ = std::abs(joint_state_.velocity[yaw_sender_->getIndex()]);
    b_velocity_ = std::abs(joint_state_.velocity[b_sender_->getIndex()]);
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
      throw std::out_of_range("Invalid dart index: " + std::to_string(dart_index));
  }
}
void Dart2Manual::setArmPosition(const std::vector<double>& joint_positions) {
  constexpr int REQUIRED_JOINTS = 5;

  if (joint_positions.size() < REQUIRED_JOINTS) {
    throw std::invalid_argument("Invalid joint positions size: expected "
        + std::to_string(REQUIRED_JOINTS) + ", got "
        + std::to_string(joint_positions.size()));
  }

  std::array<double*, REQUIRED_JOINTS> joints = {
    &arm_position_msg_data_.joint1_pos,
    &arm_position_msg_data_.joint2_pos,
    &arm_position_msg_data_.joint3_pos,
    &arm_position_msg_data_.joint4_pos,
    &arm_position_msg_data_.joint5_pos
  };

  for (size_t i = 0; i < REQUIRED_JOINTS; ++i) {
    *joints[i] = joint_positions[i];
  }
}
void Dart2Manual::setArmGripperPosition(double position)
{
  arm_position_msg_data_.joint5_pos=position;
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
      scale_ = 0.04;
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
} // namespace rm_manual