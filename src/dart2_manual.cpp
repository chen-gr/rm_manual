//
// Created by chen_gr on 2025/3/9.
//

#include "rm_manual/dart2_manual.h"

namespace rm_manual
{
Dart2Manual::Dart2Manual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : ManualBase(nh, nh_referee)
{
  XmlRpc::XmlRpcValue dart_list,targets,launch_id;
  nh.getParam("launch_id", launch_id);
  nh.getParam("dart_list", dart_list);
  nh.getParam("targets", targets);
  getList(dart_list, targets, launch_id);
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
                         const XmlRpc::XmlRpcValue& launch_id)
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

void Dart2Manual::leftSwitchDownOn()
{
  trigger_sender_->setPoint(0.7);
  ready_ = false;
  dart_fired_num_ = 0;
  a_left_sender_->setPoint(0.0);
  a_right_sender_->setPoint(0.0);
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
  if (!ready_)
  {
    trigger_sender_->setPoint(0.02);
    a_left_sender_->setPoint(downward_vel_);
    a_right_sender_->setPoint(downward_vel_);
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
    a_right_sender_->setPoint(upward_vel_);
    a_left_sender_->setPoint(upward_vel_);
  }
  if (ready_ && a_left_position_<=a_left_min_ && a_right_position_<=a_right_min_)
  {
    a_left_sender_->setPoint(0.0);
    a_right_sender_->setPoint(0.0);
  }
}

void Dart2Manual::leftSwitchUpOn()
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
  trigger_sender_->setPoint(0.02);
  ready_ = false;
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
  // TODO : 添加PC控制代码
}

void Dart2Manual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updatePc(dbus_data);
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
    //ROS_INFO("trigger_position: %lf", trigger_position_);
  }
  wheel_clockwise_event_.update(data->wheel == 1.0);
  wheel_anticlockwise_event_.update(data->wheel == -1.0);
  dbus_data_ = *data;
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