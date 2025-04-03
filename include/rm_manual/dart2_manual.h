//
// Created by chen_gr on 2025/3/9.
//

#pragma once

#include "rm_manual/manual_base.h"
#include <rm_common/decision/calibration_queue.h>
#include <rm_msgs/DartClientCmd.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/GameStatus.h>
#include <unordered_map>

namespace rm_manual
{
class Dart2Manual : public ManualBase
{
public:
  Dart2Manual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  enum AimMode
  {
    OUTPOST,
    BASE
  };
  enum MoveMode
  {
    NORMAL,
    MICRO,
    MOVING,
    STOP
  };
  enum LaunchMode
  {
    NONE,
    AIMED
  };
  struct Dart
  {
    double outpost_offset_, base_offset_;
    double outpost_b_, base_b_;
    double trigger_position_;
  };
protected:
  void sendCommand(const ros::Time& time) override;
  void getList(const XmlRpc::XmlRpcValue& darts, const XmlRpc::XmlRpcValue& targets,
               const XmlRpc::XmlRpcValue& launch_id, const XmlRpc::XmlRpcValue& trigger_position);
  void run() override;
  void checkReferee() override;
  void remoteControlTurnOn() override;
  void leftSwitchMidOn();
  void leftSwitchDownOn();
  void leftSwitchUpOn();
  void rightSwitchDownOn() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void move(rm_common::JointPointCommandSender* joint, double ch);
  void recordPosition(const rm_msgs::DbusData dbus_data);
  void waitAfterLaunch(const double time);
  void launchTwoDart();
  void getDartFiredNum();
  bool triggerIsWorked() const;
  bool triggerIsHome() const;
  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) override;
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override;
  void dartClientCmdCallback(const rm_msgs::DartClientCmd::ConstPtr& data);
  void gameRobotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data) override;
  void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data) override;
  void wheelClockwise();
  void wheelAntiClockwise();

  rm_common::JointPointCommandSender *yaw_sender_;
  rm_common::JointPointCommandSender *trigger_sender_;
  rm_common::JointPointCommandSender *a_left_sender_, *a_right_sender_; //TODO : 修改名字
  rm_common::JointPointCommandSender *b_sender_;  // TODO : 修改名字
  rm_common::CalibrationQueue *shooter_calibration_, *gimbal_calibration_;

  double b_outpost_{}, b_base_{}, yaw_outpost_{}, yaw_base_{};
  double downward_vel_, upward_vel_;
  std::unordered_map<int, Dart> dart_list_{};
  std::unordered_map<std::string, std::vector<double>> target_position_{};
  double scale_{ 0.04 }, scale_micro_{ 0.01 };
  bool if_stop_{ true }, has_stopped{ false },is_reach_{false},is_calibrate_{false},trigger_has_work_{false},ready_{false};

  rm_msgs::DbusData dbus_data_;
  uint8_t robot_id_, game_progress_, dart_launch_opening_status_;

  int dart_fired_num_ = 0, initial_dart_fired_num_ = 0;
  double trigger_home_{},trigger_work_{};
  double trigger_confirm_home_{},trigger_confirm_work_{};
  double a_left_position_{}, a_right_position_{},trigger_position_{};
  double a_left_max_{}, a_right_max_{},a_left_min_{}, a_right_min_{};

  InputEvent wheel_clockwise_event_, wheel_anticlockwise_event_;
  ros::Time stop_time_;
  ros::Subscriber dart_client_cmd_sub_;
  InputEvent dart_client_cmd_event_;
  int outpost_hp_;
  int dart_door_open_times_ = 0, last_dart_door_status_ = 1;
  int auto_state_ = OUTPOST, manual_state_ = OUTPOST, move_state_ = NORMAL, launch_state_ = NONE;
};

}