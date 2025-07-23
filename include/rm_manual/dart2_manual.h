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
#include <rm_msgs/Dart.h>

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
    INIT = 0,
    PULLDOWN = 1,
    ROTATE_PLACE = 2,
    LOADING = 3,
    ROTATE_BACK = 4,
    LOADED = 5,
    ENGAGE = 6,
    PULLUP = 7,
    READY = 8,
    PUSH = 9
  };
  enum ClampType
  {
    ZERO = 0,
    LEFT = 1,
    MID = 2,
    RIGHT = 3,
    ALL = 4
  };
  enum AutoAimState
  {
    NONE,
    AIM,
    AIMED,
    ADJUST,
    ADJUSTED,
    FINISH
  };
  enum ClampMode
  {
    ClAMP,
    RELEASE
  };
  struct Dart
  {
    double outpost_offset_, base_offset_;
    double outpost_range_, base_range_;
  };

protected:
  void sendCommand(const ros::Time& time) override;
  void getList(const XmlRpc::XmlRpcValue& darts, const XmlRpc::XmlRpcValue& targets,
               const XmlRpc::XmlRpcValue& launch_id, const XmlRpc::XmlRpcValue& rotate_positions);
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
  void updateGripper();
  void move(rm_common::JointPointCommandSender* joint, double ch);
  void recordPosition(const rm_msgs::DbusData dbus_data);
  void readyLaunchDart(int dart_fired_num);
  bool triggerIsWorked() const;
  bool triggerIsHome() const;
  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) override;
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override;
  void dartClientCmdCallback(const rm_msgs::DartClientCmd::ConstPtr& data);
  void gameRobotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data) override;
  void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data) override;
  void updateAllowDartDoorOpenTimes();
  void wheelClockwise();
  void wheelAntiClockwise();
  void setGripperAction(int dart_fired_num);
  void changeGripperState(rm_common::JointPointCommandSender* gripper, bool& is_release);
  void changeAllGripperState();
  void operateGripper(const rm_msgs::DbusData::ConstPtr& dbus_data);
  void longCameraDataCallback(const rm_msgs::Dart::ConstPtr& data);
  void shortCameraDataCallback(const rm_msgs::Dart::ConstPtr& data);
  void updateLaunchMode();
  void updateAutoAimState();
  void init();
  void pullDown();
  void rotatePlace();
  void loading();
  void rotateBack();
  void loaded();
  void pullUp();
  void engage();
  void ready();
  void push();
  void aim();
  void adjust();
  void autoAim();

  rm_common::JointPointCommandSender* yaw_sender_;
  rm_common::JointPointCommandSender* trigger_sender_;
  rm_common::JointPointCommandSender* rotate_sender_;
  rm_common::JointPointCommandSender *clamp_left_sender_, *clamp_right_sender_, *clamp_mid_sender_;
  rm_common::JointPointCommandSender* belt_left_sender_, *belt_right_sender_;
  rm_common::JointPointCommandSender* range_sender_;
  rm_common::CalibrationQueue *shooter_calibration_, *gimbal_calibration_, *clamp_calibration_;

  double range_outpost_{}, range_base_{}, yaw_outpost_{}, yaw_base_{};
  double downward_vel_, upward_vel_;
  std::unordered_map<int, Dart> dart_list_{};
  std::unordered_map<std::string, std::vector<double>> target_position_{};
//  std::unordered_map<std::string, std::vector<double>> rotate_position_{};
  std::unordered_map<rm_common::JointPointCommandSender*, double> clamp_position_{};
  std::vector<double> rotate_place_position_{}, rotate_back_position_{};

  double scale_{ 0.04 }, scale_micro_{ 0.01 };
  bool if_stop_{ true }, has_stopped{ false }, is_reach_{ false }, is_calibrate_{ false }, trigger_has_work_{ false },
      ready_{ false };
  int has_fired_num_{};
  bool clamp_calibrate_{ false };
  bool left_clamp_is_release_{ false },mid_clamp_is_release_{false},right_clamp_is_release_{false};
  bool confirm_place_{ false }, confirm_back_{ false };
  ros::Time last_time_{};
  uint8_t launch_mode_{ 0 }, last_launch_mode_{ 6 };
  uint8_t auto_aim_state_{ 0 }, last_auto_aim_state_{};
  uint8_t gripper_state_{ 0 }, last_gripper_state_{ 0 };

  rm_msgs::DbusData dbus_data_;
  uint8_t robot_id_, game_progress_{}, dart_launch_opening_status_{ 3 };
  uint16_t remain_time_{ 420 };

  int dart_fired_num_ = 0;
  double trigger_home_command_{}, trigger_work_command_{};
  double trigger_confirm_home_{}, trigger_confirm_work_{};
  double belt_left_position_{}, belt_right_position_{}, trigger_position_{};
  double belt_left_max_{}, belt_right_max_{}, belt_left_min_{}, belt_right_min_{}, belt_left_place_{}, belt_right_place_{},
      belt_left_placed_{}, belt_right_placed_{};
  double clamp_left_position_, clamp_mid_position_, clamp_right_position_, release_position_, clamp_finish_position_;
  double range_velocity_ = 0., yaw_velocity_ = 0., rotate_velocity_ = 0.;

  double short_camera_x_set_point_, long_camera_x_set_point_, long_camera_y_set_point_;
  double camera_x_offset_, camera_y_offset_;
  double long_camera_p_x_, short_camera_p_x_, long_camera_p_y_;
  bool is_long_camera_found_{ false }, is_short_camera_found_{ false }, is_long_camera_aim_{};
  double long_camera_x_{}, long_camera_y_{}, short_camera_x_{}, short_camera_y_{}, last_camera_x{};
  double long_camera_x_threshold_, retarget_threshold_;
  bool camera_central_{}, is_adjust_{};
  bool camera_is_online_{};
  bool use_auto_aim_{}, start_aim_{};
  double long_camera_x_before_push_{}, long_camera_x_after_push_{};

  InputEvent wheel_clockwise_event_, wheel_anticlockwise_event_;
  ros::Time last_engage_time_{};
  ros::Time last_rotate_time_{};
  ros::Time last_loading_time_{};
  ros::Time last_push_time_{};
  ros::Time last_ready_time_{};
  ros::Time last_init_time_{};
  ros::Time last_get_camera_data_time_{};
  ros::Time start_aim_time_{};
  ros::Subscriber dart_client_cmd_sub_;
  ros::Subscriber long_camera_data_sub_;
  ros::Subscriber short_camera_data_sub_;
  InputEvent dart_client_cmd_event_;
  int outpost_hp_;
  int allow_dart_door_open_times_ = 0, last_dart_door_status_ = 1;
  bool triggered_30s_{ false }, triggered_4min_{ false };
  int auto_state_ = BASE, manual_state_ = BASE, move_state_ = NORMAL, launch_state_ = INIT;
};

}  // namespace rm_manual
