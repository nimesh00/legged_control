
//
// Created by qiayuan on 1/24/22.
//

#pragma once

#include <legged_hw/LeggedHW.h>

#include "dds/dds_publisher.hpp"
#include "dds/dds_subscriber.hpp"

#include "LowCmd.hpp"
#include "LowState.hpp"
#include "Go2pyLowCmd.hpp"

#include "unitree_legged_sdk_3_8_0/safety.h"

using namespace org::eclipse::cyclonedds;
using namespace unitree_go::msg::dds_;
using namespace go2py_messages::msg::dds_;

namespace legged {
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct UnitreeMotorData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct UnitreeImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class UnitreeDDS : public LeggedHW {
 public:
  UnitreeDDS() = default;
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Communicate with hardware. Get data, status of robot.
   *
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;
  void copyLowStateFromDDS();

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;
  void writeLowCmdToDDS();

  void updateJoystick(const ros::Time& time);

  void updateContact(const ros::Time& time);

 private:
  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle& nh);

  std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_;
  UNITREE_LEGGED_SDK::LowState lowState_{};
  UNITREE_LEGGED_SDK::LowCmd lowCmd_{};

  UnitreeMotorData jointData_[12]{};  // NOLINT(modernize-avoid-c-arrays)
  UnitreeImuData imuData_{};
  bool contactState_[4]{};  // NOLINT(modernize-avoid-c-arrays)

  int powerLimit_{};
  int contactThreshold_{};

  ros::Publisher joyPublisher_;
  ros::Publisher contactPublisher_;
  ros::Time lastJoyPub_, lastContactPub_;

  std::unique_ptr<DDSPublisher<LowCmd_>> low_cmd_dds_pub_ = NULL;
  std::unique_ptr<DDSPublisher<Go2pyLowCmd_>> go2py_low_cmd_dds_pub_ = NULL;
  std::unique_ptr<DDSSubscriber<LowState_>> low_state_dds_sub_ = NULL;

  LowState_ lowState_dds;
  LowCmd_ lowCmd_dds;
  Go2pyLowCmd_ go2pyLowCmd_dds;
};

}  // namespace legged
