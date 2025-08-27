
//
// Created by qiayuan on 1/24/22.
//

#pragma once

#include <legged_hw/LeggedHW.h>

#include "ByteArray.hpp"
#include "JointData.hpp"
#include "QuadLog.hpp"
#include "SensorData.hpp"
#include "dds/dds_publisher.hpp"
#include "dds/dds_subscriber.hpp"
#include "xterra_legged_sdk.h"

namespace legged {
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT",
                                                       "RH_FOOT", "LH_FOOT"};

struct MotorData {
    double pos_, vel_, tau_;                 // state
    double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct ImuData {
    double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
    double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
    double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
    double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
    double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
    double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

using namespace xterra::msg::dds_;
using namespace XTERRA_LEGGED_SDK;

class xTerraHW : public LeggedHW {
   public:
    xTerraHW() = default;
    /** \brief Get necessary params from param server. Init hardware_interface.
     *
     * Get params from param server and check whether these params are set. Load
     * urdf of robot. Set up transmission and joint limit. Get configuration of
     * can bus and create data pointer which point to data received from Can
     * bus.
     *
     * @param root_nh Root node-handle of a ROS node.
     * @param robot_hw_nh Node-handle for robot hardware.
     * @return True when init successful, False when failed.
     */
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
    /** \brief Communicate with hardware. Get data, status of robot.
     *
     * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
     *
     * @param time Current time
     * @param period Current time - last time
     */
    void read(const ros::Time& time, const ros::Duration& period) override;

    /** \brief Comunicate with hardware. Publish command to robot.
     *
     * Propagate joint state to actuator state for the stored
     * transmission. Limit cmd_effort into suitable value. Call @ref
     * UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator current state.
     *
     * @param time Current time
     * @param period Current time - last time
     */
    void write(const ros::Time& time, const ros::Duration& period) override;

    void updateJoystick(const ros::Time& time);

    void updateContact(const ros::Time& time);

   private:
    bool setupJoints();

    bool setupImu();

    bool setupContactSensor(ros::NodeHandle& nh);

    void sensorDataCb(const SensorData_& msg);
    void gtDataCb(const QuadLog_& msg);
    void joyDataCb(const ByteArray_& msg);

    SensorData_ sensor_data_;
    JointData_ joint_cmd_;
    QuadLog_ gt_data_;
    ByteArray_ joy_data_;
    XboxJoystickState joystick_state;

    std::shared_ptr<DDSSubscriber<SensorData_>> sensor_data_sub_;
    std::shared_ptr<DDSSubscriber<QuadLog_>> gt_data_sub_;
    std::shared_ptr<DDSSubscriber<ByteArray_>> joy_data_sub_;
    std::shared_ptr<DDSPublisher<JointData_>> joint_cmd_pub_;

    MotorData jointData_[12]{};
    ImuData imuData_{};
    bool contactState_[4]{};
    int remap_index[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};

    bool e_stop_requested_ = false;

    int powerLimit_{};
    int contactThreshold_{};

    ros::Publisher joyPublisher_;
    ros::Publisher contactPublisher_;
    ros::Time lastJoyPub_, lastContactPub_;
};

}  // namespace legged
