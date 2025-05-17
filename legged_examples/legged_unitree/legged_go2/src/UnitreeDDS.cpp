#include "legged_go2/UnitreeDDS.h"

#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>

#include "unitree_legged_sdk_3_8_0/joystick.h"

namespace legged {
bool UnitreeDDS::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
    if (!LeggedHW::init(root_nh, robot_hw_nh)) {
        return false;
    }

    robot_hw_nh.getParam("power_limit", powerLimit_);

    setupJoints();
    setupImu();
    setupContactSensor(robot_hw_nh);

    low_cmd_dds_pub_.reset(new DDSPublisher<LowCmd_>("rt/lowcmd"));
    go2py_low_cmd_dds_pub_.reset(
        new DDSPublisher<Go2pyLowCmd_>("rt/go2py/low_cmd"));
    low_state_dds_sub_.reset(new DDSSubscriber<LowState_>("rt/lowstate"));
    sensor_dds_sub_.reset(
        new DDSSubscriber<SensorData_>("rt/go2/sim/sensor_data"));
    joint_dds_pub_.reset(
        new DDSPublisher<JointData_>("rt/go2/sim/joint_command"));
    gt_dds_sub_.reset(new DDSSubscriber<QuadLog_>("rt/go2/sim/gt_data"));

    std::string robot_type;
    root_nh.getParam("robot_type", robot_type);

    if (robot_type == "go2") {
        // Force go2 safety routine on go2 commands for basic sanitization
        // (hopefully)
        safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(
            UNITREE_LEGGED_SDK::LeggedType::Go1);
    }

    else {
        ROS_FATAL("Unknown robot type: %s", robot_type.c_str());
        return false;
    }

    joyPublisher_ = root_nh.advertise<sensor_msgs::Joy>("/joy", 10);
    contactPublisher_ = root_nh.advertise<std_msgs::Int16MultiArray>(
        std::string("/contact"), 10);
    return true;
}

void UnitreeDDS::read(const ros::Time& time, const ros::Duration& /*period*/) {
    sensorData_dds = sensor_dds_sub_->getLatestMessage();
    gtData_dds = gt_dds_sub_->getLatestMessage();

    for (int i = 0; i < 12; ++i) {
        jointData_[remap_index[i]].pos_ =
            sensorData_dds.q()[i];  // Access the array directly
        jointData_[remap_index[i]].vel_ =
            sensorData_dds.dq()[i];  // Access the array directly
        jointData_[remap_index[i]].tau_ =
            sensorData_dds.tau_est()[i];  // Access the array directly
    }

    imuData_.ori_[0] = sensorData_dds.quat()[0];
    imuData_.ori_[1] = sensorData_dds.quat()[1];
    imuData_.ori_[2] = sensorData_dds.quat()[2];
    imuData_.ori_[3] = sensorData_dds.quat()[3];
    imuData_.angularVel_[0] = sensorData_dds.gyro()[0];
    imuData_.angularVel_[1] = sensorData_dds.gyro()[1];
    imuData_.angularVel_[2] = sensorData_dds.gyro()[2];
    imuData_.linearAcc_[1] = sensorData_dds.accel()[1];
    imuData_.linearAcc_[0] = sensorData_dds.accel()[0];
    imuData_.linearAcc_[2] = sensorData_dds.accel()[2];

    for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
        contactState_[i] = gtData_dds.contact_force()[i] > contactThreshold_;
    }

    // Set feedforward and velocity cmd to zero to avoid for safety when not
    // controller setCommand
    std::vector<std::string> names = hybridJointInterface_.getNames();
    for (const auto& name : names) {
        HybridJointHandle handle = hybridJointInterface_.getHandle(name);
        handle.setFeedforward(0.);
        handle.setVelocityDesired(0.);
        handle.setKd(3.);
    }

    updateJoystick(time);
    updateContact(time);
}

void UnitreeDDS::writejointdataToDDS() {
    for (int i = 0; i < 12; ++i) {
        jointData_dds.q()[i] = lowCmd_.motorCmd[i].q;
        jointData_dds.dq()[i] = lowCmd_.motorCmd[i].dq;
        jointData_dds.tau()[i] = lowCmd_.motorCmd[i].tau;
        jointData_dds.kp()[i] = lowCmd_.motorCmd[i].Kp;
        jointData_dds.kd()[i] = lowCmd_.motorCmd[i].Kd;
    }
}

void UnitreeDDS::write(const ros::Time& /*time*/,
                       const ros::Duration& /*period*/) {
    for (int i = 0; i < 12; ++i) {
        // lowCmd_.motorCmd[i].q = static_cast<float>(jointData_[i].posDes_);
        // lowCmd_.motorCmd[i].dq = static_cast<float>(jointData_[i].velDes_);
        // lowCmd_.motorCmd[i].Kp = static_cast<float>(jointData_[i].kp_);
        // lowCmd_.motorCmd[i].Kd = static_cast<float>(jointData_[i].kd_);
        // lowCmd_.motorCmd[i].tau = static_cast<float>(jointData_[i].ff_);
        jointData_dds.q()[remap_index[i]] =
            static_cast<float>(jointData_[i].posDes_);
        jointData_dds.dq()[remap_index[i]] =
            static_cast<float>(jointData_[i].velDes_);
        jointData_dds.kp()[remap_index[i]] =
            static_cast<float>(jointData_[i].kp_);
        jointData_dds.kd()[remap_index[i]] =
            static_cast<float>(jointData_[i].kd_);
        jointData_dds.tau()[remap_index[i]] =
            static_cast<float>(jointData_[i].ff_);
    }
    // safety_->PositionLimit(lowCmd_);
    // safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);

    // writejointdataToDDS();

    // low_cmd_dds_pub_ -> publish(lowCmd_dds);
    joint_dds_pub_->publish(jointData_dds);
}

bool UnitreeDDS::setupJoints() {
    for (const auto& joint : urdfModel_->joints_) {
        int leg_index = 0;
        int joint_index = 0;
        if (joint.first.find("RF") != std::string::npos) {
            leg_index = UNITREE_LEGGED_SDK::FR_;
        } else if (joint.first.find("LF") != std::string::npos) {
            leg_index = UNITREE_LEGGED_SDK::FL_;
        } else if (joint.first.find("RH") != std::string::npos) {
            leg_index = UNITREE_LEGGED_SDK::RR_;
        } else if (joint.first.find("LH") != std::string::npos) {
            leg_index = UNITREE_LEGGED_SDK::RL_;
        } else {
            continue;
        }

        if (joint.first.find("HAA") != std::string::npos) {
            joint_index = 0;
        } else if (joint.first.find("HFE") != std::string::npos) {
            joint_index = 1;
        } else if (joint.first.find("KFE") != std::string::npos) {
            joint_index = 2;
        } else {
            continue;
        }

        int index = leg_index * 3 + joint_index;
        hardware_interface::JointStateHandle state_handle(
            joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
            &jointData_[index].tau_);
        jointStateInterface_.registerHandle(state_handle);
        hybridJointInterface_.registerHandle(HybridJointHandle(
            state_handle, &jointData_[index].posDes_,
            &jointData_[index].velDes_, &jointData_[index].kp_,
            &jointData_[index].kd_, &jointData_[index].ff_));
    }
    return true;
}

bool UnitreeDDS::setupImu() {
    imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
        "base_imu", "base_imu", imuData_.ori_, imuData_.oriCov_,
        imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
        imuData_.linearAccCov_));
    imuData_.oriCov_[0] = 0.0012;
    imuData_.oriCov_[4] = 0.0012;
    imuData_.oriCov_[8] = 0.0012;

    imuData_.angularVelCov_[0] = 0.0004;
    imuData_.angularVelCov_[4] = 0.0004;
    imuData_.angularVelCov_[8] = 0.0004;

    return true;
}

bool UnitreeDDS::setupContactSensor(ros::NodeHandle& nh) {
    nh.getParam("contact_threshold", contactThreshold_);
    for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
        contactSensorInterface_.registerHandle(
            ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
    }
    return true;
}

void UnitreeDDS::updateJoystick(const ros::Time& time) {
    if ((time - lastJoyPub_).toSec() < 1 / 50.) {
        return;
    }
    lastJoyPub_ = time;
    xRockerBtnDataStruct keyData;
    memcpy(&keyData, &lowState_.wirelessRemote[0], 40);
    sensor_msgs::Joy joyMsg;  // Pack as same as Logitech F710
    joyMsg.axes.push_back(-keyData.lx);
    joyMsg.axes.push_back(keyData.ly);
    joyMsg.axes.push_back(-keyData.rx);
    joyMsg.axes.push_back(keyData.ry);
    joyMsg.buttons.push_back(keyData.btn.components.X);
    joyMsg.buttons.push_back(keyData.btn.components.A);
    joyMsg.buttons.push_back(keyData.btn.components.B);
    joyMsg.buttons.push_back(keyData.btn.components.Y);
    joyMsg.buttons.push_back(keyData.btn.components.L1);
    joyMsg.buttons.push_back(keyData.btn.components.R1);
    joyMsg.buttons.push_back(keyData.btn.components.L2);
    joyMsg.buttons.push_back(keyData.btn.components.R2);
    joyMsg.buttons.push_back(keyData.btn.components.select);
    joyMsg.buttons.push_back(keyData.btn.components.start);
    joyPublisher_.publish(joyMsg);
}

void UnitreeDDS::updateContact(const ros::Time& time) {
    if ((time - lastContactPub_).toSec() < 1 / 50.) {
        return;
    }
    lastContactPub_ = time;

    std_msgs::Int16MultiArray contactMsg;
    for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
        contactMsg.data.push_back(gtData_dds.contact_force()[i]);
    }
    contactPublisher_.publish(contactMsg);
}

}  // namespace legged
// void UnitreeDDS::copyLowStateFromDDS() {
//     for (int i = 0; i < 12; ++i) {
//         lowState_.motorState[i].q = lowState_dds.motor_state()[i].q();
//         lowState_.motorState[i].dq = lowState_dds.motor_state()[i].dq();
//         lowState_.motorState[i].tauEst =
//         lowState_dds.motor_state()[i].tau_est();
//     }

//     for (int i = 0; i < 4; ++i) {
//         lowState_.footForce[i] = lowState_dds.foot_force()[i];
//     }

//     for (int i = 0; i < 4; ++i) {
//         lowState_.imu.quaternion[i] =
//         lowState_dds.imu_state().quaternion()[i];
//     }

//     for (int i = 0; i < 3; ++i) {
//         lowState_.imu.gyroscope[i] = lowState_dds.imu_state().gyroscope()[i];
//         lowState_.imu.accelerometer[i] =
//         lowState_dds.imu_state().accelerometer()[i];
//     }

//     for (int i = 0; i < 40; ++i) {
//         lowState_.wirelessRemote[i] = lowState_dds.wireless_remote()[i];
//     }
// }

// void UnitreeDDS::read(const ros::Time& time, const ros::Duration& /*period*/)
// {
//   lowState_dds = low_state_dds_sub_ -> getLatestMessage();

//   copyLowStateFromDDS();

//   for (int i = 0; i < 12; ++i) {
//     jointData_[i].pos_ = lowState_.motorState[i].q;
//     jointData_[i].vel_ = lowState_.motorState[i].dq;
//     jointData_[i].tau_ = lowState_.motorState[i].tauEst;
//   }

//   imuData_.ori_[0] = lowState_.imu.quaternion[1];
//   imuData_.ori_[1] = lowState_.imu.quaternion[2];
//   imuData_.ori_[2] = lowState_.imu.quaternion[3];
//   imuData_.ori_[3] = lowState_.imu.quaternion[0];
//   imuData_.angularVel_[0] = lowState_.imu.gyroscope[0];
//   imuData_.angularVel_[1] = lowState_.imu.gyroscope[1];
//   imuData_.angularVel_[2] = lowState_.imu.gyroscope[2];
//   imuData_.linearAcc_[0] = lowState_.imu.accelerometer[0];
//   imuData_.linearAcc_[1] = lowState_.imu.accelerometer[1];
//   imuData_.linearAcc_[2] = lowState_.imu.accelerometer[2];

//   for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
//     contactState_[i] = lowState_.footForce[i] > contactThreshold_;
//   }

//   // Set feedforward and velocity cmd to zero to avoid for safety when not
//   controller setCommand std::vector<std::string> names =
//   hybridJointInterface_.getNames(); for (const auto& name : names) {
//     HybridJointHandle handle = hybridJointInterface_.getHandle(name);
//     handle.setFeedforward(0.);
//     handle.setVelocityDesired(0.);
//     handle.setKd(3.);
//   }

//   updateJoystick(time);
//   updateContact(time);
// }

// #ifdef UNITREE_SDK_3_3_1
//   if (robot_type == "a1") {
//     safety_ =
//     std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::A1);
//   } else if (robot_type == "aliengo") {
//     safety_ =
//     std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Aliengo);
//   }
// #elif UNITREE_SDK_3_8_0
