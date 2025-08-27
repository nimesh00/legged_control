
//
// Created by qiayuan on 1/24/22.
//

#include "legged_xterra_hw/xTerraHW.h"

#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>

#include <eigen3/Eigen/Dense>

namespace legged {
bool xTerraHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
    if (!LeggedHW::init(root_nh, robot_hw_nh)) {
        return false;
    }

    robot_hw_nh.getParam("power_limit", powerLimit_);

    setupJoints();
    setupImu();
    setupContactSensor(robot_hw_nh);

    std::string robot_type;
    root_nh.getParam("robot_type", robot_type);

    sensor_data_sub_ = std::make_shared<DDSSubscriber<SensorData_>>(
        "rt/m2_metal/hw/sensor_data",
        std::bind(&xTerraHW::sensorDataCb, this, std::placeholders::_1), 0);
    sensor_data_ = SensorData_();

    // gt_data_sub_ = std::make_shared<DDSSubscriber<QuadLog_>>(
    //     "rt/m2_metal/sim/gt_data",
    //     std::bind(&xTerraHW::gtDataCb, this, std::placeholders::_1), 0);
    // gt_data_ = QuadLog_();

    joy_data_sub_ = std::make_shared<DDSSubscriber<ByteArray_>>(
        "rt/keyboard_bytearray/joystick_data",
        std::bind(&xTerraHW::joyDataCb, this, std::placeholders::_1), 0);
    joy_data_ = ByteArray_();
    joy_data_.data().resize(sizeof(XboxJoystickState));

    joint_cmd_pub_ = std::make_shared<DDSPublisher<JointData_>>(
        "rt/m2_metal/hw/joint_command");
    joint_cmd_ = JointData_();

    joyPublisher_ = root_nh.advertise<sensor_msgs::Joy>("/joy", 10);
    // contactPublisher_ = root_nh.advertise<std_msgs::Int16MultiArray>(
    //     std::string("/contact"), 10);
    return true;
}

void xTerraHW::sensorDataCb(const SensorData_& msg) { sensor_data_ = msg; }
// void xTerraHW::gtDataCb(const QuadLog_& msg) { gt_data_ = msg; }
void xTerraHW::joyDataCb(const ByteArray_& msg) { joy_data_ = msg; }

using mat3x3 = Eigen::Matrix<double, 3, 3>;
using vec3 = Eigen::Matrix<double, 3, 1>;
using vec4 = Eigen::Matrix<double, 4, 1>;

inline mat3x3 SkewSymm(const vec3& v) {
    mat3x3 vx;
    vx << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;

    return vx;
}

inline mat3x3 QuatToRot(const vec4& q) {
    // pinocchio convention of {x, y, z, w}
    double q0 = q(3);
    vec3 qv = q.block<3, 1>(0, 0);
    mat3x3 Rot = (2 * q0 * q0 - 1) * mat3x3::Identity() +
                 2 * q0 * SkewSymm(qv) + 2 * qv * qv.transpose();

    return Rot;
}

void xTerraHW::read(const ros::Time& time, const ros::Duration& /*period*/) {
    for (int i = 0; i < 12; ++i) {
        jointData_[remap_index[i]].pos_ = sensor_data_.q()[i];
        jointData_[remap_index[i]].vel_ = sensor_data_.dq()[i];
        jointData_[remap_index[i]].tau_ = sensor_data_.tau_est()[i];
    }

    imuData_.ori_[0] = sensor_data_.quat()[0];
    imuData_.ori_[1] = sensor_data_.quat()[1];
    imuData_.ori_[2] = sensor_data_.quat()[2];
    imuData_.ori_[3] = sensor_data_.quat()[3];
    vec4 quat = vec4::Zero();
    for (int i = 0; i < 4; ++i) quat(i) = sensor_data_.quat()[i];
    mat3x3 Rot = QuatToRot(quat);
    imuData_.angularVel_[0] = sensor_data_.gyro()[0];
    imuData_.angularVel_[1] = sensor_data_.gyro()[1];
    imuData_.angularVel_[2] = sensor_data_.gyro()[2];
    vec3 acc_base = vec3::Zero();
    for (int i = 0; i < 3; ++i) acc_base(i) = sensor_data_.accel()[i];
    acc_base += Rot.transpose() * vec3(0, 0, 9.81);
    imuData_.linearAcc_[0] = acc_base(0);
    imuData_.linearAcc_[1] = acc_base(1);
    imuData_.linearAcc_[2] = acc_base(2);
    // imuData_.linearAcc_[0] = sensor_data_.accel()[0];
    // imuData_.linearAcc_[1] = sensor_data_.accel()[1];
    // imuData_.linearAcc_[2] = sensor_data_.accel()[2];

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

void xTerraHW::write(const ros::Time& /*time*/,
                     const ros::Duration& /*period*/) {
    if (e_stop_requested_) {
        // Emergency stop requested, enter damping mode
        for (int i = 0; i < 12; ++i) {
            joint_cmd_.q()[remap_index[i]] =
                static_cast<float>(jointData_[i].posDes_);
            joint_cmd_.dq()[remap_index[i]] = 0;
            joint_cmd_.kp()[remap_index[i]] = 0;
            joint_cmd_.kd()[remap_index[i]] = 3;
            joint_cmd_.tau()[remap_index[i]] = 0;
        }
    } else {
        for (int i = 0; i < 12; ++i) {
            joint_cmd_.q()[remap_index[i]] =
                static_cast<float>(jointData_[i].posDes_);
            joint_cmd_.dq()[remap_index[i]] =
                static_cast<float>(jointData_[i].velDes_);
            joint_cmd_.kp()[remap_index[i]] =
                static_cast<float>(jointData_[i].kp_);
            joint_cmd_.kd()[remap_index[i]] =
                static_cast<float>(jointData_[i].kd_);
            joint_cmd_.tau()[remap_index[i]] =
                static_cast<float>(jointData_[i].ff_);
        }
    }
    joint_cmd_pub_->publish(joint_cmd_);
}

bool xTerraHW::setupJoints() {
    for (const auto& joint : urdfModel_->joints_) {
        int leg_index = 0;
        int joint_index = 0;
        if (joint.first.find("RF") != std::string::npos) {
            leg_index = XTERRA_LEGGED_SDK::LEG_INDEX::FR_;
        } else if (joint.first.find("LF") != std::string::npos) {
            leg_index = XTERRA_LEGGED_SDK::LEG_INDEX::FL_;
        } else if (joint.first.find("RH") != std::string::npos) {
            leg_index = XTERRA_LEGGED_SDK::LEG_INDEX::RR_;
        } else if (joint.first.find("LH") != std::string::npos) {
            leg_index = XTERRA_LEGGED_SDK::LEG_INDEX::RL_;
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

bool xTerraHW::setupImu() {
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

bool xTerraHW::setupContactSensor(ros::NodeHandle& nh) {
    nh.getParam("contact_threshold", contactThreshold_);
    for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
        contactSensorInterface_.registerHandle(
            ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
    }
    return true;
}

void xTerraHW::updateJoystick(const ros::Time& time) {
    if ((time - lastJoyPub_).toSec() < 1 / 50.) {
        return;
    }
    lastJoyPub_ = time;
    XboxJoystickState state;
    memcpy(state.buffer, &joy_data_.data()[0], sizeof(XboxJoystickState));
    sensor_msgs::Joy joyMsg;
    joyMsg.axes.push_back(-state.fields.lx);
    joyMsg.axes.push_back(state.fields.ly);
    joyMsg.axes.push_back(-state.fields.rx);
    joyMsg.axes.push_back(state.fields.ry);
    joyMsg.buttons.push_back(IS_BUTTON_PRESSED(state, XBOX_BTN_X));
    joyMsg.buttons.push_back(IS_BUTTON_PRESSED(state, XBOX_BTN_A));
    joyMsg.buttons.push_back(IS_BUTTON_PRESSED(state, XBOX_BTN_B));
    joyMsg.buttons.push_back(IS_BUTTON_PRESSED(state, XBOX_BTN_Y));
    joyMsg.buttons.push_back(IS_BUTTON_PRESSED(state, XBOX_BTN_LB));
    joyMsg.buttons.push_back(IS_BUTTON_PRESSED(state, XBOX_BTN_RB));
    joyMsg.buttons.push_back(IS_BUTTON_PRESSED(state, XBOX_BTN_START));
    joyMsg.buttons.push_back(IS_BUTTON_PRESSED(state, XBOX_BTN_BACK));
    joyPublisher_.publish(joyMsg);

    if (IS_BUTTON_PRESSED(state, XBOX_BTN_LB)) {
        std::cout << "E-Stop Requested!!\n";
        e_stop_requested_ = true;
    }
    if (IS_BUTTON_PRESSED(state, XBOX_BTN_RB)) {
        std::cout << "Disabling E-Stop\n";
        e_stop_requested_ = false;
    }
}

void xTerraHW::updateContact(const ros::Time& time) {
    if ((time - lastContactPub_).toSec() < 1 / 50.) {
        return;
    }
    lastContactPub_ = time;

    // std_msgs::Int16MultiArray contactMsg;
    // for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    //     contactMsg.data.push_back(gt_data_.contact_force()[i]);
    // }
    // contactPublisher_.publish(contactMsg);
}

}  // namespace legged
