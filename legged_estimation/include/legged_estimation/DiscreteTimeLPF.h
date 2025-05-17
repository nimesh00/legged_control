//
// Created by sudaxia on 23-10-29.
//

#pragma once

#include "legged_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <shared_mutex>

namespace legged {
using namespace ocs2;

class DiscreteTimeLPF : public StateEstimateBase {
public:
    DiscreteTimeLPF(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

    vector_t update(const ros::Time& time, const ros::Duration& period) override;

    vector_t getEstimateForceInContinuous() {
        std::lock_guard<std::mutex> lock(footMutex_);
        return estimateFc_;
    }

    vector_t getEstimateForceInDiscrete() {
        std::lock_guard<std::mutex> lock(footMutex_);
        return estimateF_;
    }
    vector_t getProFromForce() {
        std::lock_guard<std::mutex> lock(footMutex_);
        return contactProFromF_;
    }
    vector_t getProFromHeight() {
        std::lock_guard<std::mutex> lock(footMutex_);
        return contactProFromH_;
    }

    void updateJointStates(const vector_t& jointPos, const vector_t& jointVel, const vector_t& jointEffort);

    void loadSetting(const std::string& taskFile, bool verbose);

private:
    std::mutex footMutex_;
    // cutoff frequency
    scalar_t lambda_;   // analog
    scalar_t gamma_;    // digital

    scalar_t beta_;
    Eigen::Matrix<scalar_t, 12, 18> j_;
    Eigen::Matrix<scalar_t, 12, 1> aTau_;
    Eigen::Matrix<scalar_t, 12, 18> Sl_;
    Eigen::Matrix<scalar_t, 12, 1> estimateF_;
    Eigen::Matrix<scalar_t, 12, 1> estimateFc_;
    Eigen::Matrix<scalar_t, 18, 1> g_;
    Eigen::Matrix<scalar_t, 12, 18> S_;
    Eigen::Matrix<scalar_t, 18, 1> tau1_;
    Eigen::Matrix<scalar_t, 18, 1> tau2_;

    Eigen::Matrix<scalar_t, 18, 1> lastTau2_;
    Eigen::Matrix<scalar_t, 18, 1> hatTau_;
    Eigen::Matrix<scalar_t, 18, 1> lastX_;
    Eigen::Matrix<scalar_t, 18, 18> lastM_;
    Eigen::Matrix<scalar_t,  4, 1> contactProFromF_;
    Eigen::Matrix<scalar_t,  4, 1> contactProFromH_;

    Eigen::Matrix<scalar_t, 18, 1> tau2c_;
    Eigen::Matrix<scalar_t, 18, 1> lastTau2c_;

};

}  // namespace legged
