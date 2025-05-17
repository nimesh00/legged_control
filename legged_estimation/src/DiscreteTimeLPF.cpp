//
// Created by sudaxia on 23-10-30.
//

#include <pinocchio/fwd.hpp>
#include "legged_estimation/DiscreteTimeLPF.h"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged {

DiscreteTimeLPF::DiscreteTimeLPF(ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo info,
                                 const ocs2::PinocchioEndEffectorKinematics &eeKinematics)
        : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics) {
    lambda_ = 15;
    gamma_ = exp(-lambda_*0.002);
    beta_ = (1 - gamma_)/(gamma_ * 0.002);
    g_.setConstant(9.8);
    S_.setZero();
    S_.block(0, 6, 12, 12) = Eigen::MatrixXd::Identity(12, 12);
    tau1_.setZero();
    tau2_.setZero();

    tau2c_.setZero();
    lastTau2_.setZero();

    lastTau2_.setZero();
    lastX_.setZero();
    hatTau_.setZero();
    j_.setZero();
    aTau_.setZero();
    Sl_.setOnes();
    Sl_.block(0, 6, 12, 12) = Eigen::MatrixXd::Identity(12, 12);
    contactProFromF_.setZero();
    contactProFromH_.setZero();
    estimateF_.setZero();
    estimateFc_.setZero();
    eeKinematics_->setPinocchioInterface(pinocchioInterface_);
    pinocchio::crba(pinocchioInterface_.getModel(), pinocchioInterface_.getData(), Eigen::Matrix<scalar_t, 18, 1>::Zero());
    lastM_ = pinocchioInterface_.getData().M;
}

void DiscreteTimeLPF::updateJointStates(const vector_t& jointPos, const vector_t& jointVel, const vector_t& jointEffort) {
    rbdState_.segment(6, info_.actuatedDofNum) = jointPos;
    rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = jointVel;
    aTau_ = jointEffort;
}

vector_t DiscreteTimeLPF::update(const ros::Time &time, const ros::Duration &period) {
    std::lock_guard<std::mutex> lock(footMutex_);
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    size_t actuatedDofNum = info_.actuatedDofNum;

    vector_t qPino(info_.generalizedCoordinatesNum);
    vector_t vPino(info_.generalizedCoordinatesNum);

    qPino.setZero();
    qPino.segment<3>(3) = rbdState_.head<3>();  // Only set orientation, let position in origin.

    // 初始化随机数生成器 -- 弄一个噪声给电机
    std::random_device rd;
    std::mt19937 gen(rd()); // 以随机设备作为种子
    // 定义高斯分布，均值为0，标准差为0.01
    std::normal_distribution<> d(0, 0.01);
    Eigen::Matrix<double, 12, 1> noise;
    noise.setOnes();
    qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum) + noise * d(gen);

    vPino.setZero();
    vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qPino.segment<3>(3),
            rbdState_.segment<3>(info_.generalizedCoordinatesNum));  // Only set angular velocity, let linear velocity be zero
    vPino.tail(actuatedDofNum) = rbdState_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);

    pinocchio::crba(model, data, qPino);
    pinocchio::nonLinearEffects(model, data, qPino, vPino);
    pinocchio::computeCoriolisMatrix(model, data, qPino, vPino);
    pinocchio::forwardKinematics(model, data, qPino, vPino);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);

    const auto eePos = eeKinematics_->getPosition(vector_t());
    for(int i = 0; i < contactProFromH_.size(); i++) {
        contactProFromH_[i] = 0.5 * (1 + std::erf((0.02-(eePos[i][2]+0.29))/sqrt(2*0.0005)));
    }

    // compute the Jacobin of the feet
    j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, info_.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
        j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
    }

    tau1_ = beta_ * data.M * vPino;
    vector_t x = tau1_ + S_.transpose() * aTau_ - data.nle + (data.M - lastM_) * 500 * vPino;
//     vector_t x = tau1_ + S_.transpose() * aTau_ + data.C.transpose() * vPino - g_;
    tau2_ = (gamma_ - 1) * x + (gamma_) * lastTau2_;

    /* Discrete Time equation */
    hatTau_ = tau1_ + tau2_;
    estimateF_ = (Sl_ * j_.transpose()).lu().solve(Sl_ * hatTau_);

    /* Continuous Time equation */
    tau2c_ = (-(lambda_-2/0.002)*lastTau2c_ + lambda_ * x + lambda_ * lastX_)/(2/0.002 + lambda_);
//    tau2c_ = lambda_ * x + exp(-lambda_*0.002)*lastTau2c_;
    hatTau_ = tau1_ - tau2c_;
    estimateFc_ = (Sl_ * j_.transpose()).lu().solve(Sl_ * hatTau_);

    for(int i = 0; i < contactProFromF_.size(); i++) {
        contactProFromF_[i] = 0.5 * (1 + std::erf((estimateF_[3*i + 2] - 26)/sqrt(2*10)));
    }
    lastM_ = data.M;
    lastTau2_ = tau2_;
    lastTau2c_ = tau2c_;
    lastX_ = x;
    return estimateF_;
}

} // namespace legged