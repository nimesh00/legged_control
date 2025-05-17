//
// Created by sudaxia on 24-1-18.
//

#pragma once

#include <legged_estimation/StateEstimateBase.h>
#include <legged_estimation/DiscreteTimeLPF.h>
#include <legged_estimation/ContactProbabilityFromGait.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {
using namespace ocs2;

class ContactKalman {
public:
    ContactKalman();

    vector_t update(DiscreteTimeLPF* footEstimate, ContactProbabilityFromGait* gaitEstimate, const ros::Time& time, const ros::Duration& period);

private:
    Eigen::Matrix<scalar_t, 4, 1> xHat_;
    Eigen::Matrix<scalar_t, 4, 4> A_;
    Eigen::Matrix<scalar_t, 4, 4> B_;
    Eigen::Matrix<scalar_t, 4, 4> Q_;
    Eigen::Matrix<scalar_t, 8, 8> R_;
    Eigen::Matrix<scalar_t, 4, 4> P_;
    Eigen::Matrix<scalar_t, 8, 4> H_;
    Eigen::Matrix<scalar_t, 8, 1> Z_;

};

}   // namespace legged

