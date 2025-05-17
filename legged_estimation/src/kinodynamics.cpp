//
// Created by Mehul_0x on 12/5/2025.
//

#include "legged_estimation/kinodynamics.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace legged {
using namespace legged_robot;

  kinodynamics::kinodynamics() { 
    urdf_filepath = "legged_control/legged_estimation/include/legged_estimation/go2_description.urdf"; //this urdf could be really wrong, took it from LAIR

    pinocchio::urdf::buildModel(urdf_filepath, pinocchio::JointModelFreeFlyer(), model);

    data = pinocchio::Data(model);

    std::string base_name = "base";

    if (model.existFrame(base_name)) {
        ID_BASE = model.getFrameId(base_name);
    } else {
        std::cerr << "Base frame: " << base_name
                  << " missing from kinematic tree\n";
    }

    std::vector<std::string> foot_name = {"FL", "FR", "RL", "RR"};

    for (int i = 0; i < 4; ++i) {
      // Save foot frame IDs
      if (model.existFrame(foot_name[i] + "_foot_site")) {
          ID_FOOT_FRAME[i] = model.getFrameId(foot_name[i] + "_foot_site");
      } else if (model.existFrame(foot_name[i] + "_foot")) {
          ID_FOOT_FRAME[i] = model.getFrameId(foot_name[i] + "_foot");
      } else {
          std::cerr << "Foot frame " << foot_name[i] + "_foot"
                    << " missing from kinematic tree." << std::endl;
      }
    }
  } 
  
    //15 hz cutoff, 1khz sampling
    float gamma = 0.0791;
  
    float beta = 0.088;
  
    //Local aur Global ka kya scene hai?

  vec18 kinodynamics::convertGlobalToLocal(const vec19& q, const vec18& v_global) {
    pinocchio::framesForwardKinematics(model, data, q);

    vec18 v_local = vec18::Zero();
    
    v_local.block<3, 1>(0, 0) =
        data.oMf[ID_BASE].rotation().transpose() * v_global.block<3, 1>(0, 0); 
    v_local.block<3, 1>(3, 0) =
        data.oMf[ID_BASE].rotation().transpose() * v_global.block<3, 1>(3, 0);

    for (int i = 0; i < 4; ++i) {
        v_local.block<3, 1>(6 + 3 * i, 0) =
            data.oMf[ID_FOOT_FRAME[i]].rotation().transpose() *
            v_global.block<3, 1>(6 + 3 * i, 0);
      }

    return v_local;
  }

  vec18 kinodynamics::convertLocalToGlobal(const vec19& q, const vec18& v_local) {
    pinocchio::framesForwardKinematics(model, data, q);

    vec18 v_global = vec18::Zero();

    v_global.block<3, 1>(0, 0) =
        data.oMf[ID_BASE].rotation() * v_local.block<3, 1>(0, 0);
    v_global.block<3, 1>(3, 0) =
        data.oMf[ID_BASE].rotation() * v_local.block<3, 1>(3, 0);

    for (int i = 0; i < 4; ++i) {
        v_global.block<3, 1>(6 + 3 * i, 0) =
            data.oMf[ID_FOOT_FRAME[i]].rotation() *
            v_local.block<3, 1>(6 + 3 * i, 0);
    }

    return v_global;
}

  //full jacobian vs getFrameJacobian vs computeJointJacobians?


  mat18x18 kinodynamics::fullJacobian(const vec19& q, const FRAME& frame) { 
    
    mat18x18 feet_jac = mat18x18::Zero();

    // Required 2 calls before calling getFrameJacobian
    pinocchio::computeJointJacobians(model, data, q);

    pinocchio::ReferenceFrame REF_FRAME; //reference frame ka dekhna hoga
    if (frame == FRAME::LOCAL) {
        REF_FRAME = pinocchio::LOCAL;
    } else if (frame == FRAME::GLOBAL) {
        REF_FRAME = pinocchio::LOCAL_WORLD_ALIGNED;
    }

    // Get the jacobian for base
    pinocchio::getFrameJacobian(model, data, ID_BASE, REF_FRAME,
                                feet_jac.block<6, 18>(0, 0));

    // Get the Jacobian for the feet
    for (int i = 0; i < 4; ++i) {
        mat6x18 tmp_jac = mat6x18::Zero();
        // Use LOCAL_WORLD_ALIGNED and NOT WORLD. Read the below explanation of
        // pinocchio::WORLD from the documentation The WORLD frame convention
        // corresponds to the frame coincident with the Universe/Inertial frame
        // but moving with the moving part (Joint, Frame, etc.).
        pinocchio::getFrameJacobian(model, data, ID_FOOT_FRAME[i], REF_FRAME,
                                    tmp_jac);
        feet_jac.block<3, 18>(6 + 3 * i, 0) = tmp_jac.block<3, 18>(0, 0);
    }

    return feet_jac;
  }


  //M(qk)
  
  mat18x18 kinodynamics::massMatrix(const vec19& q) {  //need to define model and data
    pinocchio::crba(model, data, q);
    // fill the lower triangle portion of the mass matrix
    data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose().triangularView<Eigen::StrictlyLower>();
    return data.M;
  }

  vec18 kinodynamics::getGeneralizedMomentum(const vec19& q, const vec18& v) { //tf is v?
    mat3x3 R = pinocchio::QuatToRot(q.block<4, 1>(3, 0));
    vec18 v_local = v;
    v_local.block<3, 1>(0, 0) = R.transpose() * v.block<3, 1>(0, 0);
    v_local.block<3, 1>(3, 0) = R.transpose() * v.block<3, 1>(3, 0);

    return massMatrix(q) * v_local;
  }

  vec18 kinodynamics::gravitationalTerms(const vec19& q) {
    pinocchio::computeGeneralizedGravity(model, data, q);
    return data.g;
  }

  vec18 kinodynamics::getJointVelocities(const vec19& q, const vec18& xd) { //xd represents the desired generalized velocity of the robot in the global frame. 
    mat18x18 J = fullJacobian(q, FRAME::LOCAL);
    vec18 qd;

    vec18 xd_local = convertGlobalToLocal(q, xd);

    if (abs(J.determinant()) < 1e-15) {
        qd = vec18::Zero();
    } else {
        qd = J.completeOrthogonalDecomposition().solve(xd_local);
    }

    mat3x3 R = data.oMf[ID_BASE].rotation();

    qd.block<3, 1>(0, 0) = R * qd.block<3, 1>(0, 0);
    qd.block<3, 1>(3, 0) = R * qd.block<3, 1>(3, 0);

    return qd;
  }

  mat18x18 kinodynamics::coriolisMatrix(const vec19& q, const vec18& v) {
    mat3x3 R = pinocchio::QuatToRot(q.block<4, 1>(3, 0));
    vec18 v_local = v;
    v_local.block<3, 1>(0, 0) = R.transpose() * v.block<3, 1>(0, 0);
    v_local.block<3, 1>(3, 0) = R.transpose() * v.block<3, 1>(3, 0);
    pinocchio::computeCoriolisMatrix(model, data, q, v_local);

    return data.C;
  }

  vec18 kinodynamics::getMBOUpdate(const float beta, const vec19& q, const vec18& v,const vec12& tau) { //ismein toh filter nahi use ho raha tha

    mat18x18 M = massMatrix(q);
    mat18x18 C = coriolisMatrix(q, v);
    vec18 G = gravitationalTerms(q);

    vec18 L_mbo = vec18::Zero();
    Eigen::Matrix<double, 18, 12> S_T = Eigen::Matrix<double, 18, 12>::Zero();
    S_T.block<12, 12>(6, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    vec18 v_local = v;
    mat3x3 R = pinocchio::QuatToRot(q.block<4, 1>(3, 0));
    v_local.block<3, 1>(0, 0) = R.transpose() * v.block<3, 1>(0, 0);
    v_local.block<3, 1>(3, 0) = R.transpose() * v.block<3, 1>(3, 0);

    L_mbo = beta * getGeneralizedMomentum(q, v) + S_T * tau +
    C.transpose() * v_local - G;

    return L_mbo;
  }
  
  vec18 kinodynamics::getEstimatedtau(const float gamma, const float beta, 
                                                  const vec19& q, const vec18& v,const vec12& tau){
    
    vec18 L_mbo = getMBOUpdate(beta, q, v, tau);
    
    vec18 L_mbo_filtered = gamma * L_mbo_filtered_prev + (1 - gamma) * L_mbo; //is this correct?

    L_mbo_filtered_prev = L_mbo_filtered;
                                             
    vec18 tau_dist= beta*getGeneralizedMomentum(q,v) - L_mbo_filtered;
    
    return tau_dist;
  }

  vec12 kinodynamics::getEstimatedForcesFromMBO(const vec19& q,
                                              const vec18& tau_dist) {

    vec12 fc_est_local = vec12::Zero();
    vec12 fc_est_global = vec12::Zero();

    mat18x18 J = fullJacobian(q, kinodynamics::FRAME::LOCAL);
    pinocchio::framesForwardKinematics(model, data, q);  //ye kyun use ho raha?

    for (int i = 0; i < 4; i++) {  //isemein selector matrix kaise implement ho raha?
      fc_est_local.block<3, 1>(3 * i, 0) =
      J.block<3, 3>(6 + 3 * i, 6 + 3 * i).inverse() *
      tau_dist.block<3, 1>(6 + 3 * i, 0);
      fc_est_global.block<3, 1>(3 * i, 0) =
      data.oMf[ID_FOOT_FRAME[i]].rotation() *
      fc_est_local.block<3, 1>(3 * i, 0);
    }

    return fc_est_global;
    }



}

