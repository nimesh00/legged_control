//
// Created by Mehul_0x on 12/5/2025.
//
#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>

#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <string>

#include <legged_estimation/utils.hpp>
#include "timer.hpp"


#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <pinocchio/algorithm/model.hpp>
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/sample-models.hpp"

namespace legged {

using namespace ocs2;
using namespace legged_robot;

class ContactEstimate {
  public:
    ContactEstimate();

    enum FRAME {
      HIP,
      BASE,
      GLOBAL,
      LOCAL
    };
    

    vec18 convertGlobalToLocal(const vec19& q, const vec18& v_global);

    mat18x18 fullJacobian(const vec19& q, const FRAME& frame);

    mat18x18 fullJacobianDot(const vec19& q, const vec18& v, const FRAME& frame);

    vec18 getJointVelocities(const vec19& q, const vec18& xd);

    vec18 getJointAcceleration(const vec19& q, const vec18& qd, const vec18& xdd);

    mat18x18 massMatrix(const vec19 & q);

    vec18 getGeneralizedMomentum(const vec19 & q, const vec18 & v);

    vec18 gravitationalTerms(const vec19& q);

    mat18x18 coriolisMatrix(const vec19& q, const vec18& v);

    vec18 getExternalForcesFromMBO(const vec19& q,const vec18& tau_dist);
    
    vec18 convertLocalToGlobal(const vec19& q, const vec18& v_local);

    vec18 getMBOUpdate(const float beta, const vec19& q, const vec18& v,const vec12& tau);

    vec12 getEstimatedForcesFromMBO(const vec19& q,const vec18& tau_dist);

    vec18 getEstimatedtau(const float gamma, const float beta, 
                                  const vec19& q, const vec18& v,const vec12& tau);


  private:
    std::string urdf_filepath;

    pinocchio::Model model;
    pinocchio::Data data;
    int ID_FOOT_FRAME[4], ID_BASE;

    vec18 L_mbo_filtered_prev = vec18::Zero();
};
}