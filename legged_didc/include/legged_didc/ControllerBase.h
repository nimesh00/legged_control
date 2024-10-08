#pragma once

// #include "legged_wbc/Task.h"
#include <ocs2_core/Types.h>

#include <utility>

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
using Vector12 = Eigen::Matrix<scalar_t, 12, 1>;
using Vector18 = Eigen::Matrix<scalar_t, 18, 1>;
using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;
using Matrix12 = Eigen::Matrix<scalar_t, 12, 12>;

class ControllerBase {
 public:
  ControllerBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  // virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                          scalar_t period);

 protected:
  void updateMeasured(const vector_t& rbdStateMeasured);
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

  size_t getNumDecisionVars() const { return numDecisionVars_; }

  size_t numDecisionVars_;
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  CentroidalModelInfo info_;

  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  CentroidalModelPinocchioMapping mapping_;

  vector_t qMeasured_, vMeasured_, inputLast_;
  matrix_t j_, dj_;
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // Task Parameters:

  vector_t qDesired_, vDesired_;
  vector_t vDesired_last_;
  matrix_t J, Jc, Jd, J_r, M;
  vector_t H, G;
};

}  // namespace legged
