//
// Created by qiayuan on 2022/6/24.
//

//TODO: publish to /contact instead of /test_topic

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_controllers/LeggedController.h"
#include "legged_controllers/Instrumentor.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_estimation/DiscreteTimeLPF.h>
#include <legged_estimation/ContactProbabilityFromGait.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
                                                                         leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));
  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto& joint_name : joint_names) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }
  auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Force estimation
  setupForceEstimate();

  // Contact Probability Model
  setupContactProbability();

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());
  
  #if PROFILING
  Benchmarker_ = std::make_shared<Instrumentor>();
  #endif

  return true;
}

void LeggedController::starting(const ros::Time& time) {
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, ros::Duration(0.002));
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");
  controllerTime_ = ros::Time::now();
  mpcRunning_ = true;

  #if PROFILING
  Benchmarker_->BeginSession("benchmarking", filepath);
  #endif 

}

void LeggedController::update(const ros::Time& time, const ros::Duration& period) {
  
  PROFILE_SCOPE("LeggedController::update");
  // InstrumentationTimer timer0("LeggedController::update", Benchmarker_);
  {

    vector_t optimizedState, optimizedInput;
    size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
    vector_t x;

    {
      PROFILE_SCOPE("updateStateEstimation");
    // InstrumentationTimer timer1("updateStateEstimation", Benchmarker_);
      updateStateEstimation(time, period);
    }

    {
      PROFILE_SCOPE("MPC update");
    // InstrumentationTimer timer2("MPC update", Benchmarker_);
      mpcMrtInterface_->setCurrentObservation(currentObservation_);
    // Load the latest MPC policy
      mpcMrtInterface_->updatePolicy();

      // Evaluate the current policy
      mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);
    }
    {
      PROFILE_SCOPE("WBC update");
    // InstrumentationTimer timer3("WBC update", Benchmarker_);
    // Whole body control
      currentObservation_.input = optimizedInput;

      x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
    }
    {
      PROFILE_SCOPE("OTHER OVERHEAD");
    // InstrumentationTimer timer4("Other overhead", Benchmarker_);

    vector_t torque = x.tail(12);

    

    vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
    vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

    // Safety check, if failed, stop the controller
    if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
      ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
      stopRequest(time);
    }

    for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
      hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
    }

    // Visualization
    robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
    selfCollisionVisualization_->update(currentObservation_);

    // Publish the observation. Only needed for the command interface
    observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
    }
  }
}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size()), jointEffort(hybridJointHandles_.size());
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
    jointEffort(i) = hybridJointHandles_[i].getEffort();
  }
  for (size_t i = 0; i < contacts.size(); ++i) {
    contactFlag[i] = contactHandles_[i].isContact();
  }
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  stateEstimate_->updateJointStates(jointPos, jointVel, jointEffort);
  // estimate the force of feet
  forceEstimate_->updateJointStates(jointPos, jointVel, jointEffort);

  stateEstimate_->updateContact(contactFlag); //why? what does this do?
  forceEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  forceEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);

  vector_t forceEstimate = forceEstimate_->update(time, period);
  vector_t kalmanTest = contactKal_->update(forceEstimate_.get(), contactProbabilityG_.get(), time, period);
  // FR, FL , RR, RL

  // is it possible to make this more efficient?
  dataShow_.data.clear();
  dataShow_.data.push_back(kalmanTest[0]);
  dataShow_.data.push_back(kalmanTest[1]);
  dataShow_.data.push_back(kalmanTest[2]);
  dataShow_.data.push_back(kalmanTest[3]);

  testPublisher_.publish(dataShow_);

// // In your update function:
// if ((ros::Time::now() - lastPublishTime_).toSec() >= 0.10) {
//     testPublisher_.publish(dataShow_);
//     lastPublishTime_ = ros::Time::now();
// }

  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();

}

LeggedController::~LeggedController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### Ended MPC thread";
  #if PROFILING
  std::cerr << "\n###   Benchmarking was on, results saved at" << filepath << std::endl;
  std::cerr << "\n### To view results, open the file on Chrome://tracing";
  std::cerr << "\n### To turn off benchmarking, set PROFILING to 0 in legged_controllers/include/Instrumentor.h" ;
  #else
  std::cerr << "\n###   Benchmarking was off, to turn on benchmarking set PROFILING to 1 in legged_controllers/include/Instrumentor.h" << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  #endif
  std::cerr << "########################################################################";

  #if PROFILING
  Benchmarker_->EndSession();
  #endif
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void LeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                // mpcTimer_.startTimer();
                PROFILE_SCOPE("MPC optimization");
                // InstrumentationTimer timer5("MPC optimization", Benchmarker_); //for some reason using Benchmarker_ creates an invalid file, is it because the destructor is not called properly?
                mpcMrtInterface_->advanceMpc();
                // mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedController::setupForceEstimate() {
    forceEstimate_ = std::make_shared<DiscreteTimeLPF>(leggedInterface_->getPinocchioInterface(),
                                                       leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
    ros::NodeHandle nh;
    testPublisher_ = nh.advertise<std_msgs::Float64MultiArray>("test_topic", 10); // this test can used to vis the results that we want
}

void LeggedController::setupContactProbability() {
    ros::NodeHandle nh;
    const std::string robotName = "legged_robot";
    contactProbabilityG_ = std::make_shared<ContactProbabilityFromGait>(nh, robotName);
    contactKal_ = std::make_shared<ContactKalman>();
}

void LeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
}


}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)