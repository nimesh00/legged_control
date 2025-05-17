//
// Created by sudaxia on 23-12-20.
//

#pragma once

#include <mutex>

#include <ros/ros.h>

#include <ocs2_core/Types.h>
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/** ContactProbabilityFromGait is designed to estimate the probability of the Contact of one leg,

 * which through the current phase of the now gait cycle

 */

class ContactProbabilityFromGait {
public:
    ContactProbabilityFromGait(::ros::NodeHandle nodeHandle, const std::string& robotName);

    int getTheSetTimeIndex(const ModeSchedule& modeSchedule);

    vector_t getProFromGait() {
        return contactProbability_;
    };

    const ModeSequenceTemplate& getReceivedGait() {  return receivedGait_; }

    vector_t update(const ModeSchedule& ModeSchedule, const ros::Time& time, const ros::Time& controllerTime, const ros::Duration& period);

private:
    void cfgModeSequenceCallback(const ocs2_msgs::mode_schedule::ConstPtr& msg);

    ::ros::Subscriber cfgModeSequenceSubscriber_;

    std::mutex receivedGaitMutex_;
    ModeSequenceTemplate receivedGait_;
    std::atomic_bool gaitUpdated_;
    Eigen::Matrix<scalar_t, 4, 1> contactProbability_;

};


}  // namespace legged


