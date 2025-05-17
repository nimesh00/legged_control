//
// Created by sudaxia on 23-12-20.
//
#include <legged_estimation/ContactProbabilityFromGait.h>

#include "ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h"

namespace legged {
ContactProbabilityFromGait::ContactProbabilityFromGait(ros::NodeHandle nodeHandle, const std::string &robotName)
    : receivedGait_({0.0, 1.0}, {ModeNumber::STANCE}) {
    contactProbability_.setZero();
    cfgModeSequenceSubscriber_ = nodeHandle.subscribe(robotName + "_mpc_mode_schedule", 1, &ContactProbabilityFromGait::cfgModeSequenceCallback, this,
                                                      ::ros::TransportHints().udp());
}

int ContactProbabilityFromGait::getTheSetTimeIndex(const ModeSchedule& modeSchedule) {
    std::vector<size_t> nowSequence = modeSchedule.modeSequence;
    std::vector<size_t> receSequence = receivedGait_.modeSequence;
    int lenNow = nowSequence.size();
    int lenRece = receSequence.size();
    int StartTimeIndex = -1;
    // Loop over nowSequence array to find the first occurrence of receSequence
    for (int i = 0; i <= lenNow - lenRece; ++i) {
        bool match = true;
        // Check if the subarray of nowSequence starting at i is equal to receSequence
        for (int j = 0; j < lenRece; ++j) {
            if (nowSequence[i + j] != receSequence[j]) {
                match = false;
                break;
            }
        }
        if (match) {
            StartTimeIndex = i - 1;
            return StartTimeIndex;
        }  // Return the index of the set time
    }
    return -1;
}

vector_t ContactProbabilityFromGait::update(const ModeSchedule& ModeSchedule, const ros::Time &time, const ros::Time& controllerTime, const ros::Duration &period) {
    if(gaitUpdated_) {
        std::cerr << "Current received Gait is : " << receivedGait_ << std::endl;
        gaitUpdated_ = false;
    }
    // Need to know the current mode in this cycle
    auto startTimeIndex = getTheSetTimeIndex(ModeSchedule);
    auto currentTime = (time-controllerTime).toSec();
    auto currentMode = 15;
    scalar_t phaseTime = currentTime - ModeSchedule.eventTimes[startTimeIndex];
    scalar_t phase = 0;
    while(phaseTime < 0 || phaseTime > receivedGait_.switchingTimes.back()) {
        if( phaseTime < 0 ) {
            phaseTime += receivedGait_.switchingTimes.back();
        }
        else if( phaseTime > receivedGait_.switchingTimes.back() ) {
            phaseTime -= receivedGait_.switchingTimes.back();
        }
    }

    for( int ip = 0; ip < receivedGait_.switchingTimes.size(); ip++ ) {
        if(phaseTime - receivedGait_.switchingTimes[ip] < 0) {
            currentMode = receivedGait_.modeSequence[ip-1];
            phase = (phaseTime - receivedGait_.switchingTimes[ip-1])/(receivedGait_.switchingTimes[ip] - receivedGait_.switchingTimes[ip-1]);
            break;
        }
    }
    contact_flag_t contactFlag = modeNumber2StanceLeg(currentMode);
    for( int i = 0; i < 4; i++ ) {
        /* In this part, we should define average and covariance in ./config/a1/task.info  */
        contactProbability_[i] = 0.5 * (contactFlag[i] * (std::erf(phase/sqrt(2*0.025)) + std::erf((1-phase)/sqrt(2*0.025))) +
                                      !contactFlag[i] * (2 + std::erf((-phase)/ sqrt(2*0.025)) + std::erf((phase-1)/ sqrt(2*0.025))));
    }

    return contactProbability_;
}

void ContactProbabilityFromGait::cfgModeSequenceCallback(const ocs2_msgs::mode_schedule::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(receivedGaitMutex_);
    receivedGait_ = readModeSequenceTemplateMsg(*msg);
    gaitUpdated_ = true;
}

} // namespace legged