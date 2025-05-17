#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>
#include <string>

#include <legged_estimation/utils.hpp>
#include <legged_estimation/kinodynamics.h>

#include "timer.hpp"

namespace legged {

using namespace ocs2;
using namespace legged_robot;

class ContactEstimate : public kinodynamics {
    public:
        ContactEstimate();
        void calculateForce(const float gamma, const float beta, 
            const vec19& q, const vec18& v,const vec12& tau);
        void Kalman();

        void updateContact();
}       
}

/*things that need to happen in this file: 
1. Constructor mein kinodynamics ka constructor call karna hai
2. force calculate krna from kinodynamics
3. write the kalman filter for it
*/