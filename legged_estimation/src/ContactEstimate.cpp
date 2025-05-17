#include "legged_estimation/ContactEstimate.h"

namespace legged {
    ContactEstimate::ContactEstimate():kinodynamics() {
        // Constructor implementation
        // Initialize any other members if needed
    }
    void ContactEstimate::calculateForce(const float gamma, const float beta, 
                                        const vec19& q, const vec18& v,const vec12& tau) {
        

        vec18 tau_dist=getEstimatedtau(gamma, beta, q, v, tau);
        vec12 foot_forces= getEstimatedForcesFromMBO(q, tau_dist);

    }
    void ContactEstimate::Kalman() {
        // Implement the Kalman filter logic here
        // This function should use the kinodynamics class to perform Kalman filtering
    }
    void ContactEstimate::updateContact() {
        // Implement the contact update logic here
        // This function should update the contact information based on the current state
    }
}


