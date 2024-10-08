//
// Created by qiayuan on 22-12-23.
//

#include "legged_didc/ControllerBase.h"

#include <qpOASES.hpp>

using namespace qpOASES;

namespace legged {

class DistributedIDC : public ControllerBase {
 public:
  using ControllerBase::ControllerBase;

  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;

 protected:

 private:
    Vector12 getDesiredContactForceqpOASES(const Vector6 &b);

    void resize_qpOASES_vars();

    void resize_eigen_vars();
    
    void update_problem_size();

    void print_real_t(real_t *matrix, int nRows, int nCols);

    void copy_Eigen_to_real_t(real_t *target, Eigen::MatrixXd &source, int nRows, int nCols);

    void copy_real_t_to_Eigen(Eigen::VectorXd &target, real_t *source, int len);

    void print_QPData();

//   scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
    vector_t torqueLimits_;
    scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
    scalar_t gravity_{};

    const double NEGATIVE_NUMBER = -1000000.0;
    const double POSITIVE_NUMBER = 1000000.0;

    const double MU = 0.3;
    const double GRAVITY = 9.81;

    real_t *H_qpOASES;

    real_t *A_qpOASES;

    real_t *g_qpOASES;

    real_t *lbA_qpOASES;

    real_t *ubA_qpOASES;

    real_t *xOpt_qpOASES;

    real_t *xOpt_initialGuess;

    matrix_t H_eigen, A_eigen, g_eigen, lbA_eigen, ubA_eigen;

    vector_t xOpt_eigen;

    uint8_t real_allocated, num_vars_qp, num_constr_qp, c_st;

    double fz_min, fz_max;

    int_t qp_exit_flag;

    int_t nWSR_qpOASES;

    real_t cpu_time;

    Eigen::Matrix<double, 18, 18> Nc, NcT, I;
    Eigen::Matrix<double, 18, 1> tau_full, qdd_cmd, PD;
    Vector12 tau_ff, Fc, F, F_prev;

    Vector6 b;

    Matrix6 Kpb, Kdb;
    Matrix12 Kpa, Kda;

    int counter = 0;
};

}  // namespace legged