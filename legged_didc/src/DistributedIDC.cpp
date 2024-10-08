//
// Created by qiayuan on 22-12-23.
//

#include "legged_didc/DistributedIDC.h"

#include <qpOASES.hpp>

namespace legged {

vector_t DistributedIDC::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                             scalar_t period) {
  ControllerBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

    real_allocated = 0;
    num_vars_qp = 0;
    num_constr_qp = 0;
    c_st = 0;
    fz_min = 10;
    fz_max = 200;

//   std::cerr << "Controller update done!\n";

  Vector6 base_err = Vector6::Zero();
  base_err = qDesired_.head(6) - qMeasured_.head(6);
//   base_err(3) = qDesired_(5) - qMeasured_(5);
//   base_err(4) = qDesired_(4) - qMeasured_(4);
//   base_err(5) = qDesired_(3) - qMeasured_(3);
  Vector12 joint_err = qDesired_.tail(12) - qMeasured_.tail(12);

    // Vector6 kpb;
    // // Non-zero position gains interfere with orientation torques as well. A big F for DIDC lol!
    // kpb << 0.0 * 316.22, 0.0 * 316.22, 0.1 * 316.22,
    //     2316.22, 2316.22, 0.0 * 316.22;
    // Kpb = kpb.asDiagonal();

    // Vector6 kdb;
    // kdb << 50.40, 50.40, 50.40,
    //     50.4, 50.4, 50.40;
    // Kdb = kdb.asDiagonal();

    // Vector12 kpa;
    // Eigen::Vector3d kp(26.26, 26.26, 26.26);
    // kpa << kp, kp, kp, kp;
    // Kpa = kpa.asDiagonal();

    // Vector12 kda;
    // Eigen::Vector3d kd(13.03, 13.53, 13.53);
    // kda << kd, kd, kd, kd;
    // Kda = kda.asDiagonal();

    // Vector18 wn2 = Vector18::Zero();
    // wn2 << 0, 0, 1, 500, 500, 100, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50;

    // Vector18 Kp = M.diagonal().asDiagonal() * wn2;
    // Vector18 Kd = 2 * Kp.cwiseSqrt() * 1.2;
    // Kd(0) = 5;
    // Kd(1) = 5;
    // // std::cout << "Kp: \n" << Kp << "\n";
    // // std::cout << "Kd: \n" << Kd << "\n";
    // Kpb = Kp.block<6, 1>(0, 0).asDiagonal();
    // Kpa = Kp.block<12, 1>(6, 0).asDiagonal();
    // Kdb = Kd.block<6, 1>(0, 0).asDiagonal();
    // Kda = Kd.block<12, 1>(6, 0).asDiagonal();

    // PD.head(6) = Kpb * base_err
    //             + Kdb * (vDesired_.head(6) - vMeasured_.head(6));
    // PD.tail(12) = Kpa * joint_err
    //             + Kda * (vDesired_.tail(12) - vMeasured_.tail(12));
    
    qdd_cmd = (vDesired_ - vDesired_last_) / period;
    float a_roll = qdd_cmd(5);
    float a_pitch = qdd_cmd(4);
    float a_yaw = qdd_cmd(3);
    qdd_cmd(3) = a_roll;
    qdd_cmd(4) = a_pitch;
    qdd_cmd(5) = a_yaw;
    // qdd_cmd(2) += PD(2);

    tau_full = M * qdd_cmd + H;

    b = tau_full.head(6);
    // b.block<3,1>(0, 0) = tau_full.head(3);
    // b(3) = 1 * tau_full(5);
    // b(4) = 1 * tau_full(4);
    // b(5) = 1 * tau_full(3);

    // b(0) = std::min(std::max(b(0), -50.), 50.);
    // b(1) = std::min(std::max(b(1), -50.), 50.);
    // b(2) = std::min(std::max(b(2), -200.), 200.);
    // b(3) = std::min(std::max(b(3), -200.), 200.);
    // b(4) = std::min(std::max(b(4), -200.), 200.);
    // b(5) = std::min(std::max(b(5), -50.), 50.);

    b(0) = std::min(std::max(b(0), -10.), 10.);
    b(1) = std::min(std::max(b(1), -10.), 10.);
    b(2) = std::min(std::max(b(2), -200.), 200.);
    b(3) = std::min(std::max(b(3), -20.), 20.);
    b(4) = std::min(std::max(b(4), -30.), 30.);
    b(5) = std::min(std::max(b(5), -5.), 5.);

    Fc = getDesiredContactForceqpOASES(b);

    // distributing the 18x1 generalized force into 12x1 joint space torques
    Eigen::MatrixXd JabT = J.block<12, 6>(6, 0).transpose();
    Eigen::MatrixXd Jaa = J.block<12, 12>(6, 6);
    Eigen::MatrixXd JaaT = Jaa.transpose();
    Eigen::MatrixXd JabT_inv = JabT.transpose() * (JabT * JabT.transpose()).inverse();
    // Map from body wrench to joint torques: should be -JaaT * JabT_inv
    Eigen::MatrixXd J_b2t = -JaaT * JabT_inv;
    Eigen::MatrixXd N_b2t = (Eigen::MatrixXd::Identity(12, 12) - J_b2t * (J_b2t.transpose() * J_b2t).inverse() * J_b2t.transpose());

    tau_ff = -JaaT * Fc + N_b2t * tau_full.tail(12);

    if (counter++ == 100) {
        // std::cerr << "base_err: " << base_err.transpose() << "\n";
        // std::cerr << "b: " << b.transpose() << "\n";
        // std::cerr << "q_ref: " << qDesired_.head(6).transpose() << "\n";
        // std::cerr << "q_act: " << qMeasured_.head(6).transpose() << "\n";
        // std::cerr << "Fc: " << Fc.transpose() << "\n";
        // std::cerr << "tau_ff: " << tau_ff.transpose() << "\n";
        counter = 0;
    }

    vDesired_last_ = vDesired_;

  return tau_ff;
}

// QP resizing
void DistributedIDC::resize_qpOASES_vars()
{
    if (real_allocated)
    {
        free(H_qpOASES);
        free(A_qpOASES);
        free(g_qpOASES);
        free(lbA_qpOASES);
        free(ubA_qpOASES);
        free(xOpt_qpOASES);
        free(xOpt_initialGuess);
    }

    H_qpOASES = (real_t *)malloc(num_vars_qp * num_vars_qp * sizeof(real_t));
    A_qpOASES = (real_t *)malloc(num_constr_qp * num_vars_qp * sizeof(real_t));
    g_qpOASES = (real_t *)malloc(num_vars_qp * 1 * sizeof(real_t));
    lbA_qpOASES = (real_t *)malloc(num_constr_qp * 1 * sizeof(real_t));
    ubA_qpOASES = (real_t *)malloc(num_constr_qp * 1 * sizeof(real_t));
    xOpt_qpOASES = (real_t *)malloc(num_vars_qp * 1 * sizeof(real_t));
    xOpt_initialGuess = (real_t *)malloc(num_vars_qp * 1 * sizeof(real_t));

    real_allocated = 1;
    // std::cout << "Resized QP vars." << std::endl;
}

// Eigen QP matrices resizing
void DistributedIDC::resize_eigen_vars()
{
    H_eigen.resize(num_vars_qp, num_vars_qp);
    A_eigen.resize(num_constr_qp, num_vars_qp);
    g_eigen.resize(num_vars_qp, 1);
    lbA_eigen.resize(num_constr_qp, 1);
    ubA_eigen.resize(num_constr_qp, 1);
    xOpt_eigen.resize(num_vars_qp, 1);

    H_eigen.setZero();
    A_eigen.setZero();
    g_eigen.setZero();
    lbA_eigen.setZero();
    ubA_eigen.setZero();
    xOpt_eigen.setZero();

    // std::cout << "Resized Eigen vars." << std::endl;
}

void DistributedIDC::update_problem_size() {
    c_st = 0;
    for (int i = 0; i < 4; i++) {
        if (contactFlag_[i]) {
            c_st++;
        }
    }

    num_vars_qp = 3 * c_st;
    num_constr_qp = 5 * c_st;
    // std::cout << "Updated problem size." << std::endl;
}

void DistributedIDC::copy_Eigen_to_real_t(real_t *target, Eigen::MatrixXd &source, int nRows, int nCols)
{
    int count = 0;

    // Strange Behavior: Eigen matrix matrix(count) is stored by columns (not rows)
    for (int i = 0; i < nRows; i++)
    {
        for (int j = 0; j < nCols; j++)
        {
            target[count] = source(i, j);
            count++;
        }
    }
}

void DistributedIDC::copy_real_t_to_Eigen(Eigen::VectorXd &target, real_t *source, int len)
{
    for (int i = 0; i < len; i++)
    {
        target(i) = source[i];
    }
}

void DistributedIDC::print_real_t(real_t *matrix, int nRows, int nCols)
{
    int count = 0;
    for (int i = 0; i < nRows; i++)
    {
        for (int j = 0; j < nCols; j++)
        {
            std::cout << matrix[count] << "\t";
            count++;
        }
        std::cout << "\n";
    }
}

void DistributedIDC::print_QPData()
{
    std::cout << "\n\n";
    std::cout << "\n\nH = ";

    print_real_t(H_qpOASES, num_vars_qp, num_vars_qp);
    std::cout << "\n\nA = ";
    print_real_t(A_qpOASES, num_constr_qp, num_vars_qp);
    std::cout << "\n\ng = ";
    print_real_t(g_qpOASES, num_vars_qp, 1);
    std::cout << "\n\nlbA = ";
    print_real_t(lbA_qpOASES, num_constr_qp, 1);
    std::cout << "\n\nubA = ";
    print_real_t(ubA_qpOASES, num_constr_qp, 1);
}

// Eigen::VectorXd DistributedIDC::clipVector(const Eigen::VectorXd &b, float F)
// {
//     Eigen::VectorXd result = b;
//     for (int i = 0; i < result.size() / 2; ++i)
//     {
//         if (result[i] > F)
//         {
//             result[i] = F;
//         }
//         else if (result[i] < -F)
//         {
//             result[i] = -F;
//         }
//     }
//     return result;
// }

// void DistributedIDC::cleanFc(Vector12 &Fc)
// {
//     for (int i = 0; i < 4; i++)
//     {
//         if (Fc(3 * i + 2) < 0)
//             Fc(3 * i + 2) = fz_min;

//         if (Fc(3 * i + 2) > fz_max)
//             Fc(3 * i + 2) = fz_max;
//     }
// }

inline Eigen::Matrix3d skew_symm(const Eigen::Vector3d &v) {
    Eigen::Matrix3d vx;
    vx << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;

    return vx;
}

// balance DistributedIDC for the stance legs
Vector12 DistributedIDC::getDesiredContactForceqpOASES(const Vector6& b) {
    eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
    Eigen::VectorXd rc = qMeasured_.head(3);          // CoM actual position
    std::vector<vector3_t> pf = eeKinematics_->getPosition(vector_t()); // computing the actual sim feet position in the inertial frame

    // std::cout << "b:" << b.lpNorm<Eigen::Infinity>() << std::endl;

    // update the QP matrices as per the current contact state
    update_problem_size();
    resize_qpOASES_vars();
    resize_eigen_vars();

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, num_vars_qp);
    Eigen::MatrixXd f_prev(num_vars_qp, 1);

    int nc = 0;
    for (int i = 0; i < 4; i++) {
        if (contactFlag_[i]) {
            A.block<3, 3>(0, 3 * nc) = Eigen::MatrixXd::Identity(3, 3);
            A.block<3, 3>(3, 3 * nc) = skew_symm(pf[i] - rc);
            f_prev.block<3, 1>(3 * nc, 0) = F_prev.block<3, 1>(3 * i, 0);
            nc++;
        }
    }

    Eigen::VectorXd s(6);
    s << 1, 1, 2, 20, 20, 5;
    // s << 1, 1, 5, 100, 100, 50;
    Eigen::MatrixXd S = s.asDiagonal();
    Eigen::MatrixXd W = 1e-2 * Eigen::MatrixXd::Identity(num_vars_qp, num_vars_qp);
    Eigen::MatrixXd V = 1e-3 * Eigen::MatrixXd::Identity(num_vars_qp, num_vars_qp);

    // CALCULATE H
    H_eigen = 2 * (A.transpose() * S * A + W + V);

    copy_Eigen_to_real_t(H_qpOASES, H_eigen, num_vars_qp, num_vars_qp);

    // CALCULATE g
    g_eigen = -2 * A.transpose() * S * b;
    g_eigen += -2 * V * f_prev;

    copy_Eigen_to_real_t(g_qpOASES, g_eigen, num_vars_qp, 1);

    // CALCULATE A
    // for now assuming mu is fixed, and n_i = [0 0 1], t1 = [1 0 0] and t2 = [0 1 0]
    Eigen::MatrixXd C_i(5, 3);
    C_i << 1, 0, -MU,
        0, 1, -MU,
        0, 1, MU,
        1, 0, MU,
        0, 0, 1;

    for (int i = 0; i < c_st; i++)
    {
        A_eigen.block<5, 3>(5 * i, 3 * i) = C_i;
    }

    copy_Eigen_to_real_t(A_qpOASES, A_eigen, num_constr_qp, num_vars_qp);

    // CALCULATE lbA and ubA
    Eigen::VectorXd di_lb(5);
    Eigen::VectorXd di_ub(5);

    di_lb << NEGATIVE_NUMBER,
        NEGATIVE_NUMBER,
        0,
        0,
        fz_min;

    di_ub << 0,
        0,
        POSITIVE_NUMBER,
        POSITIVE_NUMBER,
        fz_max;

    for (int i = 0; i < c_st; i++)
    {
        lbA_eigen.block<5, 1>(5 * i, 0) = di_lb;
        ubA_eigen.block<5, 1>(5 * i, 0) = di_ub;
    }

    copy_Eigen_to_real_t(lbA_qpOASES, lbA_eigen, num_constr_qp, 1);
    copy_Eigen_to_real_t(ubA_qpOASES, ubA_eigen, num_constr_qp, 1);

    // update the previous time-step's data in the initial guess
    copy_Eigen_to_real_t(xOpt_initialGuess, f_prev, num_vars_qp, 1);

    // solve the QP only if there is at least a leg in stance
    if (num_vars_qp)
    {
        QProblem qp_obj(num_vars_qp, num_constr_qp);
        Options options;
        options.setToMPC();
        options.printLevel = PL_NONE;
        qp_obj.setOptions(options);

        // print_QPData();

        nWSR_qpOASES = 1000;
        qp_exit_flag = qp_obj.init(
            H_qpOASES, g_qpOASES, A_qpOASES, nullptr, nullptr, lbA_qpOASES,
            ubA_qpOASES, nWSR_qpOASES, &cpu_time);

        // std::cout << "Exit flag: " << qp_exit_flag << std::endl;
        // std::cout << "num_vars_qp: " << unsigned(num_vars_qp) << std::endl;

        qp_obj.getPrimalSolution(xOpt_qpOASES);
        copy_real_t_to_Eigen(xOpt_eigen, xOpt_qpOASES, num_vars_qp);
    }

    Eigen::VectorXd F = Eigen::VectorXd(12);

    nc = 0;
    for (int i = 0; i < 4; i++)
    {
        if (contactFlag_[i] == 0)
        // if (m_cs(i) == 0)
        {
            F.block<3, 1>(3 * i, 0) = Eigen::Vector3d(0, 0, 0);
            continue;
        }
        F.block<3, 1>(3 * i, 0) = xOpt_eigen.block<3, 1>(3 * nc++, 0);
    }

    F_prev = F;

    return F; // this is in inertial frame
}

}  // namespace legged
