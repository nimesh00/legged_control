#pragma once

#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<int, 4, 1> int4;
typedef Eigen::Matrix<int, 12, 1> int12;
typedef Eigen::Matrix<double, 2, 1> vec2;
typedef Eigen::Matrix<double, 3, 1> vec3;
typedef Eigen::Matrix<double, 4, 1> vec4;
typedef Eigen::Matrix<double, 6, 1> vec6;
typedef Eigen::Matrix<double, 7, 1> vec7;
typedef Eigen::Matrix<double, 12, 1> vec12;
typedef Eigen::Matrix<double, 18, 1> vec18;
typedef Eigen::Matrix<double, 19, 1> vec19;
typedef Eigen::Matrix<double, 3, 3> mat3x3;
typedef Eigen::Matrix<double, 4, 4> mat4x4;
typedef Eigen::Matrix<double, 6, 6> mat6x6;
typedef Eigen::Matrix<double, 8, 8> mat8x8;
typedef Eigen::Matrix<double, 12, 12> mat12x12;
typedef Eigen::Matrix<double, 12, 18> mat12x18;
typedef Eigen::Matrix<double, 18, 18> mat18x18;
typedef Eigen::Matrix<double, 6, 12> mat6x12;
typedef Eigen::Matrix<double, 12, 6> mat12x6;
typedef Eigen::Matrix<double, 6, 18> mat6x18;
typedef Eigen::Matrix<double, 18, 6> mat18x6;

namespace pinocchio {
    inline mat3x3 QuatToRot(const vec4& q) {
        // pinocchio convention of {x, y, z, w}
        double q0 = q(3);
        vec3 qv = q.block<3, 1>(0, 0);
        mat3x3 Rot = (2 * q0 * q0 - 1) * mat3x3::Identity() +
                    2 * q0 * SkewSymm(qv) + 2 * qv * qv.transpose();

        return Rot;
    }

    inline mat3x3 SkewSymm(const vec3& v) {
        mat3x3 vx;
        vx << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;

        return vx;
    }
}