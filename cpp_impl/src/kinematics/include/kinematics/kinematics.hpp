#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>

namespace kinematics
{
    /* Overloaded functions for the forward kinematics */
    Eigen::Vector3d fk(Eigen::Vector3d q);
    Eigen::Vector3d fk(double q0,
                       double q1,
                       double q2,
                       double q3);
    Eigen::Vector3d fk(float q0,
                       float q1,
                       float q2,
                       float q3);

    Eigen::Vector4d ik(Eigen::Vector3d pos, double pitch=0);
    Eigen::Vector4d ik(double x, double y, double z, double pitch=0);
    Eigen::Vector4d ik(float x, float y, float z, float pitch=0);
    Eigen::Vector4d ik_nearest(double x, double y, double z,
                               double q0, double q1, double q2, double q3);

    bool in_workspace(double x, double y, double z);
}

#endif // KINEMATICS_H