#include "kinematics.hpp"
#include <iostream>

namespace kinematics
{
    constexpr double L[] = {0.05, 0.02, 0.0825, 0.086, 0.075};

    Eigen::Matrix3d rot_x(double theta)
    {
        double ct = cos(theta), st = sin(theta);
        Eigen::Matrix3d rot;
        rot << 1.0,  0.0,   0.0,
               0.0,   ct, -st,
               0.0,   st,  ct;

        return rot;
    }

    Eigen::Matrix3d rot_y(double theta)
    {
        double ct = cos(theta), st = sin(theta);
        Eigen::Matrix3d rot;
        rot << ct,  0,  st,
                0,  1,   0,
              -st,  0,  ct;

        return rot;
    }

    Eigen::Matrix3d rot_z(double theta)
    {
        double ct = cos(theta), st = sin(theta);
        Eigen::Matrix3d rot;
        rot << ct,  -st,  0,
               st,   ct,  0,
                0,    0,  1;

        return rot;
    }

    Eigen::Vector3d fk(Eigen::Vector4d q)
    {
        return fk(q(0), q(1), q(2), q(3));
    }
    Eigen::Vector3d fk(double q0,
                       double q1,
                       double q2,
                       double q3)
    {
        return fk((float)q0, (float)q1, (float)q2, (float)q3);
    }
    Eigen::Vector3d fk(float q0,
                       float q1,
                       float q2,
                       float q3)
    {
        double q[] = {q0, q1, q2, q3};
        Eigen::Vector3d bar(0, 0, 1);
        Eigen::Vector3d pos = bar * (L[0] + L[1]);
        Eigen::Matrix3d rot = rot_z(q[0]);

        for(uint i = 1; i < 4; i++)
        {
            rot *= rot_y(q[i]);
            pos += rot * (L[i+1] * bar);
        }
 
        return pos; 
    }

    Eigen::Vector4d ik(Eigen::Vector3d pos, double pitch)
    {
        return ik(pos(0), pos(1), pos(2), pitch);
    }

    Eigen::Vector4d ik(double x, double y, double z, double pitch)
    {
        return ik((float)x, (float)y, (float)z, (float)pitch);
    }

    Eigen::Vector4d ik(float x, float y, float z, float pitch)
    {
        double q0, q1, q2, q3;
        if(in_workspace(x,y,z))
        {
            // We need to subtract the base height from z
            z -= (L[0] + L[1]);

            // From the desired x and y we can directly find q0
            q0 = atan2(y, x);

            // Then we find the desired wrist location from the pitch
            // We can assume everything is in the xz plane, given that the remaining 
            // rotation is handled by q0
            double r = sqrt(x*x + y*y);
            double xw = r - L[4] * cos(pitch);
            double zw = z - L[4] * sin(pitch);

            // Then we find q2 and q1 from the law of sines/cosines
            q2 = acos((xw*xw + zw*zw - L[2]*L[2] - L[3]*L[3]) / (2*L[2]*L[3]));
            double c1 = ((L[2] + L[3]*cos(q2)) * xw + L[3]*sin(q2)*zw) / (xw*xw + zw*zw);
            double s1 = ((L[2]+L[3]*cos(q2)) * zw - L[3]*sin(q2)*xw) / (xw*xw + zw*zw);
            q1 = atan2(s1, c1);

            // q3 directly follows from the pitch
            q3 = pitch - (q1 + q2);
        }
        else
        {
            std::cerr << "Demanded position lies outside the workspace." << std::endl;
            double deg2rad = M_PI * 0.005555555555555556;
            q0 = deg2rad * 45;
            q1 = deg2rad * -50;
            q2 = deg2rad * 110;
            q3 = deg2rad * 30;
        }

        return Eigen::Vector4d(q0, q1, q2, q3);
    }

    Eigen::Vector4d ik_nearest(double x, double y, double z,
                               double q0, double q1, double q2, double q3)
    {
        (void)x, (void)y, (void)z;
        (void)q0, (void)q1, (void)q2, (void)q3;
        Eigen::Vector4d q;
        //TODO
        return q;
    }

    bool in_workspace(double x, double y, double z)
    {
        Eigen::Vector3d loc(x, y, z);
        
        // Outside the ws if distance to the top of the base link is larger
        // then the sum of the length of the rest of the links
        if((loc - Eigen::Vector3d(0, 0, L[0])).norm() > L[1] + L[2] + L[3] + L[4])
            return false;

        return true;
    }
}