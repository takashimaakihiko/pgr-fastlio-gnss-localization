#ifndef POSE_UTILS_H_
#define POSE_UTILS_H_

#include <cmath>

namespace pose_utils
{

struct RPY
{
    double roll;
    double pitch;
    double yaw;
};

// quaternion (x,y,z,w) -> roll/pitch/yaw, ZYX convention
inline RPY quat_to_rpy(double qx, double qy, double qz, double qw)
{
    const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);

    const double sinp = 2.0 * (qw * qy - qz * qx);

    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);

    RPY rpy;
    rpy.roll = std::atan2(sinr_cosp, cosr_cosp);
    rpy.pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp)
                                      : std::asin(sinp);
    rpy.yaw = std::atan2(siny_cosp, cosy_cosp);
    return rpy;
}

// wrap angle to [-pi, pi]
inline double normalize_angle(double a)
{
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

} // namespace pose_utils

#endif
