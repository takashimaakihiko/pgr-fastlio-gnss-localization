#ifndef TRAJECTORY_LOGGER_H_
#define TRAJECTORY_LOGGER_H_

#include <fstream>
#include <iomanip>
#include <string>

// CSV logger for estimated trajectory.
// Format: timestamp,x,y,z,roll,pitch,yaw
// timestamp = lidar end time of the scan (s, lidar/IMU clock domain)
class TrajectoryLogger
{
public:
    bool open(const std::string &path)
    {
        ofs_.open(path, std::ios::out | std::ios::trunc);
        if (!ofs_.is_open())
        {
            return false;
        }
        ofs_ << "timestamp,x,y,z,roll,pitch,yaw\n";
        ofs_ << std::fixed << std::setprecision(9);
        return true;
    }

    void write(double timestamp, double x, double y, double z,
               double roll, double pitch, double yaw)
    {
        if (!ofs_.is_open())
        {
            return;
        }
        ofs_ << timestamp << "," << x << "," << y << "," << z << ","
             << roll << "," << pitch << "," << yaw << "\n";
    }

    void flush()
    {
        if (ofs_.is_open())
        {
            ofs_.flush();
        }
    }

private:
    std::ofstream ofs_;
};

#endif
