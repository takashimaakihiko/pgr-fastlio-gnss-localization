// lio_node: MID360 + FAST-LIO realtime odometry (no ROS)
//
// Usage:
//   lio_node <sdk_config.json> <fastlio_config.json> <out_dir>
//
//   sdk_config.json     Livox-SDK2 network config (host_ip, ports)
//                       default: ../config/mid360_lidar.json
//   fastlio_config.json FAST-LIO parameters
//                       default: ../config/mid360_fastlio.json
//   out_dir             output directory for trajectory.csv / output.txt
//                       default: ../data/trajectory/

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "fast_lio.hpp"
#include "lidar_source.hpp"
#include "pose_utils.hpp"
#include "trajectory_logger.hpp"

namespace
{
std::atomic<bool> g_stop{false};

void sig_handler(int)
{
    g_stop.store(true);
}
} // namespace

int main(int argc, char **argv)
{
    const std::string sdk_cfg =
        (argc > 1) ? argv[1] : "../config/mid360_lidar.json";
    const std::string lio_cfg =
        (argc > 2) ? argv[2] : "../config/mid360_fastlio.json";
    std::string out_dir =
        (argc > 3) ? argv[3] : "../data/trajectory";
    if (!out_dir.empty() && out_dir.back() != '/')
    {
        out_dir += '/';
    }

    FastLio lio(lio_cfg, out_dir);

    TrajectoryLogger logger;
    if (!logger.open(out_dir + "trajectory.csv"))
    {
        std::cerr << "[lio_node] cannot open csv: " << out_dir
                  << "trajectory.csv" << std::endl;
        return 1;
    }

    Mid360Source src;
    src.set_frame_callback([&lio](const custom_messages::CustomMsg &msg) {
        lio.feed_lidar(
            custom_messages::CstMsgConstPtr(new custom_messages::CustomMsg(msg)));
    });
    src.set_imu_callback([&lio](const custom_messages::Imu &msg) {
        lio.feed_imu(
            custom_messages::ImuConstPtr(new custom_messages::Imu(msg)));
    });

    if (!src.init(sdk_cfg))
    {
        return 1;
    }

    std::signal(SIGINT, sig_handler);
    std::signal(SIGTERM, sig_handler);

    std::cout << "[lio_node] running. logging to " << out_dir
              << "trajectory.csv" << std::endl;

    double last_stamp = 0.0;
    uint64_t frame_count = 0;

    while (!g_stop.load())
    {
        lio.process();

        if (lio.has_pose())
        {
            const double stamp = lio.get_timestamp();
            if (stamp != last_stamp)
            {
                last_stamp = stamp;
                const auto pose = lio.get_pose();
                const auto rpy = pose_utils::quat_to_rpy(
                    pose[3], pose[4], pose[5], pose[6]);
                const double theta = pose_utils::normalize_angle(rpy.yaw);
                logger.write(stamp, pose[0], pose[1], pose[2],
                             rpy.roll, rpy.pitch, theta);
                if (++frame_count % 10 == 0)
                {
                    logger.flush();
                    std::cout << "\r[lio_node] frames: " << frame_count
                              << "  x: " << pose[0] << "  y: " << pose[1]
                              << "  theta: " << theta << "   " << std::flush;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    logger.flush();
    src.shutdown();
    std::cout << "\n[lio_node] stopped. " << frame_count << " frames logged."
              << std::endl;
    return 0;
}
