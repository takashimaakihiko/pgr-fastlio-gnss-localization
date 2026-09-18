// Point cloud dump tool: captures MID360(s) frames via Livox-SDK2 and
// writes them to an ascii PCD file (viewable with pcl_viewer,
// CloudCompare, or scripts/view_pointcloud.py).
//
// Usage:
//   pc_dump <lidar_cfg.json> <out.pcd> [duration_s=5] [max_points=500000]
//
// Points are accumulated in the lidar frame, so keep the sensor (or the
// scene) still during capture. intensity = reflectivity.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <deque>
#include <string>
#include <thread>
#include <vector>

#include "lidar_source.hpp"

namespace {

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop = true; }

struct Pt
{
    float x, y, z, intensity;
};

} // namespace

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::fprintf(stderr,
            "usage: %s <lidar_cfg.json> <out.pcd> [duration_s=5] [max_points=500000]\n",
            argv[0]);
        return 1;
    }
    const std::string lidar_cfg = argv[1];
    const std::string out_path  = argv[2];
    const double duration_s     = (argc > 3) ? std::atof(argv[3]) : 5.0;
    const size_t max_points     = (argc > 4) ? std::strtoul(argv[4], nullptr, 10)
                                             : 500000;

    std::signal(SIGINT, on_sigint);

    Mid360Source src;
    if (!src.init(lidar_cfg)) {
        std::fprintf(stderr, "[pc_dump] SDK init failed\n");
        return 1;
    }
    std::fprintf(stderr, "[pc_dump] SDK initialized\n");

    std::vector<Pt> cloud;
    cloud.reserve(max_points);
    std::atomic<uint64_t> frames{0};

    src.set_frame_callback([&](const custom_messages::CustomMsg &msg) {
        frames.fetch_add(1);
        for (const auto &p : msg.points) {
            if (cloud.size() >= max_points) break;
            // drop invalid / zero points
            if (p.x == 0.0 && p.y == 0.0 && p.z == 0.0) continue;
            cloud.push_back({static_cast<float>(p.x),
                             static_cast<float>(p.y),
                             static_cast<float>(p.z),
                             static_cast<float>(p.reflectivity)});
        }
    });


    const auto t0 = std::chrono::steady_clock::now();
    while (!g_stop.load()) {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
                .count();
        if (elapsed >= duration_s || cloud.size() >= max_points) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    src.shutdown();
    std::fprintf(stderr, "[pc_dump] %llu frames, %zu points -> %s\n",
                 static_cast<unsigned long long>(frames.load()),
                 cloud.size(), out_path.c_str());

    FILE *fp = std::fopen(out_path.c_str(), "w");
    if (!fp) {
        std::fprintf(stderr, "[pc_dump] cannot open %s\n", out_path.c_str());
        return 1;
    }
    std::fprintf(fp,
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "WIDTH %zu\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %zu\n"
        "DATA ascii\n",
        cloud.size(), cloud.size());
    for (const auto &p : cloud)
        std::fprintf(fp, "%.4f %.4f %.4f %.0f\n", p.x, p.y, p.z, p.intensity);
    std::fclose(fp);
    return 0;
}
