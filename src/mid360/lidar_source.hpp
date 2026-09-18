#ifndef MID360_LIDAR_SOURCE_H_
#define MID360_LIDAR_SOURCE_H_

#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"
#include "msgs.h"

// Livox MID360 data source built on Livox-SDK2 (no ROS).
//
// Replicates the semantics of livox_ros_driver2's CustomMsg path so that
// FAST-LIO's livox handler can consume the data directly:
//  - ethernet point packets are aggregated into frames spanning
//    frame_interval_ns (default 100 ms -> ~10 Hz)
//  - frame header.stamp / timebase = absolute ns timestamp of the first
//    buffered point; per-point offset_time = ns relative to frame base
//  - per-point absolute time = packet timestamp + index * point_interval,
//    where point_interval = packet.time_interval * 100 / dot_num (ns)
//  - when the lidar reports "no time sync", the host steady clock at
//    packet reception is used (same as the official driver)
class Mid360Source
{
public:
    using FrameCallback = std::function<void(const custom_messages::CustomMsg &)>;
    using ImuCallback   = std::function<void(const custom_messages::Imu &)>;

    Mid360Source() = default;
    ~Mid360Source();

    bool init(const std::string &sdk_config_path);
    void shutdown();

    void set_frame_callback(FrameCallback cb) { frame_cb_ = std::move(cb); }
    void set_imu_callback(ImuCallback cb) { imu_cb_ = std::move(cb); }
    void set_frame_interval_ns(uint64_t ns) { frame_interval_ns_ = ns; }

private:
    struct TimedPoint
    {
        double x, y, z;          // m
        uint8_t reflectivity;
        uint8_t tag;
        uint8_t line;
        uint64_t abs_time_ns;
    };

    static void PointCloudCb(uint32_t handle, const uint8_t dev_type,
                             LivoxLidarEthernetPacket *data, void *client_data);
    static void ImuCb(uint32_t handle, const uint8_t dev_type,
                      LivoxLidarEthernetPacket *data, void *client_data);

    void on_point_packet(uint8_t dev_type, const LivoxLidarEthernetPacket *data);
    void on_imu_packet(const LivoxLidarEthernetPacket *data);
    void emit_frame();
    uint64_t packet_timestamp_ns(const LivoxLidarEthernetPacket *data) const;

    std::mutex mtx_;
    std::deque<TimedPoint> buf_;
    uint64_t frame_interval_ns_ = 100000000;
    uint8_t line_num_ = 4;   // kLineNumberMid360
    FrameCallback frame_cb_;
    ImuCallback imu_cb_;
    bool initialized_ = false;
};

#endif
