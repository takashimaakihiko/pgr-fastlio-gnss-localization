#include "lidar_source.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>

namespace
{
constexpr uint8_t kTimestampTypeNoSync = 0;
constexpr uint64_t kNsPerSecond = 1000000000;
} // namespace

Mid360Source::~Mid360Source()
{
    shutdown();
}

bool Mid360Source::init(const std::string &sdk_config_path)
{
    if (initialized_)
    {
        return true;
    }
    if (!LivoxLidarSdkInit(sdk_config_path.c_str()))
    {
        std::cerr << "[mid360] LivoxLidarSdkInit failed (config: "
                  << sdk_config_path << ")" << std::endl;
        return false;
    }
    SetLivoxLidarPointCloudCallBack(&Mid360Source::PointCloudCb, this);
    SetLivoxLidarImuDataCallback(&Mid360Source::ImuCb, this);
    initialized_ = true;
    std::cout << "[mid360] SDK initialized" << std::endl;
    return true;
}

void Mid360Source::shutdown()
{
    if (initialized_)
    {
        LivoxLidarSdkUninit();
        initialized_ = false;
    }
}

void Mid360Source::PointCloudCb(uint32_t handle, const uint8_t dev_type,
                                LivoxLidarEthernetPacket *data, void *client_data)
{
    auto *self = static_cast<Mid360Source *>(client_data);
    if (self == nullptr || data == nullptr)
    {
        return;
    }
    self->on_point_packet(dev_type, data);
}

void Mid360Source::ImuCb(uint32_t handle, const uint8_t dev_type,
                         LivoxLidarEthernetPacket *data, void *client_data)
{
    auto *self = static_cast<Mid360Source *>(client_data);
    if (self == nullptr || data == nullptr)
    {
        return;
    }
    self->on_imu_packet(data);
}

uint64_t Mid360Source::packet_timestamp_ns(const LivoxLidarEthernetPacket *data) const
{
    if (data->time_type == kTimestampTypeNoSync)
    {
        return static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count());
    }
    uint64_t stamp = 0;
    std::memcpy(&stamp, data->timestamp, sizeof(stamp));
    return stamp;
}

void Mid360Source::diag_tick(const LivoxLidarEthernetPacket *data, bool is_imu)
{
    const auto now = std::chrono::steady_clock::now();
    if (now - diag_last_log_ < std::chrono::seconds(2))
    {
        return;
    }
    diag_last_log_ = now;
    std::cerr << "[mid360] pkts point=" << diag_point_packets_
              << " (pts=" << diag_points_ << ")"
              << " imu=" << diag_imu_packets_
              << " frames=" << diag_frames_
              << " | last pkt type=" << int(data->data_type)
              << " dot_num=" << data->dot_num
              << " time_type=" << int(data->time_type)
              << " t_int=" << data->time_interval
              << (is_imu ? " [imu]" : " [pts]") << std::endl;
}

void Mid360Source::on_point_packet(uint8_t dev_type,
                                   const LivoxLidarEthernetPacket *data)
{
    if (data->dot_num == 0)
    {
        return;
    }
    diag_point_packets_++;
    diag_points_ += data->dot_num;
    diag_tick(data, false);

    const uint64_t pkt_ts_ns = packet_timestamp_ns(data);
    // official driver: point_interval = time_interval * 100 / dot_num (ns)
    const uint64_t interval_ns =
        static_cast<uint64_t>(data->time_interval) * 100 / data->dot_num;

    std::vector<TimedPoint> new_points;
    new_points.reserve(data->dot_num);

    if (data->data_type == kLivoxLidarCartesianCoordinateHighData)
    {
        const auto *raw =
            reinterpret_cast<const LivoxLidarCartesianHighRawPoint *>(data->data);
        for (uint32_t i = 0; i < data->dot_num; ++i)
        {
            TimedPoint p;
            p.x = raw[i].x / 1000.0;
            p.y = raw[i].y / 1000.0;
            p.z = raw[i].z / 1000.0;
            p.reflectivity = raw[i].reflectivity;
            p.tag = raw[i].tag;
            p.line = static_cast<uint8_t>(i % line_num_);
            p.abs_time_ns = pkt_ts_ns + i * interval_ns;
            new_points.push_back(p);
        }
    }
    else if (data->data_type == kLivoxLidarCartesianCoordinateLowData)
    {
        const auto *raw =
            reinterpret_cast<const LivoxLidarCartesianLowRawPoint *>(data->data);
        for (uint32_t i = 0; i < data->dot_num; ++i)
        {
            TimedPoint p;
            p.x = raw[i].x / 100.0;
            p.y = raw[i].y / 100.0;
            p.z = raw[i].z / 100.0;
            p.reflectivity = raw[i].reflectivity;
            p.tag = raw[i].tag;
            p.line = static_cast<uint8_t>(i % line_num_);
            p.abs_time_ns = pkt_ts_ns + i * interval_ns;
            new_points.push_back(p);
        }
    }
    else
    {
        // spherical / double-echo formats are not used by MID360 default config
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mtx_);
        for (const auto &p : new_points)
        {
            buf_.push_back(p);
        }
        if (buf_.size() >= 2 &&
            buf_.back().abs_time_ns - buf_.front().abs_time_ns >= frame_interval_ns_)
        {
            emit_frame();
        }
    }
}

void Mid360Source::emit_frame()
{
    // caller must hold mtx_
    custom_messages::CustomMsg msg;
    const uint64_t base_ns = buf_.front().abs_time_ns;

    msg.header.stamp.fromSec(static_cast<double>(base_ns) / kNsPerSecond);
    msg.header.frame_id = "livox_frame";
    msg.header.seq = 0;
    msg.timebase = base_ns;
    msg.lidar_id = 0;
    for (int i = 0; i < 3; ++i)
    {
        msg.rsvd[i] = 0;
    }
    msg.point_num = buf_.size();
    msg.points.reserve(buf_.size());

    for (const auto &p : buf_)
    {
        custom_messages::CustomPoint cp;
        cp.offset_time = p.abs_time_ns - base_ns;
        cp.x = p.x;
        cp.y = p.y;
        cp.z = p.z;
        cp.reflectivity = p.reflectivity;
        cp.tag = p.tag;
        cp.line = p.line;
        msg.points.push_back(cp);
    }
    buf_.clear();
    diag_frames_++;

    if (frame_cb_)
    {
        frame_cb_(msg);
    }
}

void Mid360Source::on_imu_packet(const LivoxLidarEthernetPacket *data)
{
    if (data->data_type != kLivoxLidarImuData || data->dot_num == 0)
    {
        return;
    }
    diag_imu_packets_++;
    diag_tick(data, true);

    const uint64_t pkt_ts_ns = packet_timestamp_ns(data);
    const uint64_t interval_ns =
        static_cast<uint64_t>(data->time_interval) * 100 / data->dot_num;
    const auto *raw =
        reinterpret_cast<const LivoxLidarImuRawPoint *>(data->data);

    for (uint32_t i = 0; i < data->dot_num; ++i)
    {
        custom_messages::Imu msg;
        msg.header.stamp.fromSec(
            static_cast<double>(pkt_ts_ns + i * interval_ns) / kNsPerSecond);
        msg.header.frame_id = "livox_frame";
        msg.header.seq = 0;
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        for (int k = 0; k < 9; ++k)
        {
            msg.orientation_covariance[k] = 0.0;
            msg.angular_velocity_covariance[k] = 0.0;
            msg.linear_acceleration_covariance[k] = 0.0;
        }
        // MID360 IMU units: gyro rad/s, acc m/s^2
        msg.angular_velocity.x = raw[i].gyro_x;
        msg.angular_velocity.y = raw[i].gyro_y;
        msg.angular_velocity.z = raw[i].gyro_z;
        msg.linear_acceleration.x = raw[i].acc_x;
        msg.linear_acceleration.y = raw[i].acc_y;
        msg.linear_acceleration.z = raw[i].acc_z;

        if (imu_cb_)
        {
            imu_cb_(msg);
        }
    }
}
