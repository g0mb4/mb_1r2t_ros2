#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mb_1r2t/serial_device_linux.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <string>
#include <vector>

class MB_1r2t : public rclcpp::Node {
public:
    MB_1r2t();

private:
    static const size_t DATA_SIZE = 120;

    static constexpr float RANGE_MIN = 0.11;
    static constexpr float RANGE_MAX = 8.0;

    static constexpr float ANGLE_MIN = 0.0;
    static constexpr float ANGLE_MAX = 2 * M_PI;
    static constexpr float ANGLE_INC = ANGLE_MAX / 400.0;

    static constexpr float SCAN_TIME = 0.2;

    enum State {
        SYNC0 = 0,
        SYNC1,
        HEADER,
        DATA
    };

    enum SyncByte {
        SYNC_BYTE0 = 0xAA,
        SYNC_BYTE1 = 0x55,
    };

    enum PacketType {
        SCAN_DONE = 0x91,
        SCAN_DATA = 0x2C
    };

    struct Packet {
        uint8_t sync_0;
        uint8_t sync_1;
        uint8_t type;
        uint8_t data_length;
        uint16_t start_angle;
        uint16_t stop_angle;
        uint16_t crc;
        uint8_t data[DATA_SIZE] = { 0 };
    };

    struct ScanResult {
        float angle;
        float distance;
        float intensity;
    };

    void publish_loop();

    void publish_laser_scan();
    void publish_point_cloud();

    void parse_packet();

    void scan_done();
    void scan_data();

    std::unique_ptr<SerialDevice> m_serial_device;
    rclcpp::TimerBase::SharedPtr m_timer;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_scan_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr m_point_cloud_publisher;

    sensor_msgs::msg::LaserScan m_laser_scan_msg;
    sensor_msgs::msg::PointCloud m_point_cloud_msg;

    std::vector<ScanResult> m_scan_results;

    State m_state { SYNC0 };
    Packet m_packet = {};
    std::string m_frame_id;
    float m_position_z = 0;
};