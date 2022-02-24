#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mb_1r2t/serial_device_linux.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <string>
#include <vector>

class MB_1r2t : public rclcpp::Node {
public:
    static const size_t BUFFER_SIZE = 160;

    MB_1r2t();

private:
    static const uint8_t KSYNC0 = 0xAA;
    static const uint8_t KSYNC1 = 0x55;

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

    struct PackageHeader {
        uint8_t type;
        uint8_t data_length;
        uint16_t start_angle;
        uint16_t stop_angle;
        uint16_t crc;
    };

    struct ScanResult {
        float angle;
        float distance;
        float intensity;
    };

    void update();

    void publish_laser_scan();
    void publish_point_cloud();

    void parse_packet();

    std::unique_ptr<SerialDevice> m_serial_device;
    rclcpp::TimerBase::SharedPtr m_timer;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_scan_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr m_point_cloud_publisher;

    sensor_msgs::msg::LaserScan m_laser_scan_msg;
    sensor_msgs::msg::PointCloud m_point_cloud_msg;

    std::vector<ScanResult> m_scan_results;

    uint8_t m_buffer[BUFFER_SIZE] = { 0 };
    State m_state { SYNC0 };
    PackageHeader m_package_header = {};
    std::string m_frame_id;
    float m_position_z = 0;
};