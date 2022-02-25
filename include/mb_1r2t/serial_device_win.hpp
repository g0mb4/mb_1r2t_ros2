#pragma once

#include <rclcpp/rclcpp.hpp>

#include <stdint.h>

class SerialDevice {
public:
    SerialDevice(rclcpp::Node& node, const std::string& port);
    ~SerialDevice();

    bool read(uint8_t* buffer, size_t len);

private:
    static const uint32_t BAUDRATE = 153600;

    rclcpp::Node& m_node;
};