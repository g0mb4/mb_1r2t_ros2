#include <mb_1r2t/serial_device_win.hpp>

SerialDevice::SerialDevice(rclcpp::Node& node, const std::string& port)
    : m_node(node)
{
    (void)port;

    RCLCPP_FATAL(m_node.get_logger(), "Windows is not supported yet.");
    exit(1);
}

SerialDevice::~SerialDevice()
{
}

bool SerialDevice::read(uint8_t* buffer, size_t len)
{
    (void)buffer;
    (void)len;

    return false;
}