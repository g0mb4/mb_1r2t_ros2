#include <mb_1r2t/serial_device_linux.hpp>

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termio.h>
#include <unistd.h>

SerialDevice::SerialDevice(rclcpp::Node& node, const std::string& port)
    : m_node(node)
{
    m_fd = open(port.c_str(), O_RDWR | O_NOCTTY);

    if (m_fd < 0) {
        RCLCPP_FATAL(m_node.get_logger(), "Unable to open `%s`: %s", port.c_str(), strerror(errno));
        exit(1);
    }

    struct termios tty = {};

    tty.c_cflag = B38400; // placeholder
    tty.c_cflag |= CLOCAL | CREAD; // non-modem device, enable reading
    tty.c_cflag &= ~CSIZE; // unset number of bits per character
    tty.c_cflag |= CS8; // 8-bit characters
    tty.c_cflag &= ~PARENB; // no parity bit
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CRTSCTS; // no hardware flowcontrol

    tty.c_lflag = ~(ICANON); // non-canonical mode
    tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | IEXTEN); // no echo

    /* non-canonical mode */
    tty.c_iflag = 0;
    tty.c_oflag = 0;

    if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
        RCLCPP_FATAL(m_node.get_logger(), "Unable to apply serial settings: ", strerror(errno));
        exit(1);
    }

    /* to achieve the custom high baudrate the following steps required:
        struct termios2 options;
        ioctl(fd, TCGETS2, &options);
        options.c_cflag &= ~CBAUD;
        options.c_cflag |= BOTHER;
        options.c_ispeed = BAUDRATE;
        options.c_ospeed = BAUDRATE;
        ioctl(fd, TCSETS2, &options);

       but this uses termios2 from asm/termios.h, whis is painful to include
       the workaround:
    */

#define TCGETS2_ 0x802C542A
#define TCSETS2_ 0x402C542B
#define BROTHER_ 0x1000

    int buf[64] = { 0 };
    if ((ioctl(m_fd, TCGETS2_, buf)) < 0) {
        RCLCPP_FATAL(m_node.get_logger(), "Unable to get struct termios2: ", strerror(errno));
        exit(1);
    }

    buf[2] &= ~CBAUD; // remove current baud rate
    buf[2] |= BROTHER_; // allow custom baud rate using int input
    buf[9] = BAUDRATE;
    buf[10] = BAUDRATE;

    if ((ioctl(m_fd, TCSETS2_, buf)) < 0) {
        RCLCPP_FATAL(m_node.get_logger(), "Unable to set struct termios2: ", strerror(errno));
        exit(1);
    }

#undef TCGETS2_
#undef TCSETS2_
#undef BROTHER_
}

SerialDevice::~SerialDevice()
{
    if (m_fd >= 0) {
        close(m_fd);
    }
}

bool SerialDevice::read(uint8_t* buffer, size_t len)
{
    int n = ::read(m_fd, buffer, len);

    /* too noisy
    if (n == -1) {
        RCLCPP_ERROR(m_node.get_logger(), "Read error: %s", strerror(errno));
    } else if (n != (int)len) {
        RCLCPP_ERROR(m_node.get_logger(), "Read error: expected %d bytes, got %d bytes.", len, n);
    }
    */

    return n == (int)len;
}