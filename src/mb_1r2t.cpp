#include <geometry_msgs/msg/point32.hpp>
#include <mb_1r2t/mb_1r2t.hpp>

MB_1r2t::MB_1r2t()
    : rclcpp::Node("mb_1r2t")
{
    m_timer = create_wall_timer(std::chrono::milliseconds(1),
        std::bind(&MB_1r2t::update, this));

    m_laser_scan_publisher = create_publisher<sensor_msgs::msg::LaserScan>("/laser_scan", 10);
    m_point_cloud_publisher = create_publisher<sensor_msgs::msg::PointCloud>("/point_cloud", 10);

    m_serial_device = std::make_unique<SerialDevice>(*this, "/dev/ttyUSB0");

    m_frame_id = "lidar";

    m_laser_scan_msg.angle_min = ANGLE_MIN;
    m_laser_scan_msg.angle_max = ANGLE_MAX;
    m_laser_scan_msg.angle_increment = ANGLE_INC;
    m_laser_scan_msg.range_min = RANGE_MIN;
    m_laser_scan_msg.range_max = RANGE_MAX;
    m_laser_scan_msg.scan_time = SCAN_TIME;
    m_laser_scan_msg.time_increment = SCAN_TIME / (ANGLE_MAX / ANGLE_INC);

    m_laser_scan_msg.header.frame_id = m_frame_id;
    m_point_cloud_msg.header.frame_id = m_frame_id;

    RCLCPP_INFO(get_logger(), "started");
}

void MB_1r2t::update()
{
    parse_packet();
}

void MB_1r2t::publish_laser_scan()
{
    std::reverse(m_laser_scan_msg.ranges.begin(), m_laser_scan_msg.ranges.end());
    std::reverse(m_laser_scan_msg.intensities.begin(), m_laser_scan_msg.intensities.end());

    m_laser_scan_publisher->publish(m_laser_scan_msg);

    m_laser_scan_msg.ranges.clear();
    m_laser_scan_msg.intensities.clear();
}

void MB_1r2t::publish_point_cloud()
{
    m_point_cloud_msg.header.stamp = now();

    m_point_cloud_publisher->publish(m_point_cloud_msg);

    m_point_cloud_msg.points.clear();
}

void MB_1r2t::parse_packet()
{
    switch (m_state) {
    case SYNC0: {
        if (m_serial_device->read(&m_buffer[0], 1) == false) {
            break;
        }

        if (m_buffer[0] == KSYNC0) {
            m_state = SYNC1;
        }

        break;
    }
    case SYNC1: {
        if (m_serial_device->read(&m_buffer[1], 1) == false) {
            break;
        }

        if (m_buffer[1] == KSYNC1) {
            m_state = HEADER;
        } else {
            m_state = SYNC0;
        }

        break;
    }
    case HEADER: {
        if (m_serial_device->read(&m_buffer[2], 8) == false) {
            m_state = SYNC0;
            break;
        }

        m_package_header.type = m_buffer[2];
        m_package_header.data_length = m_buffer[3];
        m_package_header.start_angle = m_buffer[5] << 8 | m_buffer[4];
        m_package_header.stop_angle = m_buffer[7] << 8 | m_buffer[6];
        m_package_header.crc = m_buffer[9] << 8 | m_buffer[8];

        m_state = DATA;
        break;
    }
    case DATA: {
        uint16_t bytes_to_read = m_package_header.data_length * 3;
        if (m_serial_device->read(&m_buffer[10], bytes_to_read) == false) {
            m_state = SYNC0;
            break;
        }

        if (m_package_header.type & 1) {
            publish_point_cloud();
        }

        // corrupt data
        if (bytes_to_read != 120) {
            m_state = SYNC0;
            break;
        }

        int16_t diff = m_package_header.stop_angle - m_package_header.start_angle;
        if (m_package_header.stop_angle < m_package_header.start_angle) {
            diff = 0xB400 - m_package_header.start_angle + m_package_header.stop_angle;
        }

        int16_t angle_per_sample = 0;
        if (diff > 1) {
            angle_per_sample = diff / (m_package_header.data_length - 1);
        }

        for (int i = 0; i < m_package_header.data_length; ++i) {
            uint16_t index = 10 + (i * 3);
            uint8_t intensity = m_buffer[index + 0];
            uint8_t distance_L = m_buffer[index + 1];
            uint8_t distance_H = m_buffer[index + 2];

            uint16_t distance = (uint16_t)(distance_H << 8) + (uint16_t)distance_L;
            float distancef = (float)distance / 4000.0;

            float step = M_PI * 2;
            float angle = (m_package_header.start_angle + angle_per_sample * i);
            float anglef = (step * (angle / 0xB400));

            if (anglef < m_last_angle) {

                publish_laser_scan();

                m_laser_scan_msg.header.stamp = now();
            }

            m_last_angle = anglef;

            m_laser_scan_msg.ranges.push_back(distancef);
            m_laser_scan_msg.intensities.push_back(intensity);

            geometry_msgs::msg::Point32 point;
            float angle_inv = (M_PI * 2) - anglef;
            point.x = cos(angle_inv) * distancef;
            point.y = sin(angle_inv) * distancef;
            point.z = m_position_z;

            m_point_cloud_msg.points.emplace_back(point);
        }

        m_state = SYNC0;
        break;
    }
    default: {
        RCLCPP_ERROR(get_logger(), "Unknown state: %d", m_state);
        m_state = SYNC0;
    }
    }
}