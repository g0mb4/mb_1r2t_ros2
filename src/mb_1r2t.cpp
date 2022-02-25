#include <mb_1r2t/mb_1r2t.hpp>

#include <geometry_msgs/msg/point32.hpp>

#include <algorithm>

MB_1r2t::MB_1r2t()
    : rclcpp::Node("mb_1r2t_node")
{
    m_timer = create_wall_timer(std::chrono::milliseconds(1),
        std::bind(&MB_1r2t::publish_loop, this));

    m_laser_scan_publisher = create_publisher<sensor_msgs::msg::LaserScan>("/laser_scan", 10);
    m_point_cloud_publisher = create_publisher<sensor_msgs::msg::PointCloud>("/point_cloud", 10);

    m_serial_device = std::make_unique<SerialDevice>(*this, "/dev/ttyUSB0");

    m_frame_id = "lidar";

    m_laser_scan_msg.range_min = RANGE_MIN;
    m_laser_scan_msg.range_max = RANGE_MAX;
    m_laser_scan_msg.scan_time = SCAN_TIME;
    m_laser_scan_msg.time_increment = SCAN_TIME / (ANGLE_MAX / ANGLE_INC);

    m_laser_scan_msg.header.frame_id = m_frame_id;
    m_point_cloud_msg.header.frame_id = m_frame_id;

    RCLCPP_INFO(get_logger(), "started");
}

void MB_1r2t::publish_loop()
{
    parse_packet();
}

void MB_1r2t::publish_laser_scan()
{
    std::sort(m_scan_results.begin(), m_scan_results.end(), [](const ScanResult& a, const ScanResult& b) {
        return a.angle < b.angle;
    });

    float avg_increment = 0;
    float min_angle = m_scan_results[0].angle;
    float max_angle = m_scan_results[m_scan_results.size() - 1].angle;

    for (size_t i = 1; i < m_scan_results.size(); ++i) {
        avg_increment += m_scan_results[i].angle - m_scan_results[i - 1].angle;
    }
    avg_increment /= (float)(m_scan_results.size() - 1);

    m_laser_scan_msg.angle_min = min_angle;
    m_laser_scan_msg.angle_max = max_angle;
    m_laser_scan_msg.angle_increment = avg_increment;

    for (ScanResult& s : m_scan_results) {
        m_laser_scan_msg.ranges.emplace_back(s.distance);
        m_laser_scan_msg.intensities.emplace_back(s.distance);
    }

    m_laser_scan_publisher->publish(m_laser_scan_msg);

    m_scan_results.clear();
    m_laser_scan_msg.ranges.clear();
    m_laser_scan_msg.intensities.clear();
}

void MB_1r2t::publish_point_cloud()
{
    m_point_cloud_msg.header.stamp = now();

    m_point_cloud_publisher->publish(m_point_cloud_msg);

    m_point_cloud_msg.points.clear();
}

void MB_1r2t::scan_done()
{
    publish_point_cloud();
    publish_laser_scan();

    m_laser_scan_msg.header.stamp = now();
}

void MB_1r2t::scan_data()
{
    // TODO: search datasheet to verify this
    int16_t diff = m_packet.stop_angle - m_packet.start_angle;
    if (m_packet.stop_angle < m_packet.start_angle) {
        diff = 0xB400 - m_packet.start_angle + m_packet.stop_angle;
    }

    int16_t angle_per_sample = 0;
    if (diff > 1) {
        angle_per_sample = diff / (m_packet.data_length - 1);
    }

    for (int i = 0; i < m_packet.data_length; ++i) {
        uint16_t index = i * 3;
        uint8_t intensity = m_packet.data[index + 0];
        uint8_t distance_L = m_packet.data[index + 1];
        uint8_t distance_H = m_packet.data[index + 2];

        uint16_t distance = (uint16_t)(distance_H << 8) + (uint16_t)distance_L;
        float distancef = (float)distance / 4000.0;

        float step = M_PI * 2;
        float angle = (m_packet.start_angle + angle_per_sample * i);
        float anglef = (step * (angle / 0xB400));
        float angle_inv = (M_PI * 2) - anglef;

        ScanResult result;
        result.angle = angle_inv;
        result.distance = distancef;
        result.intensity = intensity;

        m_scan_results.emplace_back(result);

        geometry_msgs::msg::Point32 point;
        point.x = cos(angle_inv) * distancef;
        point.y = sin(angle_inv) * distancef;
        point.z = m_position_z;

        m_point_cloud_msg.points.emplace_back(point);
    }
}

void MB_1r2t::parse_packet()
{
    switch (m_state) {
    case SYNC0:
        if (m_serial_device->read(&m_packet.sync_0, 1) == false) {
            break;
        }

        if (m_packet.sync_0 == SYNC_BYTE0) {
            m_state = SYNC1;
        }

        break;

    case SYNC1:
        if (m_serial_device->read(&m_packet.sync_1, 1) == false) {
            break;
        }

        if (m_packet.sync_1 == SYNC_BYTE1) {
            m_state = HEADER;
        } else {
            m_state = SYNC0;
        }

        break;

    case HEADER:
        if (m_serial_device->read(m_packet.data, 8) == false) {
            m_state = SYNC0;
            break;
        }

        m_packet.type = m_packet.data[0];
        m_packet.data_length = m_packet.data[1];
        m_packet.start_angle = m_packet.data[3] << 8 | m_packet.data[2];
        m_packet.stop_angle = m_packet.data[5] << 8 | m_packet.data[4];
        m_packet.crc = m_packet.data[7] << 8 | m_packet.data[6];

        m_state = DATA;
        break;

    case DATA: {
        uint16_t bytes_to_read = m_packet.data_length * 3;

        // invalid data
        if (bytes_to_read > DATA_SIZE) {
            m_state = SYNC0;
            break;
        }

        if (m_serial_device->read(m_packet.data, bytes_to_read) == false) {
            m_state = SYNC0;
            break;
        }

        if (m_packet.type == SCAN_DONE) {
            scan_done();
        } else if (m_packet.type == SCAN_DATA) {
            scan_data();
        } else {
            RCLCPP_ERROR(get_logger(), "Unknown packet type: %02X", m_packet.type);
        }

        m_state = SYNC0;
        break;
    }
    default:
        RCLCPP_ERROR(get_logger(), "Unknown state: %d", m_state);
        m_state = SYNC0;
    }
}
