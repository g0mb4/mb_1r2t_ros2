#include <mb_1r2t/mb_1r2t.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<MB_1r2t>());

    rclcpp::shutdown();
    return 0;
}