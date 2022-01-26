#include "cobalt/utils/odomTotransform.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cobaltUtils::odomTotransform>());
    rclcpp::shutdown();
    return 0;
}