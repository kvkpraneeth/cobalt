#include "cobalt/utils/pointCloudTransport.hpp"

using namespace cobaltUtils;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto ign_node = std::make_shared<ignition::transport::Node>();

    auto ros_node = rclcpp::Node::make_shared("ros_ign_pcl");

    ros_node->declare_parameter("topics");
    std::vector<std::string> topics;
    ros_node->get_parameter("topics", topics);

    std::string frame_id;
    ros_node->declare_parameter("frame_id", "base_footprint");
    ros_node->get_parameter("frame_id", frame_id);

    std::vector<std::shared_ptr<pointCloudHandler>> pclHandles;

    for(auto topic : topics){

        pclHandles.push_back(
            std::make_shared<pointCloudHandler>(
                topic,
                frame_id,
                ign_node,
                ros_node
            )
        );

    }

    rclcpp::spin(ros_node);

    ignition::transport::waitForShutdown();

    return 0;
}