#include "cobalt/utils/robustImageBridge.hpp"

using namespace cobaltUtils;

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    auto node_ = rclcpp::Node::make_shared("ros_ign_image");

    node_->declare_parameter("frame_id", "base_footprint");

    std::string frame_id;

    node_->get_parameter("frame_id", frame_id);

    node_->declare_parameter("topics");

    std::vector<std::string> topics;

    node_->get_parameter("topics", topics);

    auto it_node = std::make_shared<image_transport::ImageTransport>(node_);

    auto ign_node = std::make_shared<ignition::transport::Node>();

    std::vector<std::shared_ptr<Handler>> handlers;

    for (auto topic : topics) {
        
        handlers.push_back(
            std::make_shared<Handler>(
                topic,
                frame_id, 
                it_node, 
                ign_node
            )
        );

    }

    rclcpp::spin(node_);

    ignition::transport::waitForShutdown();

    return 0;

}