#pragma once

#include <ignition/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <ros_ign_bridge/convert.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace cobaltUtils{

    class Handler
    {
        public:

            Handler(const std::string & _topic,
                const std::string & _frame_id,
                std::shared_ptr<image_transport::ImageTransport>& _it_node, 
                std::shared_ptr<ignition::transport::Node>& _ign_node)
            {
                this->ros_pub = _it_node->advertise(_topic, 1);
                this->frame_id=_frame_id;
                _ign_node->Subscribe(_topic, &Handler::OnImage, this);
            }

        private:

            void OnImage(const ignition::msgs::Image & _ign_msg)
            {
                sensor_msgs::msg::Image ros_msg;
                ros_ign_bridge::convert_ign_to_ros(_ign_msg, ros_msg);
                ros_msg.header.frame_id = this->frame_id;
                this->ros_pub.publish(ros_msg);
            }

            image_transport::Publisher ros_pub;

            std::string frame_id;

    };
    
}