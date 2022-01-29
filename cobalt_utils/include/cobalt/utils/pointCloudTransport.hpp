#pragma once

#include <ignition/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <ros_ign_bridge/convert.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace cobaltUtils{

    class pointCloudHandler
    {

        typedef sensor_msgs::msg::PointCloud2 pcl2;

        public:
            pointCloudHandler(const std::string& _topic, 
                const std::string& _frame_id,
                const std::shared_ptr<ignition::transport::Node>& _ign_node,
                const std::shared_ptr<rclcpp::Node>& _ros_node) 
            {
                this->frame_id = _frame_id;

                _ign_node->Subscribe(_topic, &pointCloudHandler::OnPcl, this);

                this->pclPub = 
                    _ros_node->create_publisher<pcl2>(_topic, rclcpp::QoS(10));
            }

        private:

            void OnPcl(const ignition::msgs::PointCloudPacked & _ign_msg)
            {
                sensor_msgs::msg::PointCloud2 ros_msg;
                ros_ign_bridge::convert_ign_to_ros(_ign_msg, ros_msg);
                ros_msg.header.frame_id = this->frame_id;
                this->pclPub->publish(ros_msg);
            }

            rclcpp::Publisher<pcl2>::SharedPtr pclPub;

            std::string frame_id;

    };

}