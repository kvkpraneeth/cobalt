#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace cobaltUtils{

    class odomTotransform : public rclcpp::Node
    {

        public:

            odomTotransform() : Node("odomTotransform")
            {

                this->declare_parameter<std::string>(
                    "OdometryTopic", "odom"
                );
                
                std::string OdomTopic;

                this->get_parameter("OdometryTopic", OdomTopic);

                OdomSub = this->create_subscription<nav_msgs::msg::Odometry>(
                    OdomTopic, 
                    rclcpp::QoS(10),
                    std::bind(
                        &odomTotransform::OdomCB, 
                        this, 
                        std::placeholders::_1)
                );

                TfBroadcaster = 
                    std::make_shared<tf2_ros::TransformBroadcaster>(*this);

            }
        
        private:

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr OdomSub;
            
            void OdomCB(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
            {
                geometry_msgs::msg::TransformStamped TfStamped;
                
                TfStamped.header = msg->header;
                TfStamped.child_frame_id = msg->child_frame_id;

                TfStamped.transform.translation.x = msg->pose.pose.position.x;
                TfStamped.transform.translation.y = msg->pose.pose.position.y;
                TfStamped.transform.translation.z = msg->pose.pose.position.z;

                TfStamped.transform.rotation.x = msg->pose.pose.orientation.x;
                TfStamped.transform.rotation.y = msg->pose.pose.orientation.y;
                TfStamped.transform.rotation.z = msg->pose.pose.orientation.z;
                TfStamped.transform.rotation.w = msg->pose.pose.orientation.w;
            
                TfBroadcaster->sendTransform(TfStamped);
            }

            std::shared_ptr<tf2_ros::TransformBroadcaster> TfBroadcaster;

    }; 

}