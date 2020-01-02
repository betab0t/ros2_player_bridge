#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <libplayerc/playerc.h>
#include <math.h>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sensors_node");

    std::string player_server_ip("127.0.0.1");
    if (argc > 1)
        player_server_ip = argv[1];

    playerc_client_t *playerc_client = playerc_client_create(NULL, player_server_ip.c_str(), 6665);
    if (playerc_client_connect(playerc_client) == 0)
    {
        RCLCPP_INFO(node->get_logger(), "Successfully connected to Player server!");

        auto robot_laser = playerc_laser_create(playerc_client, 0);
        auto robot_position2d = playerc_position2d_create(playerc_client, 0);

        if (playerc_laser_subscribe(robot_laser, PLAYER_OPEN_MODE) == 0 &&
            playerc_position2d_subscribe(robot_position2d, PLAYER_OPEN_MODE) == 0)
        {
            RCLCPP_INFO(node->get_logger(), "Successfully subscribed all Player topics!");

            auto laser_publisher = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(10));

            sensor_msgs::msg::LaserScan laser_msg;
            laser_msg.header.frame_id = "laser_link";
            laser_msg.ranges.resize(360);
            laser_msg.time_increment = static_cast<float>((1/24)/360);
            laser_msg.angle_increment = static_cast<float>((2 * M_PI) / 360);
            laser_msg.angle_min = static_cast<float>(-M_PI);
            laser_msg.angle_max = static_cast<float>(M_PI);
            laser_msg.range_min = 0.0;
            laser_msg.range_max = 10.0;

            tf2_ros::TransformBroadcaster transform_broadcaster(node);
            tf2::Quaternion q;
            geometry_msgs::msg::TransformStamped odom_tf_msg;

            odom_tf_msg.header.frame_id = "odom";
            odom_tf_msg.child_frame_id = "base_link";

            auto odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";
            for (unsigned int i = 0; i < odom_msg.pose.covariance.size(); ++i) {
                odom_msg.pose.covariance[i] = 0.0;
            }

            rclcpp::Rate rate(10); // 10hz
            rclcpp::Time now;

            while (rclcpp::ok())
            {
                playerc_client_read(playerc_client);
                for(int i = robot_laser->scan_count - 1; i >= 0; i--)
                {
                    laser_msg.ranges[(robot_laser->scan_count - 1) - i] = (float)robot_laser->ranges[i];
                }
                now = node->now();
                laser_msg.header.stamp = now;
                laser_publisher->publish(laser_msg);

                q.setRPY(0.0, 0.0, robot_position2d->pa);

                odom_msg.header.stamp = now;
                odom_msg.pose.pose.position.x = robot_position2d->px;
                odom_msg.pose.pose.position.y = robot_position2d->py;
                odom_msg.pose.pose.position.z = 0.0;
                odom_msg.pose.pose.orientation.x = q.x();
                odom_msg.pose.pose.orientation.y = q.y();
                odom_msg.pose.pose.orientation.z = q.z();
                odom_msg.pose.pose.orientation.w = q.w();
                odom_msg.twist.twist.linear.x = robot_position2d->vx;
                odom_msg.twist.twist.linear.y = robot_position2d->vy;
                odom_msg.twist.twist.linear.z = 0.0;
                odom_msg.twist.twist.angular.x = 0.0;
                odom_msg.twist.twist.angular.y = 0.0;
                odom_msg.twist.twist.angular.z = robot_position2d->va;

                odom_publisher->publish(odom_msg);

                odom_tf_msg.header.stamp = now;
                odom_tf_msg.transform.translation.x = robot_position2d->px;
                odom_tf_msg.transform.translation.y = robot_position2d->py;
                odom_tf_msg.transform.translation.z = 0.0;
                odom_tf_msg.transform.rotation.x = q.x();
                odom_tf_msg.transform.rotation.y = q.y();
                odom_tf_msg.transform.rotation.z = q.z();
                odom_tf_msg.transform.rotation.w = q.w();
                transform_broadcaster.sendTransform(odom_tf_msg);

                rate.sleep();
            }

            playerc_client_disconnect(playerc_client);
        }
    }

    rclcpp::shutdown();
    return 0;
}