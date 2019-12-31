#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <libplayerc/playerc.h>
#include <math.h>

int main(int argc, char *argv[]) {
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
        if (playerc_laser_subscribe(robot_laser, PLAYER_OPEN_MODE) == 0)
        {
            RCLCPP_INFO(node->get_logger(), "Successfully subscribed to laser!");

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

            rclcpp::Rate rate(10); // 10hz

            while (rclcpp::ok())
            {
                playerc_client_read(playerc_client);
                for(int i = robot_laser->scan_count - 1; i >= 0; i--)
                {
                    laser_msg.ranges[(robot_laser->scan_count - 1) - i] = (float)robot_laser->ranges[i];
                }
                laser_msg.header.stamp = node->now();
                laser_publisher->publish(laser_msg);
                rate.sleep();
            }

            playerc_client_disconnect(playerc_client);
        }
    }

    rclcpp::shutdown();
    return 0;
}