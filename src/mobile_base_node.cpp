#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <libplayerc/playerc.h>

playerc_position2d_t *robot_position2d;

void cmd_vel_topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    playerc_position2d_set_cmd_vel(robot_position2d, msg->linear.x, msg->linear.y, msg->angular.z, 1);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mobile_base_node");

    std::string player_server_ip("127.0.0.1");
    if (argc > 1)
        player_server_ip = argv[1];

    playerc_client_t *playerc_client = playerc_client_create(NULL, player_server_ip.c_str(), 6665);
    if (playerc_client_connect(playerc_client) == 0)
    {
        std::cout << "Successfully connected to Player server!";

        robot_position2d = playerc_position2d_create(playerc_client, 0);
        if (playerc_position2d_subscribe(robot_position2d, PLAYER_OPEN_MODE) == 0)
        {
            RCLCPP_INFO(node->get_logger(), "Successfully subscribed to position!");

            auto subscription = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10), cmd_vel_topic_callback);
            rclcpp::spin(node);
            playerc_client_disconnect(playerc_client);
        }
    }
    rclcpp::shutdown();
    return 0;
}