# ros2_player_bridge
## Build
```sh
colcon build --symlink-install
source install/setup.bash
```

## Run
```sh
ros2 launch ros2_player_bridge ros2_player_bridge.launch.py player_ip:=<IP_OPTIONAL>
```

## Topics
* /cmd_vel -> Twist
* /scan -> LaserScan