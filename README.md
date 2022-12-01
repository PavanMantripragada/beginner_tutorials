# beginner_tutorials
ROS Humble Publisher/Subscriber Package

## Dependencies
- ROS Humble Installed on PC 
- rclcpp
- std_msgs
- example_interfaces

## Creating a ros workspace
```
mkdir ros2_ws/src
```

## Cloning the package
```
cd <your_path>/ros2_ws/src
git clone https://github.com/PavanMantripragada/beginner_tutorials.git
```

## Building the package
```
cd <your_path>/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select beginner_tutorials
```

## Testing the package
```
cd <your_path>/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select beginner_tutorials
source install/setup.bash
colcon test --event-handlers console_direct+ --packages-select beginner_tutorials
```

## For Checking tf
Source the workspace by running `source install/setup.bash` in all terminals.

### To start static tf publisher
```
ros2 run beginner_tutorials talker
```
### To see the transform on terminal
```
ros2 run tf2_ros tf2_echo world talk
```
### To store the tf tree
```
cd <your_path_to_repo>/beginner_tutorials/results
ros2 run tf2_tools view_frames
```
### To record a bag file
```
cd <your_path_to_repo>/beginner_tutorials/launch
ros2 launch beginner_tutorials rosbag_record_launch.xml bag_record:=1
```
If the argument `bag_record` is set to `0` it won't record. To terminate press `Ctrl+C` on the terminal.
### To see info of recorded bag file
```
cd <your_path_to_repo>/beginner_tutorials/results
ros2 bag info all_topics
```
### To play the recorded bag file and see
Run `ros2 run beginner_tutorials listener` in a seperate terminal and then
```
cd <your_path_to_repo>/beginner_tutorials/results
ros2 bag play all_topics
```

## For Running Publisher/Subscriber via Message changing service
Open three terminals and run `source install/setup.bash` in all and then run following commands in each terminal. The last client will change the message being published to sum of the two ints given as input. All of these can be launched together using the last launch command.
```
ros2 run beginner_tutorials talker
ros2 run beginner_tutorials listener
ros2 run beginner_tutorials client --ros-args -p A:=1 -p B:=2
ros2 launch beginner_tutorials client launch_everything.yaml A:=8 B:=8
```
For debug options launch all nodes as follows
```
ros2 run beginner_tutorials talker --ros-args --log-level debug
ros2 run beginner_tutorials listener --ros-args --log-level debug
ros2 run beginner_tutorials client --ros-args --log-level debug -p A:=1 -p B:=2
```

## Auxilary Information for Developers

## Formating the code
```
cd <your_path_to_repo>/beginner_tutorials
clang-format -style=Google -i ./src/publisher_member_function.cpp  ./src/subscriber_member_function.cpp ./src/message_change_client.cpp ./src/publisher_main.cpp ./test/publisher_test.cpp ./include/beginner_tutorials/publisher_member_function.hpp
```

## Running cppcheck
```
cd <path to repository>
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingInclude $( find . -name \*.hpp -or -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/")
```

## Running cpplint
```
cd <path to repository>
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```