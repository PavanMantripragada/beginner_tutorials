# beginner_tutorials
ROS Humble Publisher/Subscriber Package

## Dependencies
- ROS Humble Installed on PC 
- rclcpp
- std_msgs

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

## Running the code
Open two terminals and run `source install/setup.bash` in both and then run following commands in each terminal.
```
ros2 run beginner_tutorials talker
ros2 run beginner_tutorials listener
```


## Auxilary Information for Developers

## Formating the code
```
cd <your_path_to_repo>/beginner_tutorials
clang-format -style=Google -i ./src/publisher_member_function.cpp  ./src/subscriber_member_function.cpp
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