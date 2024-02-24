# ROS2 Tutorial : Service Definition

## Contents
     - What Are They?
     - Defining A Service

## What Are They?

## Defining A Service

Be sure to source:
```
source /opt/ros/humble/setup.bash
```
and:
```
source ./install/setup.bash
```

1) Create the package:
```
ros2 pkg create --build-type ament_cmake tutorial_service_definition  --dependencies rclcpp
```
2) 
```
cd src/tutorial_service_definition
mkdir srv && cd srv
gedit Haiku.srv
```
3) Add the following:
```
# Request
int64 line_number
---
# Response
string line
```
4) Modify `CMakeLists.txt` by adding these lines before `ament_package()` at the end:
```
find_package(rosidl_default_generators REQUIRED)                                                    # Needed to convert .action files to C++

rosidl_generate_interfaces(${PROJECT_NAME}
                           "srv/Haiku.srv")
```
5) Modify `package.xml` with the following:
```
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```
6) Navigate back to root of workspace and build:
```
colcon build --packages-select tutorial_service_definition
```



