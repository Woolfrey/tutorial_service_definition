# ROS2 Tutorial : Service Definition

This is part 2 in a series:

1. Publishers & Subscribers
2. Services : Defining a Custom Service
3. Services & Clients
4. Actions: Defining a Custom Action
5. Action Servers and Action Clients

## Contents
- What Are They?
- Defining A Service

## What Are They?

A `service` in ROS2 is a structured message type that defines a `request` data type, and a `return` data type. It is contained in a `.srv` file with the following structure:
```
# Request
datatype request_name
---
# Response
datatype response_name
```
This defines how a `client` node can communicate with a `service` node. The `client` makes a request of the specified data type. The `client` does something with that information, and returns the `reponse` data type.

A `service` differs from the `publisher` and `subscriber` paradigm in that the latter is suited to streaming data. A `publisher` node will make data publicly available on the `ROS2` network that any number of `subscribers` may access and make use of. In contrast, a `service` is a direction communication between the `server` and `client`.

We can think of the `publisher` and `subscriber` model like a newsagency, where different magazines and journals are made publicly available that customers can subscribe to based on relevant interests.

The `service` and `client` model is more like sending a letter directly to someone, who will then write back directly to you with the requested information.

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
7) Check:
```
ros2 interface show tutorial_service_definition/srv/Haiku
```
You should see:
```
# Request
int64 line_number
---
# Response
string line
```



