#By Akhrorov Temurbek [12204574]
#Smart Mobility Engineering Lab.
###BEginner: CLI Tools#### 
#Part1 - Configuring Environment

# Source the setup files:
source /opt/ros/foxy/setup.bash

#Add sourcing to your shell startup script
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

#Check environment variables:
printenv | grep -i ROS

#Set the ROS_DOMAIN_ID variable:
export ROS_DOMAIN_ID=<21>
echo "export ROS_DOMAIN_ID=<21>" >> ~/.bashrc

#Set the ROS_LOCALHOST_ONLY variable:
export ROS_LOCALHOST_ONLY=1:
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc


#Part2 - Using turlesim,ros2, and rqt


# Start by sourcing your setup files in a new terminal, as described in the previous tutorial.
# Install the turtlesim package for your ROS 2 distro.
sudo apt update
sudo apt install ros-foxy-turtlesim
# Check that the package is installed.
ros2 pkg executables turtlesim

# To start turtlesim, enter the following command in your terminal:
ros2 run turtlesim turtlesim_node

# Open a new terminal and source ROS 2 again.
# Now you will run a new node to control the turtle in the first node.
ros2 run turtlesim turtle_teleop_key

# Open a new terminal to install rqt and its plugins.
sudo apt update
sudo apt install ~nros-foxy-rqt*
# To run rqt:
rqt

# When running rqt for the first time, the window will be blank.
# Select Plugins > Services > Service Caller from the menu bar at the top.

# Call the /spawn service, which creates a new turtle in the turtlesim window. Specify the new turtle's name and coordinates.

# Use the /set_pen service to change the pen color and width of a turtle in the turtlesim window.

# You need a second teleop node to control turtle2.
# Remap the cmd_vel topic to control turtle2 separately.
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel

#Part3 - Understanding nodes


# To run turtlesim:
ros2 run turtlesim turtlesim_node

# ros2 node list will show you the names of all running nodes.
ros2 node list

# Remapping allows you to reassign default node properties, like node name.
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

# To examine your latest node, my_turtle, run the following command:
ros2 node info /my_turtle

#Part4 - Understanding topics


# Starting up turtlesim
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

# Running rqt_graph to visualize nodes and topics
rqt_graph

# Listing all active topics
ros2 topic list

# Echoing data from a topic
ros2 topic echo /turtle1/cmd_vel

# Getting information about a topic
ros2 topic info /turtle1/cmd_vel

# Displaying the message structure of a topic
ros2 interface show geometry_msgs/msg/Twist

# Publishing data to a topic (twist and circular movement)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# Viewing the publishing rate of a topic
ros2 topic hz /turtle1/pose

# Stopping the running nodes


#Part5 - Understanding services


# Starting up the turtlesim nodes
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

# Listing all active services
ros2 service list

# Finding the type of a service
ros2 service type /clear

# Listing services with their types
ros2 service list -t

# Finding services of a specific type
ros2 service find std_srvs/srv/Empty

# Displaying the service message structure
ros2 interface show std_srvs/srv/Empty.srv

# Calling a service
ros2 service call /clear std_srvs/srv/Empty

# Calling a service with arguments
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"


#Part6- Understanding Parameters


# Starting up the turtlesim nodes
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

# Listing parameters belonging to nodes
ros2 param list

# Getting the value of a parameter
ros2 param get /turtlesim background_g

# Setting a parameter to a new value
ros2 param set /turtlesim background_r 150

# Saving parameters to a file
ros2 param dump /turtlesim

# Loading parameters from a file
ros2 param load /turtlesim ./turtlesim.yaml

# Starting the turtlesim node with loaded parameters
ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml


#Part7 - Understanding actions


# Start turtlesim nodes
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

# Press keys to control turtle orientation
#For ex. "Q" for rotation

# Get information about a node's actions
ros2 node info /turtlesim

# List available actions in the ROS graph
ros2 action list

# Get detailed information about an action
ros2 action info /turtle1/rotate_absolute

# Display the structure of an action's messages
ros2 interface show turtlesim/action/RotateAbsolute

# Send an action goal to rotate the turtle
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"

# Send an action goal with feedback
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback


#Part8 - Using rqt_console to view logs


# Start the rqt_console tool to view log messages
ros2 run rqt_console rqt_console

# Publish a linear velocity command to make the turtle move
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"

#This section provides information about the various logger levels in ROS 2, including Fatal, Error, Warn, Info, and Debug. 
#These levels indicate the severity of log messages, with Fatal being the most severe and Debug providing detailed debugging information.

#Set the default logger level for the /turtlesim node
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN


#Part9 - Launching nodes


#Open a new terminal and run the launch file
ros2 launch turtlesim multisim.launch.py

# turtlesim/launch/multisim.launch.py
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "turtlesim1", package='turtlesim', executable='turtlesim_node', output='screen'),
        launch_ros.actions.Node(
            namespace= "turtlesim2", package='turtlesim', executable='turtlesim_node', output='screen'),
    ])

#Control the first turtlesim node
ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

#Control the second turtlesim node
ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"


#Part10 - Recording and playing back data


#Start turtlesim and teleop_turtle nodes
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

#Create a directory for saving bag files
mkdir bag_files
cd bag_files

#List available topics
ros2 topic list

#View data from the /turtle1/cmd_vel topic
ros2 topic echo /turtle1/cmd_vel

#Start recording data from a topic
ros2 bag record /turtle1/cmd_vel

#Record data from multiple topics with a custom file name
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose

#View information about a bag file
ros2 bag info subset

#Replay data from a bag file
ros2 bag play subset

#Monitoring the frequency of data publication on a topic
ros2 topic hz /turtle1/pose



###Chapter 2###
                                                                 #Part1 - Using colcon to build packages

# Install colcon on Linux
sudo apt install python3-colcon-common-extensions

# Create a ROS 2 workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Clone ROS 2 example packages into the workspace
git clone https://github.com/ros2/examples src/examples -b foxy

# Build the workspace using colcon with symlink install
colcon build --symlink-install

# Run tests for the built packages
colcon test

# Source the environment script
source install/setup.bash

# Run a subscriber node from the examples
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function

# Run a publisher node from the examples
ros2 run examples_rclcpp_minimal_publisher publisher_member_function


                                                                    #Part2 - Creating a workspace

# Source your ROS 2 installation environment
source /opt/ros/foxy/setup.bash

# Create a new workspace directory named ros2_ws
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the ros_tutorials repository's 'foxy-devel' branch into your workspace
git clone https://github.com/ros/ros_tutorials.git -b foxy-devel

# Navigate to the workspace root and resolve dependencies
cd ..
rosdep install -i --from-path src --rosdistro foxy -y

# Build the packages in your workspace using colcon
colcon build

# Open a new terminal and source the main ROS 2 environment as the underlay
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
source install/local_setup.bash

# Run the turtlesim from the overlay
ros2 run turtlesim turtlesim_node

# Run the turtlesim from the overlay to see the modified title bar
ros2 run turtlesim turtlesim_node

# Open a new terminal, source only the main ROS 2 environment, and run turtlesim from the underlay
source /opt/ros/foxy/setup.bash
ros2 run turtlesim turtlesim_node


                                                                      #Part3 - Creating a package

# Navigate to the source directory of your ROS 2 workspace
cd ~/ros2_ws/src

# Create a new ROS 2 package named "my_package" using ament_cmake build type
ros2 pkg create --build-type ament_cmake --node-name my_node my_package

# Navigate to the root of your ROS 2 workspace
cd ~/ros2_ws

# Build all packages in the workspace using colcon
colcon build

colcon build --packages-select my_package

# Source your main ROS 2 installation to set up the environment
source /opt/ros/foxy/setup.bash

# Source your workspace's setup file to include the newly built package
source install/local_setup.bash

# Run the executable named "my_node" from "my_package"
ros2 run my_package my_node


                                                                  #Part4 - Writing a simple publisher and subscriber (C++)

#Run the following command to create a new ROS 2 package named "cpp_pubsub" using the ament_cmake build type
ros2 pkg create --build-type ament_cmake cpp_pubsub

#Write the Publisher Node (publisher_member_function.cpp)
wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_publisher/member_function.cpp

#Customizing package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>cpp_pubsub</name>
  <version>0.0.0</version>
  <description>Examples of minimal publisher/subscriber using rclcpp</description>
  <maintainer email="tim@todo.todo">tim</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

#Customizing CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()

#Download the example subscriber code from the ROS 2 GitHub repository using the wget command
wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_subscriber/member_function.cpp

#Building and Running the ROS 2 Package:
#Ensure that the necessary dependencies (rclcpp and std_msgs) are installed by running rosdep install from the root of your workspace (ros2_ws).
#Building package using colcon build while specifying the package to build (cpp_pubsub).
#Source the setup files of your workspace to set up the environment variables.
#Run the "talker" node using ros2 run cpp_pubsub talker. This node will publish messages.
#In a new terminal, run the "listener" node using ros2 run cpp_pubsub listener. This node will subscribe to the published messages and display them.
#Use Ctrl+C in each terminal to stop the nodes.


                                                             #Part5 - Writing a simple publisher and subscriber (Python)

#Open a new terminal and ensure your ROS 2 installation is sourced.
#Navigate to your ROS 2 workspace directory created in a previous tutorial (in this case, "ros2_ws").
#Go into the "src" directory within your workspace since packages should be created there.
#Run the following command to create a new ROS 2 package named "py_pubsub" using the ament_python build type:
ros2 pkg create --build-type ament_python py_pubsub

#Write the Publisher Node (publisher_member_function.py)
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py

#Customize package.xml for Dependencies:
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>py_pubsub</name>
  <version>0.0.0</version>
  <description>Examples of minimal publisher/subscriber using rclpy</description>
  <maintainer email="tim@todo.todo">TSim</maintainer>
  <license>Apache License 2.0</license>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

#Customize setup.py for Entry Point:
#from setuptools import setup

#package_name = 'py_pubsub'

#setup(
#    name=package_name,
#    version='0.0.0',
#    packages=[package_name],
#    data_files=[
#        ('share/ament_index/resource_index/packages',
#            ['resource/' + package_name]),
#        ('share/' + package_name, ['package.xml']),
#    ],
#   install_requires=['setuptools'],
#    zip_safe=True,
#    maintainer='Tim',
#    maintainer_email='tim@todo.todo',
#    description='Examples of minimal publisher/subscriber using rclpy',
#    license='Apache License 2.0',
#    tests_require=['pytest'],
#    entry_points={
#        'console_scripts': [
#                'talker = py_pubsub.publisher_member_function:main',
#                'listener = py_pubsub.subscriber_member_function:main',
#        ],
#},
#)

#Return to the ros2_ws/src/py_pubsub/py_pubsub directory to create the subscriber node.
#Download the example subscriber code from the ROS 2 GitHub repository using the wget command:
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py

#Building and Running the ROS 2 Package:

#Build your package using colcon build while specifying the package to build (py_pubsub).

#Source the setup files of your workspace to set up the environment variables.

#Run the publisher node using 'ros2 run py_pubsub talker'. This node will publish messages.

#In a new terminal, run the subscriber node using 'ros2 run py_pubsub listener'. This node will subscribe to the published messages and display them.

#Use Ctrl+C in each terminal to stop the nodes.


                                                                 #Part6 - Writing a simple service and client (C++)

# Open a new terminal and ensure your ROS 2 installation is sourced.
# Navigate to your ROS 2 workspace directory created in a previous tutorial (e.g., "ros2_ws").
# Go into the "src" directory within your workspace since packages should be created there.
# Create a new package named "cpp_srvcli" using the ament_cmake build type.
# Include dependencies on "rclcpp" and "example_interfaces," which contains the .srv file.
# This command generates necessary files and folders for your package.

ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces

# Open the package.xml file for the "cpp_srvcli" package in your text editor.
# Fill in the <description>, <maintainer>, and <license> tags with appropriate information.
# Ensure that the necessary dependencies ("rclcpp" and "example_interfaces") are added automatically.

<description>C++ client server tutorial</description>
<maintainer email="tim@email.com">Tim</maintainer>
<license>Apache License 2.0</license>

# Inside the "ros2_ws/src/cpp_srvcli/src" directory, create a new file named "add_two_ints_server.cpp."
# Paste the provided C++ code within this file.
# This code defines a service node that receives two integers as a request and responds with their sum.

# The "add" function processes the request and calculates the sum.
# It also logs the incoming request and the response.

# The "main" function initializes ROS 2, creates a node named "add_two_ints_server,"
# and advertises a service named "add_two_ints" for this node.
# It then spins the node to make the service available.

# Add the following code block to the CMakeLists.txt file to create an executable named "server" for the service node:
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)

# To make the executable discoverable by "ros2 run," add the following lines to install the target:

install(TARGETS
    server
  DESTINATION lib/${PROJECT_NAME})
  
# Inside the "ros2_ws/src/cpp_srvcli/src" directory, create a new file named "add_two_ints_client.cpp."
# Paste the provided C++ code within this file.
# This code defines a client node that sends a request with two integers to the service node and receives the sum as a response.

# The "main" function initializes ROS 2, checks if the correct number of arguments are provided (two integers),
# creates a node named "add_two_ints_client," and creates a client for the "add_two_ints" service.

# It constructs a request with the provided integers and waits for the service to become available.
# Once the service is available, it sends the request and waits for the response asynchronously.
# Finally, it prints the received sum or an error message.

# Update the CMakeLists.txt file to create an executable named "client" for the client node:

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)

# Run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:

rosdep install -i --from-path src --rosdistro foxy -y

# Navigate back to the root of your workspace and build the package:

colcon build --packages-select cpp_srvcli

# Open a new terminal and source the setup files of your workspace:

source install/setup.bash

# Start the service node:

ros2 run cpp_srvcli server

# Open another terminal, source the setup files again, and run the client node with two integers as arguments:

ros2 run cpp_srvcli client 2 3

# The client will send the request, and the server will respond with the sum. You will see the log messages indicating the process.

# Use Ctrl+C in the server terminal to stop the service node.



                                                             #Part7 - Writing a simple service and client (Python)

# Open a new terminal and ensure your ROS 2 installation is sourced.
# Navigate to your ROS 2 workspace directory created in a previous tutorial (e.g., "ros2_ws").
# Go into the "src" directory within your workspace since packages should be created there.
# Create a new package named "py_srvcli" using the ament_python build type.
# Include dependencies on "rclpy" and "example_interfaces," which contains the .srv file.
# This command generates necessary files and folders for your package.

ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces

# Open the package.xml file for the "py_srvcli" package in your text editor.
# Fill in the <description>, <maintainer>, and <license> tags with appropriate information.
# Ensure that the necessary dependencies ("rclpy" and "example_interfaces") are added automatically.

<description>Python client server tutorial</description>
<maintainer email="tim@email.com">Tim</maintainer>
<license>Apache License 2.0</license>

# Add the same information to the setup.py file for the maintainer, maintainer_email, description, and license fields:

maintainer='Tim',
maintainer_email='tim@email.com',
description='Python client server tutorial',
license='Apache License 2.0',


# Inside the "ros2_ws/src/py_srvcli/py_srvcli" directory, create a new file named "service_member_function.py."
# Paste the provided Python code within this file.
# This code defines a service node that receives two integers as a request and responds with their sum.

# The code imports necessary modules and defines a class named MinimalService.
# The class constructor initializes the node, creates a service, and defines a callback function.

# The code defines a ROS 2 service node that adds two integers.

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# To allow the "ros2 run" command to run your service node, add the following line between the 'console_scripts' brackets in setup.py:

'service = py_srvcli.service_member_function:main',


# Inside the "ros2_ws/src/py_srvcli/py_srvcli" directory, create a new file named "client_member_function.py."
# Paste the provided Python code within this file.
# This code defines a client node that sends a request with two integers to the service node and receives the sum as a response.

# The code imports necessary modules and defines a class named MinimalClientAsync.
# The class constructor initializes the node, creates a client, and defines a function to send the request.


# The code defines a ROS 2 client node that sends a request to the service node for adding two integers.

import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
# To allow the "ros2 run" command to run your client node, add the following line between the 'console_scripts' brackets in setup.py:

'client = py_srvcli.client_member_function:main',

# It’s good practice to run "rosdep" in the root of your workspace ("ros2_ws") to check for missing dependencies before building:

rosdep install -i --from-path src --rosdistro foxy -y

# Navigate back to the root of your workspace, "ros2_ws," and build your new package:

colcon build --packages-select py_srvcli

# Open a new terminal, navigate to "ros2_ws," and source the setup files:

source install/setup.bash

# Now run the service node:

ros2 run py_srvcli service

# The node will wait for the client’s request.

# Open another terminal and source the setup files again. Start the client node, followed by any two integers separated by a space:

ros2 run py_srvcli client 2 3

# If you chose 2 and 3, for example, the client would receive a response like this:

# [INFO] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5

# Return to the terminal where your service node is running. You will see that it published log messages when it received the request:

# [INFO] [minimal_service]: Incoming request
# a: 2 b: 3

# Enter Ctrl+C in the server terminal to stop the node from spinning.


                                                                      #Part8 - Creating custom msg and srv files

# Create a new package named "tutorial_interfaces" in your ROS 2 workspace.
ros2 pkg create --build-type ament_cmake tutorial_interfaces

# Inside the tutorial_interfaces/msg directory, create a new file called Num.msg.
# Define a custom message that transfers a single 64-bit integer called "num".

int64 num

# Inside the tutorial_interfaces/msg directory, create a new file called Sphere.msg.
# Define a custom message that includes a message from another package (geometry_msgs/Point) and a float64 called "radius".
geometry_msgs/Point center
float64 radius

# Inside the tutorial_interfaces/srv directory, create a new file called AddThreeInts.srv.
# Define a custom service with a request containing three int64 fields (a, b, c) and a response containing a single int64 field called "sum".
int64 a
int64 b
int64 c
---
int64 sum

#Modify CMakeLists.txt
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs
)

#Modify package.xml
<depend>geometry_msgs</depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

# Build the "tutorial_interfaces" package to generate language-specific code.
colcon build --packages-select tutorial_interfaces

# Source the workspace to make the custom interfaces discoverable.
source install/setup.bash

# Check the message definitions using the 'ros2 interface show' command.
ros2 interface show tutorial_interfaces/msg/Num
ros2 interface show tutorial_interfaces/msg/Sphere
ros2 interface show tutorial_interfaces/srv/AddThreeInts

#Modify the publisher node to use the custom Num.msg for publishing integer values.

#Modify the subscriber node to use the custom Num.msg for receiving integer values.

# Add dependencies on tutorial_interfaces for both publisher and subscriber.
find_package(tutorial_interfaces REQUIRED)

# Update dependencies for talker and listener.
ament_target_dependencies(talker rclcpp tutorial_interfaces)
ament_target_dependencies(listener rclcpp tutorial_interfaces)

#Modify the client and server nodes to use the custom AddThreeInts.srv for requesting and responding with integers.

# Add dependencies on tutorial_interfaces for both server and client.
find_package(tutorial_interfaces REQUIRED)

# Update dependencies for server and client.
ament_target_dependencies(server rclcpp tutorial_interfaces)
ament_target_dependencies(client rclcpp tutorial_interfaces)

# Build the package that includes publisher, subscriber, client, and server nodes.
colcon build --packages-select <package_name>

# Run the nodes.
ros2 run <package_name> talker
ros2 run <package_name> listener

# For the client and server:
ros2 run <package_name> server
ros2 run <package_name> client <arg1> <arg2> <arg3>



                                                                  # Part9 - Implementing custom interfaces

# Create a new package named "more_interfaces" and create a directory for message files.
ros2 pkg create --build-type ament_cmake more_interfaces
mkdir more_interfaces/msg

#Create a Message File (AddressBook.msg)
uint8 PHONE_TYPE_HOME=0
uint8 PHONE_TYPE_WORK=1
uint8 PHONE_TYPE_MOBILE=2

string first_name
string last_name
string phone_number
uint8 phone_type

#Update package.xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

# Update CMakeLists.txt
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/AddressBook.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

#Create a Node Using the Custom Message
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"

#...

#Update the CMakeLists.txt to add a new target for the publisher node.
find_package(rclcpp REQUIRED)

add_executable(publish_address_book src/publish_address_book.cpp)
ament_target_dependencies(publish_address_book rclcpp)

install(TARGETS
    publish_address_book
  DESTINATION lib/${PROJECT_NAME})

#Link against the Interface
rosidl_target_interfaces(publish_address_book
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

#Build and Run the Package
cd ~/ros2_ws
colcon build --packages-up-to more_interfaces

#Source the workspace and run the publisher:
source install/local_setup.bash
ros2 run more_interfaces publish_address_book
 


                                                                          # Part10 - Using Parameters in a class (C++)

# Create a new ROS 2 workspace named ros2_ws and navigate to its source directory.
ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp

#Update package.xml
<description>C++ parameter tutorial</description>
<maintainer email="Tim@email.com">Tim</maintainer>
<license>Apache License 2.0</license>

#Write the C++ Node
#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("minimal_param_node")
  {
    this->declare_parameter("my_parameter", "world");

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalParam::timer_callback, this));
  }

  void timer_callback()
  {
    std::string my_param = this->get_parameter("my_parameter").as_string();

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameters(all_new_parameters);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}

#Add Executable
add_executable(minimal_param_node src/cpp_parameters_node.cpp)
ament_target_dependencies(minimal_param_node rclcpp)

install(TARGETS
    minimal_param_node
  DESTINATION lib/${PROJECT_NAME}
)

#Build and Run
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --packages-select cpp_parameters

#We source the setup files to set up the ROS 2 environment.
source install/setup.bash
ros2 run cpp_parameters minimal_param_node

#Change via the Console
ros2 run cpp_parameters minimal_param_node

#In another terminal, you can list parameters:
ros2 param list

#To change the parameter, run:
ros2 param set /minimal_param_node my_parameter earth

#Change via a Launch File
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="minimal_param_node",
            name="custom_minimal_param_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])

#Edit the CMakeLists.txt file and add the following code below the previous additions:
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#Now, build the package again:
colcon build --packages-select cpp_parameters

#And run the node using the launch file:
ros2 launch cpp_parameters cpp_parameters_launch.py

                                                                      #Part11 - Using parameters in a class (Python)

# Create a new ROS 2 workspace named ros2_ws.
ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy

#Update package.xml as in a previous example 

#Create a new file named python_parameters_node.py inside the "python_parameters" directory and add the following code:
import rclpy
import rclpy.node

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')

        self.declare_parameter('my_parameter', 'world')

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

#Open the setup.py file. Match the maintainer, maintainer_email, description, and license fields to your package.xml:
maintainer='Tim',
maintainer_email='tim@email.com',
description='Python parameter tutorial',
license='Apache License 2.0',

#Add the following line within the console_scripts brackets of the entry_points field:
#entry_points={
#    'console_scripts': [
#        'minimal_param_node = python_parameters.python_parameters_node:main',
#    ],
#},

#Build and Run
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --packages-select python_parameters

#Source the setup files:
source install/setup.bash

#We execute the node we've created, and it should print a message every second.
ros2 run python_parameters minimal_param_node

#Change via the Console
ros2 run python_parameters minimal_param_node

#In another terminal, you can list parameters:
ros2 param list

#To change the parameter, run:
ros2 param set /minimal_param_node my_parameter earth

# Change via a Launch File
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='python_parameters',
            executable='minimal_param_node',
            name='custom_minimal_param_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'my_parameter': 'earth'}
            ]
        )
    ])

#This Python launch file configures and launches the node, setting the "my_parameter" to "earth" during launch.

#Open the setup.py file again file again and add import statements to the top of the file and the following new statement to the data_files parameter:
import os
from glob import glob
# ...

setup(
  # ...
  data_files=[
      # ...
      (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ]
  )

#build the package again:
colcon build --packages-select python_parameters

#And run the node using the launch file:
ros2 launch python_parameters python_parameters_launch.py

                                                               #Part12 - Using ros2doctor to identify issues


# Checking ROS 2 setup with ros2doctor.
ros2 doctor

# Start the turtlesim system in one terminal.
ros2 run turtlesim turtlesim_node

# Open another terminal and run teleop controls for turtlesim.
ros2 run turtlesim turtle_teleop_key

# Now, in a new terminal, run ros2doctor again.
ros2 doctor

# In two new terminals, run these commands to create subscribers for the topics.
ros2 topic echo /turtle1/color_sensor
ros2 topic echo /turtle1/pose

# Get a full report of your ROS 2 setup.
ros2 doctor --report

#The report provides information categorized into sections such as network configuration, platform information, RMW middleware, ROS 2 information, and topic list.

#Example Section:
NETWORK CONFIGURATION
...

PLATFORM INFORMATION
...

RMW MIDDLEWARE
...

ROS 2 INFORMATION
...

TOPIC LIST
...

                                                                 #Part13 - Creating and using plugins (C++)
  
 
 # Create a package for the base class
ros2 pkg create --build-type ament_cmake polygon_base --dependencies pluginlib --node-name area_node

#In this step, we create a new ROS 2 package named polygon_base to define the base class for our plugins. This package depends on pluginlib, which is used for loading plugins dynamically.
// Define the base class RegularPolygon
#ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
#define POLYGON_BASE_REGULAR_POLYGON_HPP

namespace polygon_base
{
  class RegularPolygon
  {
    public:
      virtual void initialize(double side_length) = 0;
      virtual double area() = 0;
      virtual ~RegularPolygon(){}

    protected:
      RegularPolygon(){}
  };
}  // namespace polygon_base

#endif  // POLYGON_BASE_REGULAR_POLYGON_HPP

# Update CMakeLists.txt to make the header file available
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

# Create a package for the plugins
ros2 pkg create --build-type ament_cmake polygon_plugins --dependencies polygon_base pluginlib --library-name polygon_plugins


#Implement Square and Triangle classes
namespace polygon_plugins
{
  class Square : public polygon_base::RegularPolygon
  {
    public:
      void initialize(double side_length) override
      {
        side_length_ = side_length;
      }

      double area() override
      {
        return side_length_ * side_length_;
      }

    protected:
      double side_length_;
  };

  class Triangle : public polygon_base::RegularPolygon
  {
    public:
      void initialize(double side_length) override
      {
        side_length_ = side_length;
      }

      double area() override
      {
        return 0.5 * side_length_ * getHeight();
      }

      double getHeight()
      {
        return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
      }

    protected:
      double side_length_;
  };
}

#In this code, we implement two classes, Square and Triangle, which inherit from the RegularPolygon base class. These classes provide specific implementations for calculating the area of squares and triangles.
<!-- Create plugins.xml to declare plugins -->
<library path="polygon_plugins">
  <class type="polygon_plugins::Square" base_class_type="polygon_base::RegularPolygon">
    <description>This is a square plugin.</description>
  </class>
  <class type="polygon_plugins::Triangle" base_class_type="polygon_base::RegularPolygon">
    <description>This is a triangle plugin.</description>
  </class>
</library>


#Export the plugin classes
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)

# Build the packages
colcon build --packages-select polygon_base polygon_plugins


# Source the setup files
source install/setup.bash

#We source the setup files to set up the ROS 2 environment.
// Use the plugins in a ROS 2 node
#include <pluginlib/class_loader.hpp>
#include <polygon_base/regular_polygon.hpp>

int main(int argc, char** argv)
{
  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  // Create a ClassLoader for RegularPolygon plugins
  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("polygon_base", "polygon_base::RegularPolygon");

  try
  {
    // Create instances of Square and Triangle plugins
    std::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createSharedInstance("polygon_plugins::Triangle");
    triangle->initialize(10.0);

    std::shared_ptr<polygon_base::RegularPolygon> square = poly_loader.createSharedInstance("polygon_plugins::Square");
    square->initialize(10.0);

    // Calculate and print areas
    printf("Triangle area: %.2f\n", triangle->area());
    printf("Square area: %.2f\n", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}



                                                               









