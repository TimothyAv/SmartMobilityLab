# by Akhrorov Temurbek 12204574
# ROS2 Advanced

# Enable topic statistics in ROS 2 using C++.

# Change directory to the location where the example talker code will be downloaded.
wget -O member_function_with_topic_statistics.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function_with_topic_statistics.cpp

# Run the subscriber node with statistics enabled.
ros2 run cpp_pubsub listener_with_topic_statistics

# Start the talker node.
ros2 run cpp_pubsub talker

# In a new terminal window, execute the following command to list topics.
ros2 topic list

# View the statistics data published to a specific topic.
ros2 topic echo /statistics

# Use the Fast DDS Discovery Server as the discovery protocol.

# Start the discovery server.
fastdds discovery --server-id 0

# Set the ROS_DISCOVERY_SERVER environment variable to point to the discovery server.
export ROS_DISCOVERY_SERVER=127.0.0.1:11811

# Launch the listener node with a custom name.
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server

# Open another terminal and set the ROS_DISCOVERY_SERVER environment variable.
export ROS_DISCOVERY_SERVER=127.0.0.1:11811

# Start the talker node with a custom name.
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server

# Run another listener node not connected to the discovery server.
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=simple_listener

# Create a talker using the default DDS distributed discovery mechanism for discovery.
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=simple_talker

# Establish communication with redundant discovery servers.
fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811
fastdds discovery --server-id 1 --ip-address 127.0.0.1 --port 11888

# Set the ROS_DISCOVERY_SERVER variable to reference both servers.
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"

# Start a talker and a listener connected to the redundant servers.
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

# Establish communication with a backed-up server.
fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811 --backup

# Set the ROS_DISCOVERY_SERVER variable to connect to the backed-up server.
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"

# Start a talker and a listener connected to the backed-up server.
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

# Run the first server listening on the default port.
fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811

# Run the second server listening on a different port.
fastdds discovery --server-id 1 --ip-address 127.0.0.1 --port 11888

# Run talker and listener nodes connected to specific servers using ROS_DISCOVERY_SERVER.
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_1
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_1
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_2
export ROS_DISCOVERY_SERVER=";127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_2

# Create a talker and listener that discover each other through the server.
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker

# Instantiate a ROS 2 Daemon using the Super Client configuration.
export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
ros2 daemon stop
ros2 daemon start
ros2 topic list
ros2 node info /talker
ros2 topic info /chatter
ros2 topic echo /chatter

# Visualize the Node's Graph using the rqt_graph tool.
export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
rqt_graph

# Build a system with a talker and listener using the Super Client configuration.
# Start a Server.
fastdds discovery -i 0 -l 127.0.0.1 -p 11811

# Run the talker and listener in separate terminals.
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker

# Continue using the ROS 2 CLI with the new configuration.
export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
ros2 topic list --no-daemon
ros2 node info /talker --no-daemon --spin-time 2

# Implement a custom memory allocator.

# Run the example executable that prints variable values.
ros2 run demo_nodes_cpp allocator_tutorial

# Alternatively, run the example with an intra-process pipeline.
ros2 run demo_nodes_cpp allocator_tutorial intra

# Unlock the potential of the Fast DDS middleware.

# Create a new package named sync_async_node_example_cpp in a new workspace.
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs -- sync_async_node_example_cpp

# Add a file named sync_async_writer.cpp to the package with synchronous and asynchronous publishers.
# Synchronous publisher uses /sync_topic, and asynchronous publisher uses /async_topic.

# Source the setup files and run the node.
source install/setup.bash
ros2 run sync_async_node_example_cpp SyncAsyncWriter

# Open two terminals and run the publisher and subscriber nodes.
# The /async_topic messages should reach the subscriber, while the /sync_topic subscriber is in a different partition and won't receive data.

# Start the service node in one terminal.
ros2 run sync_async_node_example_cpp ping_service

# In another terminal, run the client node to send requests and receive responses.

# The service console will show the server is ready to serve, incoming requests, and sending back responses.

# Record a bag from a C++ node.

# If ROS Bag 2 is not installed, you can install it using the provided command.

# Create a new package and build it.
ros2 pkg create --build-type ament_cmake bag_recorder_nodes --dependencies example_interfaces rclcpp rosbag2_cpp std_msgs

# Build the package.
colcon build --packages-select bag_recorder_nodes

# Run the bag recorder node.
ros2 run bag_recorder_nodes simple_bag_recorder

# Start the talker node in another terminal.

# Terminate both nodes and run the listener node to play the recorded bag.

# Play the bag with the recorded data.

# Build the package.
colcon build --packages-select bag_recorder_nodes

# Run the data generator node.
ros2 run bag_recorder_nodes data_generator_node

# Wait for about 30 seconds and then stop the node. Play back the created bag.

# Open another terminal and echo the /synthetic topic.

# Build the package.
colcon build --packages-select bag_recorder_nodes

# Run the data generator executable.
ros2 run bag_recorder_nodes data_generator_executable

# Play back the created bag and echo the /synthetic topic.

# Record a bag from a Python node.

# Install ROS Bag 2 if it's not already installed.

# Create a new package for Python nodes and build it.
ros2 pkg create --build-type ament_python bag_recorder_nodes_py --dependencies rclpy rosbag2_py example_interfaces std_msgs

# Build the package.
colcon build --packages-select bag_recorder_nodes_py

# Run the Python bag recorder node.

# Start the talker node in another terminal.

# Terminate both nodes and run the listener node to play the recorded bag.

# Play the bag with the recorded data.

# Build the package.
colcon build --packages-select bag_recorder_nodes_py

# Run the data generator node.
ros2 run bag_recorder_nodes_py data_generator_node

# Wait for about 30 seconds and then stop the node. Play back the created bag.

# Open another terminal and echo the /synthetic topic.

# Build the package.
colcon build --packages-select bag_recorder_nodes_py

# Run the data generator executable.
ros2 run bag_recorder_nodes_py data_generator_executable

# Play back the created bag and echo the /synthetic topic.

# Install the ROS 2 rosbag2 package if not installed already.
sudo apt install ros-humble-rosbag2

# Create a new ROS 2 package named 'bag_reading_cpp' with dependencies.
ros2 pkg create --build-type ament_cmake --license Apache-2.0 bag_reading_cpp --dependencies rclcpp rosbag2_cpp turtlesim

# Build the 'bag_reading_cpp' package within the workspace.
colcon build --packages-select bag_reading_cpp

# Source the setup files for the workspace.
source install/setup.bash

# Run the 'simple_bag_reader' script, specifying the path to your setup bag.
ros2 run bag_reading_cpp simple_bag_reader /path/to/setup

# Webots Installation on Ubuntu.

# Install Webots ROS 2 package via apt.
sudo apt-get install ros-humble-webots-ros2

# Source the ROS 2 environment.
source /opt/ros/humble/setup.bash

# Set the WEBOTS_HOME environment variable to specify the Webots installation location.
export WEBOTS_HOME=/usr/local/webots

# If Webots ROS 2 package is installed from source, source the ROS 2 workspace.
cd ~/ros2_ws
source install/local_setup.bash

# Launch Webots simulation using ROS 2 launch command.
ros2 launch webots_ros2_universal_robot multirobot_launch.py

# Webots Installation on Windows Subsystem for Linux (WSL).

# Install Webots ROS 2 package via apt within the WSL environment.
sudo apt-get install ros-humble-webots-ros2

# Source the ROS 2 environment in WSL.
source /opt/ros/humble/setup.bash

# Set the WEBOTS_HOME environment variable for Webots installation on Windows (use '/mnt' to refer to Windows paths).
export WEBOTS_HOME=/mnt/c/Program\ Files/Webots

# If Webots ROS 2 package is installed from source, source the ROS 2 workspace.
cd ~/ros2_ws
source install/local_setup.bash

# Launch Webots simulation using ROS 2 launch command within WSL.
ros2 launch webots_ros2_universal_robot multirobot_launch.py

# Ensure functionality by running examples with RViz.
sudo apt install ros-humble-slam-toolbox
ros2 launch webots_ros2_tiago robot_launch.py rviz:=true slam:=true

# Control the Tiago robot using teleop_twist_keyboard.
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Enable RViz2 to work with WSL (requires X11 forwarding or WSL upgrade).
wsl --update

# Webots Installation on macOS with a VM.

# Install ROS 2 distribution following Ubuntu/Debian instructions inside the VM.
mkdir /home/ubuntu/shared

# Mount the shared folder to the host (adjust the path to your shared folder).
sudo mount -t 9p -o trans=virtio share /home/ubuntu/shared -oversion=9p2000.L

# Add the shared folder to /etc/fstab for automatic mounting on VM startup.

# Set the WEBOTS_SHARED_FOLDER environment variable in the VM for data exchange between host and VM.

# Install Webots ROS 2 package via apt within the VM.
sudo apt-get install ros-humble-webots-ros2

# Download the Webots simulation server and run it outside the VM (on the host).
export WEBOTS_HOME=/Applications/Webots.app
python3 local_simulation_server.py

# Source the ROS 2 environment and the ROS 2 workspace if not done already.
source /opt/ros/humble/setup.bash

# Set the correct WEBOTS_SHARED_FOLDER environment variable (adjust paths if needed).

# Launch Webots simulation using ROS 2 launch command.

# Install the Fast DDS middleware.

# Modify the colcon build command to include the security plugins for Fast DDS.
colcon build --symlink-install --cmake-args -DSECURITY=ON

# Create a workspace directory for the security demo.
mkdir ~/sros2_demo

# Use sros2 utilities to create a keystore for security materials.
cd ~/sros2_demo
ros2 security create_keystore demo_keystore

# Generate keys and certificates for talker and listener nodes.
ros2 security create_enclave demo_keystore /talker_listener/talker
ros2 security create_enclave demo_keystore /talker_listener/listener

# Set environment variables for security.
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# Launch the talker node with security enabled.
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker

# Launch the listener node with security enabled.
ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener

# Examine the security keystore content using openssl.

# View X.509 certificate details.
cd ~/sros2_demo/demo_keys/public
openssl x509 -in ca.cert.pem -text -noout

# Display elliptic curve private key details.
cd ~/sros2_demo/demo_keys/private
openssl ec -in ca.key.pem -text -noout

# Verify the S/MIME signature of the governance file.
openssl smime -verify -in governance.p7s -CAfile ../public/permissions_ca.cert.pem

# Test ROS 2 security by launching talker and listener nodes with security.

# Customize and run the listener node to simulate secure communication.
ros2 run demo_nodes_cpp listener --ros-args --enclave /talker_listener/listener

# Securely launch the talker node as well.
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker

# Enable the deployment guidelines workspace.

# Create a workspace folder.
mkdir ~/security_gd_tutorial
cd ~/security_gd_tutorial

# Build a Docker image using the provided Dockerfile.
docker build -t ros2_security/deployment_tutorial --build-arg ROS_DISTRO=humble .

# Download the Docker Compose configuration YAML file.
wget https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Advanced/Security/resources/deployment_gd/compose.deployment.yaml

# Run the deployment example using Docker Compose.
docker compose -f compose.deployment.yaml up

# Attach to running containers to explore their keystore content.

# In one terminal, access the keystore of the listener container.
docker exec -it tutorial-listener-1 bash
cd keystore
tree

# In another terminal, access the keystore of the talker container.
docker exec -it tutorial-talker-1 bash
cd keystore
tree


