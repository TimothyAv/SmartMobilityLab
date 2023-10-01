# Temur Akhrorov 12204574
# ROS 2 Documentation: Intermediate level


# Managing Dependencies with rosdep
                           
# With the following command, ROS users may quickly install rosdep using the ROS distribution package manager:
apt-get install python3-rosdep

# If the system package is unavailable and you aren't using ROS, you may use pip to directly install rosdep from https://pypi.org:
pip install rosdep

# We are now prepared to use the tool since we have a basic grasp of rosdep, package.xml, and rosdistro. If this is your first time using rosdep, initialize it by running:
sudo rosdep init
rosdep update

# To install dependencies, we can finally run rosdep install. To install all dependencies, this is often done across a workspace with many packages in a single call. The order would seem like this if you are in the root of the workspace with a src directory containing source code:
rosdep install --from-paths src -y --ignore-src

# Creating an action
                            
# Set up a workspace and create a package named action_tutorials_interfaces:
mkdir -p ros2_ws/src # You can reuse an existing workspace with this naming convention
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces

# Create an 'action' directory in our ROS 2 package action_tutorials_interfaces:
cd action_tutorials_interfaces
mkdir action

# Now, we should be able to build the package containing the Fibonacci action definition:
# Navigate to the root of the workspace
cd ~/ros2_ws
# Build
colcon build

# By convention, action types are prefixed by their package name and the term 'action'. So, when referring to our new action, it will have the full name action_tutorials_interfaces/action/Fibonacci.

# You can verify that our action built successfully using the command-line tool:

# Source our workspace
# On Windows: execute install/setup.bat
. install/setup.bash
# Check the existence of our action definition
ros2 interface show action_tutorials_interfaces/action/Fibonacci

# Writing an action server and client (C++)
                            
# Navigate to the action workspace you created in the previous tutorial (remember to source the workspace), and create a new package for the C++ action server:                            
# cd ~/ros2_ws/src
ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp
                         
# Now, we can compile the package. Go to the top-level of the ros2_ws directory and execute:
colcon build

# With the action client built, you can now run it. Make sure an action server is running in a separate terminal. Then, source the workspace you just built (ros2_ws), and attempt to run the action client:
ros2 run action_tutorials_cpp fibonacci_action_client

# Writing an action server and client (Python)
                                 
# Let's attempt to run our Python action server:
python3 fibonacci_action_server.py

# In another terminal, you can use the command-line interface to send a goal:
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

# After restarting the action server, you can confirm that feedback is now published by using the command-line tool with the --feedback option:
ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

# To test our action client, first run the previously built action server in another terminal:
python3 fibonacci_action_server.py

# In another terminal, run the action client:
python3 fibonacci_action_client.py

# With an action server running in a separate terminal, proceed to run our Fibonacci action client:
python3 fibonacci_action_client.py

# Composing multiple nodes in a single process
                            
# To check the components registered and available in the workspace, execute the following in a shell:
ros2 component types

# In the first shell, start the component container:
ros2 run rclcpp_components component_container

# Open the second shell and verify that the container is running using ros2 command line tools:
ros2 component list

# In the second shell, load the talker component (refer to talker source code):
ros2 component load /ComponentManager composition composition::Talker

# Execute another command in the second shell to load the listener component (refer to listener source code):
ros2 component load /ComponentManager composition composition::Listener

# The ros2 command-line utility can now be used to inspect the state of the container:
ros2 component list

# In the first shell:
ros2 run rclcpp_components component_container

# In the second shell (refer to server and client source code):
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client

# In the shell call (refer to source code):
ros2 run composition manual_composition

# This demo provides an alternative to run-time composition by creating a generic container process and explicitly passing the libraries to load without using ROS interfaces. The process opens each library and creates one instance of each "rclcpp::Node" class in the library source code:
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so

# While command-line tools are useful for debugging and diagnosing component configurations, it's often more convenient to start a set of components at the same time. To automate this, we can use a launch file:
ros2 launch composition composition_demo.launch.py

# In the first shell, start the component container:
ros2 run rclcpp_components component_container

# Verify that the container is running using ros2 command line tools:
ros2 component list

# You should see a name of the component:
/ComponentManager

# Use the unique ID to unload the node from the component container:
ros2 component unload /ComponentManager 1 2

# The component manager name and namespace can be remapped via standard command line arguments:
ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns

# In the first shell, start the component container:
ros2 run rclcpp_components component_container

# Remap the node name:
ros2 component load /ComponentManager composition composition::Talker --node-name talker2

# Remap the namespace:
ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns

# Remap both:
ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2

# Now use ros2 command line utility:
ros2 component list

# The ros2 component load command-line supports passing arbitrary parameters to the node as it is constructed. This functionality can be used as follows:
ros2 component load /ComponentManager image_tools image_tools::Cam2Image -p burger_mode:=true

# The ros2 component load command-line supports passing particular options to the component manager for use when constructing the node. As of now, the only command-line option that is supported is to instantiate a node using intra-process communication. This functionality can be used as follows:
ros2 component load /ComponentManager composition composition::Talker -e use_intra_process_comms:=true

# Monitoring for parameter changes (C++)
                             
# Remember that packages should be created in the src directory, not the root of the workspace. So, navigate into ros2_ws/src and then create a new package there:
ros2 pkg create --build-type ament_cmake cpp_parameter_event_handler --dependencies rclcpp

# It's a good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# Navigate back to the root of your workspace, ros2_ws, and build your new package:
colcon build --packages-select cpp_parameter_event_handler

# Open a new terminal, navigate to ros2_ws, and source the setup files:
. install/setup.bash

# Now run the node:
ros2 run cpp_parameter_event_handler parameter_event_handler

# The node is now active and has a single parameter. It will print a message whenever this parameter is updated. To test this, open up another terminal, source the ROS setup file as before (. install/setup.bash), and execute the following command:
ros2 param set node_with_parameters an_int_param 43

# In a terminal, navigate back to the root of your workspace, ros2_ws, and build your updated package as before:
colcon build --packages-select cpp_parameter_event_handler

# Then source the setup files:
. install/setup.bash

# Now, to test monitoring of remote parameters, first run the newly-built parameter_event_handler code:
ros2 run cpp_parameter_event_handler parameter_event_handler

# Now, to test monitoring of remote parameters, first run the newly-built parameter_event_handler code:
ros2 run cpp_parameter_event_handler parameter_event_handler

# Next, from another terminal (with ROS initialized), run the parameter_blackboard demo application, as follows:
ros2 run demo_nodes_cpp parameter_blackboard

# Finally, from a third terminal (with ROS initialized), let's set a parameter on the parameter_blackboard node:
ros2 param set parameter_blackboard a_double_param 3.45

# LAUNCH
                                
# Creating a launch file
                                
# Create a new directory to store your launch files:
mkdir launch

# To run the launch file created above, enter the directory you created earlier and run the following command:
ros2 launch <package_name> <launch_file_name>

# To see the system in action, open a new terminal and run the ros2 topic pub command on the /turtlesim1/turtle1/cmd_vel topic to get the first turtle moving:
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"

# While the system is still running, open a new terminal and run rqt_graph to visualize the relationship between the nodes in your launch file:
rqt_graph

# Introducing launch files into ROS2 packages
                        
# Create a workspace for the package to reside in:
mkdir -p launch_ws/src
cd launch_ws/src

ros2 pkg create py_launch_example --build-type ament_python

# Go to the top-level of the workspace and build it:
colcon build                              
                  
# After a successful colcon build and sourcing the workspace, you should be able to run the launch file as follows:
ros2 launch py_launch_example my_script_launch.py

# Using substitutions
                                                                               
# Create a new package of build_type ament_python called launch_tutorial:
ros2 pkg create launch_tutorial --build-type ament_python

# Inside that package, create a directory called launch:
mkdir launch_tutorial/launch

# Navigate to the root of the workspace and build the package:
colcon build

# Now you can launch the example_main.launch.py file using the ros2 launch command.
ros2 launch launch_tutorial example_main.launch.py

# If you want to modify the provided launch arguments, you can either update them in the launch_arguments dictionary in example_main.launch.py or launch the example_substitutions.launch.py with your preferred arguments. To view the available arguments for the launch file, run the following command:
ros2 launch launch_tutorial example_substitutions.launch.py --show-args

# Now you can pass the desired arguments to the launch file as follows:
ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200

# Using event handlers
                             
# Go to the root of the workspace and build the package:
colcon build

# Now you can launch the example_event_handlers.launch.py file using the ros2 launch command.
ros2 launch launch_tutorial example_event_handlers.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200

# Managing large projects
                           
# To see the result of our code, build the package and launch the top-level launch file using the following command:
ros2 launch launch_tutorial launch_turtlesim.launch.py

# If you want to control turtle1, run the teleop node:
ros2 run turtlesim turtle_teleop_key

# Installing necessary packages and dependencies for TF2
sudo apt-get install ros-humble-turtle-tf2-py ros-humble-tf2-tools ros-humble-tf-transformations

# Launching the turtle_tf2_py demo
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py

# Controlling the turtle with keyboard input
ros2 run turtlesim turtle_teleop_key

# Viewing frames broadcasted by tf2
ros2 run tf2_tools view_frames

# Examining the transform between turtle2 and turtle1
ros2 run tf2_ros tf2_echo turtle2 turtle1

# Using rviz to visualize tf2 frames
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz

# Creating a new Python package for learning_tf2_py
ros2 pkg create --build-type ament_python learning_tf2_py

# Downloading static broadcaster Python code
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/static_turtle_tf2_broadcaster.py

# Checking for missing dependencies
rosdep install -i --from-path src --rosdistro humble -y

# Building the new package
colcon build --packages-select learning_tf2_py

# Sourcing setup files and running the static_turtle_tf2_broadcaster node
. install/setup.bash
ros2 run learning_tf2_py static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0

# Echoing the tf_static topic to check if the static transform has been published
ros2 topic echo /tf_static

# Publishing a static coordinate transform to tf2 using xyz offset and roll/pitch/yaw angles
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --yaw yaw --pitch pitch --roll roll --frame-id frame_id --child-frame-id child_frame_id

# Publishing a static coordinate transform to tf2 using xyz offset and quaternion
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --qx qx --qy qy --qz qz --qw qw --frame-id frame_id --child-frame-id child_frame_id

# Creating a new C++ package for learning_tf2_cpp
ros2 pkg create --build-type ament_cmake --dependencies geometry_msgs rclcpp tf2 tf2_ros turtlesim -- learning_tf2_cpp

# Downloading static broadcaster C++ code
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/static_turtle_tf2_broadcaster.cpp

# Echoing the tf_static topic to check if the static transform has been published
ros2 topic echo /tf_static

# Publishing a static coordinate transform to tf2 using xyz offset and roll/pitch/yaw angles
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --yaw yaw --pitch pitch --roll roll --frame-id frame_id --child-frame-id child_frame_id

# Publishing a static coordinate transform to tf2 using xyz offset and quaternion
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --qx qx --qy qy --qz qz --qw qw --frame-id frame_id --child-frame-id child_frame_id

# Creating a Python package for learning_tf2_py
ros2 pkg create --build-type ament_python learning_tf2_py

# Downloading broadcaster Python code
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_broadcaster.py

# Checking for missing dependencies
rosdep install -i --from-path src --rosdistro humble -y

# Building the package
colcon build --packages-select learning_tf2_py

# Sourcing setup files and running the launch file for turtle_tf2_demo
. install/setup.bash
ros2 launch learning_tf2_py turtle_tf2_demo.launch.py

# Controlling the turtle with keyboard input
ros2 run turtlesim turtle_teleop_key

# Using tf2_echo tool to check the turtle pose in tf2
ros2 run tf2_ros tf2_echo world turtle1

# Creating a C++ package for learning_tf2_cpp
ros2 pkg create --build-type ament_cmake --dependencies geometry_msgs rclcpp tf2 tf2_ros turtlesim -- learning_tf2_cpp

# Downloading broadcaster C++ code
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_broadcaster.cpp

# Checking for missing dependencies
rosdep install -i --from-path src --rosdistro humble -y

# Building the package
colcon build --packages-select learning_tf2_cpp

# Sourcing setup files and running the launch file for turtle_tf2_demo
. install/setup.bash
ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py

# Controlling the turtle with keyboard input
ros2 run turtlesim turtle_teleop_key

# Using tf2_echo tool to check the turtle pose in tf2
ros2 run tf2_ros tf2_echo world turtle1

# Creating a C++ package for learning_tf2_cpp
ros2 pkg create --build-type ament_cmake --dependencies geometry_msgs rclcpp tf2 tf2_ros turtlesim -- learning_tf2_cpp

# Downloading listener C++ code
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_listener.cpp

# Checking for missing dependencies
rosdep install -i --from-path src --rosdistro humble -y

# Building the package
colcon build --packages-select learning_tf2_cpp

# Sourcing setup files
. install/setup.bash

# Running the full turtle demo
ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py

# Controlling the turtle with keyboard input
ros2 run turtlesim turtle_teleop_key

# Adding a frame named "carrot1" to the turtle model
ros2 pkg create --build-type ament_python learning_tf2_py

# Downloading fixed frame broadcaster Python code
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/fixed_frame_tf2_broadcaster.py

# Checking for missing dependencies
rosdep install -i --from-path src --rosdistro humble -y

# Building the package
colcon build --packages-select learning_tf2_py

# Sourcing setup files
. install/setup.bash

# Running the launch file with the fixed frame broadcaster
ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo.launch.py

# Running the launch file with a different target frame "carrot1"
ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo.launch.py target_frame:=carrot1

# Downloading dynamic frame broadcaster Python code
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/dynamic_frame_tf2_broadcaster.py

# Checking for missing dependencies
rosdep install -i --from-path src --rosdistro humble -y

# Sourcing setup files
. install/setup.bash

# Running the launch file with the dynamic frame broadcaster
ros2 launch learning_tf2_py turtle_tf2_dynamic_frame_demo.launch.py

# Running the turtle_tf2_demo launch file
ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py

# Running the turtle_tf2_demo launch file
ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py

# Running the turtle_tf2_fixed_frame_demo launch file
ros2 launch learning_tf2_cpp turtle_tf2_fixed_frame_demo.launch.py

# Running the turtle_tf2_fixed_frame_demo launch file
ros2 launch learning_tf2_cpp turtle_tf2_fixed_frame_demo.launch.py

# Running the turtle_tf2_demo launch file with debugging enabled
ros2 launch learning_tf2_cpp start_tf2_debug_demo.launch.py

# Controlling turtle1 with keyboard input
ros2 run turtlesim turtle_teleop_key

# Using tf2_echo to check if tf2 knows about the transform between turtle3 and turtle1
ros2 run tf2_ros tf2_echo turtle3 turtle1

# Using view_frames to get a graphical representation of frame relationships
ros2 run tf2_tools view_frames

# Running the turtle_tf2 demo with debugging
ros2 launch turtle_tf2 start_debug_demo.launch.py

# Checking if tf2 knows about the transform between turtle2 and turtle1
ros2 run tf2_ros tf2_echo turtle2 turtle1

# Using tf2_monitor to get statistics on transform timing
ros2 run tf2_ros tf2_monitor turtle2 turtle1

# Running the turtle_tf2_fixed_frame_demo launch file
ros2 launch turtle_tf2_fixed_frame_demo.launch.py

# Running the turtle_tf2_fixed_frame_demo launch file with advanced time-travel API
ros2 launch turtle_tf2_fixed_frame_demo.launch.py

# Running the turtle_tf2_fixed_frame_demo launch file
ros2 launch turtle_tf2_fixed_frame_demo.launch.py

# Running the turtle_tf2_fixed_frame_demo launch file with advanced time-travel API
ros2 launch turtle_tf2_fixed_frame_demo.launch.py

# Running the turtle_tf2_debug_demo launch file
ros2 launch learning_tf2_cpp start_tf2_debug_demo.launch.py

# Controlling turtle1 with keyboard input
ros2 run turtlesim turtle_teleop_key

# Using tf2_echo to check the transform between turtle2 and turtle1 at the current time
ros2 run tf2_ros tf2_echo turtle2 turtle1

# Running the turtle_tf2_debug_demo launch file with advanced time-travel API
ros2 launch turtle_tf2_debug_demo.launch.py

# Running the turtle_tf2_debug_demo launch file with advanced time-travel API
ros2 launch turtle_tf2_debug_demo.launch.py

# Running tests in ROS 2 from the command line
colcon test --ctest-args tests [package_selection_args]

# Viewing test results
colcon test-result --all

# Viewing detailed test results
colcon test-result --all --verbose

# Compiling and running C++ tests in debug mode
colcon build --cmake-clean-cache --mixin debug

# Running a C++ test using gdb
gdb -ex run ./build/rcl/test/test_logging

# Running tests with pytest
colcon test --packages-select <name-of-pkg> --pytest-args -k name_of_the_test_function

# Viewing pytest output while running tests
colcon test --event-handlers console_cohesion+

# Building a visual robot model from scratch and launching it in RViz
ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf

# Building a robot model with multiple shapes and launching it in RViz
ros2 launch urdf_tutorial display.launch.py model:=urdf/02-multipleshapes.urdf

# Building a robot model with origins for visual elements and launching it in RViz
ros2 launch urdf_tutorial display.launch.py model:=urdf/03-origins.urdf

# Building a robot model with materials for coloring and launching it in RViz
ros2 launch urdf_tutorial display.launch.py model:=urdf/04-materials.urdf

# Building a movable robot model using URDF and launching it in RViz
ros2 launch urdf_tutorial display.launch.py model:=urdf/06-flexible.urdf

# Using Xacro to simplify URDF code and launching the Xacro-based robot model in RViz
ros2 launch urdf_tutorial display.launch.py model:=urdf/08-macroed.urdf.xacro

# Launching a robot model with robot_state_publisher and visualizing it in RViz
ros2 launch urdf_tutorial display.launch.py model:=urdf/06-flexible.urdf


