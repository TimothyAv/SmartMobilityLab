# ROS 2 Action Server and Client for Order Delivery

This repository contains a ROS 2 action server and client for simulating an order delivery system. It utilizes the "OrderDelivery" action defined in the `delivery_action_interfaces` package.

## Prerequisites

- ROS 2 installed and set up.
- A ROS 2 workspace with the `delivery_action_interfaces` package containing the "OrderDelivery" action definition.

## Installation

Clone this repository into your ROS 2 workspace:

```bash
cd <your_ros2_workspace>/src
git clone <repository_url>

Build the workspace:
cd <your_ros2_workspace>
colcon build

Creating a README file on GitHub to describe your ROS 2 action server and client setup is a great idea. Here's a template for a README.md file that you can use. Be sure to replace the placeholders with your specific information:
![delivery_action_diagram](https://github.com/TimothyAv/SmartMobilityLab/assets/89968559/0e0442c8-ab98-4dce-a871-c4874aae27b4)

markdown
Copy code
# ROS 2 Action Server and Client for Order Delivery

This repository contains a ROS 2 action server and client for simulating an order delivery system. It utilizes the "OrderDelivery" action defined in the `delivery_action_interfaces` package.

## Prerequisites

- ROS 2 installed and set up.
- A ROS 2 workspace with the `delivery_action_interfaces` package containing the "OrderDelivery" action definition.

## Installation

Clone this repository into your ROS 2 workspace:

```bash
cd <your_ros2_workspace>/src
git clone <repository_url>
Build the workspace:

bash
Copy code
cd <your_ros2_workspace>
colcon build
Usage
Running the Action Server (C++)
To run the C++ action server for order delivery:

bash
Copy code
ros2 run <your_package_name> order_delivery_server
The server will be ready to accept delivery orders.

Running the Action Client (C++)
To run the C++ action client to send a delivery request:

bash
Copy code
ros2 run <your_package_name> order_delivery_client
The client will send a delivery request to the action server and display the result.

Running the Action Client (Python)
To run the Python action client:

bash
Copy code
ros2 run <your_package_name> order_delivery_client.py
This Python client will also send a delivery request to the action server and display the result.

Action Definition
The "OrderDelivery" action definition is located in the delivery_action_interfaces package. It consists of three parts: Request, Result, and Feedback.

Request: Specifies the order to be delivered and the destination.
Result: Indicates whether the delivery was successful and provides a message.
Feedback: Includes the progress of the delivery.


