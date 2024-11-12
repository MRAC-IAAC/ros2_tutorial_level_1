# ROS2 Tutorial - Level 1

Welcome to the ROS2 tutorial using `turtlesim`. This guide will help you understand key concepts such as the variables `DOMAIN ID` and `ROS_LOCALHOST_ONLY`, as well as main concepts: packages, nodes, topics, publishers, and subscribers.

This tutorial has been designed by MRAC Faculty [Marita Georganta](https://www.linkedin.com/in/marita-georganta/) for MRAC02 students or anyone with some understanding of Python and ROS. The contents are characterised as Level 1, and more advanced concepts are included in the Level 2 and Level 3 tutorials.

The fundamentals of ROS2 communication architecture and DDS can be found in the ROS2 Introduction.

---

## Prerequisites

- Make sure you have built and run the provided Docker container and have launched `terminator`. ROS2 will be installed in the container when the image is built.
- Familiarity with basic Python programming is recommended.

---

## Table of Contents

0. [Prerequisites](#prerequisites)
1. [Understanding Key Environment Variables](#1-understanding-key-environment-variables)
   - [DOMAIN ID](#11-domain-id)
   - [ROS_LOCALHOST_ONLY](#12-ros_localhost_only)
2. [Using the Turtlesim Package](#2-using-the-turtlesim-package)
   - [What is a Package?](#21-what-is-a-package)
   - [What is a Node?](#22-what-is-a-node)
3. [Running a Node](#3-running-a-node)
   - [Running the `turtlesim_node`](#running-the-turtlesim_node)
4. [Understanding Topics](#4-understanding-topics)
   - [Listing Active Topics](#listing-active-topics)
   - [Getting Information About a Topic](#getting-information-about-a-topic)
5. [Running All Nodes in the Turtlesim Package](#5-running-all-nodes-in-the-turtlesim-package)
   - [Turtlesim Node](#51-turtlesim-node)
   - [Turtlesim Teleop Node](#52-turtlesim-teleop-node)
6. [Publishers and Subscribers](#6-publishers-and-subscribers)
   - [What is a Publisher?](#61-what-is-a-publisher)
   - [What is a Subscriber?](#62-what-is-a-subscriber)
   - [Example: Publishing to `/turtle1/cmd_vel`](#example-publishing-to-turtle1cmd_vel)
7. [Python Example: Publisher and Subscriber](#7-python-example-publisher-and-subscriber)
   - [Writing a Publisher in Python](#71-writing-a-publisher-in-python)
   - [Writing a Subscriber in Python](#72-writing-a-subscriber-in-python)
8. [Summary](#summary)

---

## 1. Understanding Key Environment Variables

The `DOMAIN_ID` and `ROS_LOCALHOST_ONLY` are important variables that define the communication 'channel' and 'visibility' of your local machine, thus robot.

Before you dive into what these variables are, note that since this tutorial uses Docker, the variables have been already in the Dockerfile. Feel free to change them accordingly later, and remember to re-build the image everytime you make a change. You can see the values set for `DOMAIN ID` and `ROS_LOCALHOST_ONLY` if you use the command `printenv | grep -i ROS` at anytime.

### 1.1. DOMAIN ID

The `DOMAIN ID` is an environment variable used by ROS2's underlying communication layer, DDS (Data Distribution Service), to separate communication networks or 'channels'.

- **Purpose**: It allows you to run multiple independent ROS2 networks on the same physical network, preventing them from interfering with each other.
- **Usage**: The `DOMAIN ID` is an integer, and setting different values for different ROS2 applications isolates their communications.
- **Example**: If you have two robots running ROS2 and you want them to operate in separate networks, set `DOMAIN_ID=1` for one and `DOMAIN_ID=2` for the other. However, in the case you do want them to operate in the same network use the same `DOMAIN_ID`.

```bash
export DOMAIN_ID=1  # Setting the DOMAIN ID to 1
```

### 1.2. ROS_LOCALHOST_ONLY

`ROS_LOCALHOST_ONLY` restricts all ROS2 communication to the local machine.

- **Purpose**: When set to `1`, ROS2 nodes will only communicate on `localhost` (127.0.0.1). This is useful for testing and development.
- **Default Value**: It is `0` by default, meaning communication can occur over the network.
- **Example**: To restrict communication to your local machine:

```bash
export ROS_LOCALHOST_ONLY=1  # Restrict communication to localhost
```

## 2. Using the Turtlesim Package

### 2.1. What is a Package?

In ROS2, a **package** is a collection of files and resources that work together to provide functionality, such as libraries, executables (nodes), configuration files, and more.

- **Example**: `turtlesim` is a package that provides a simple simulator with a turtle in a 2D world.

### 2.2. What is a Node?

A **node** is an executable that communicates with other nodes using ROS2. Nodes perform computations, exchange data, and are the building blocks of a ROS2 application.

- **Example**: In the `turtlesim` package, `turtlesim_node` is a node that opens a window with a turtle simulation.

---

## 3. Running a Node

To run a node in ROS2, use the command:

```bash
ros2 run <package_name> <node_name>
```

### 3.1. Running the `turtlesim_node`
```bash
ros2 run turtlesim turtlesim_node
```
- **Explanation**: This command starts the `turtlesim_node`, opening a window where you can see a turtle.

### 3.2. Active Nodes

To see all currently active nodes, use the command:

```bash
ros2 node list
```

### 3.3. Available Nodes

To see all executables (nodes) available within a package, use the command:

```bash
ros2 pkg executables <package_name>
```

### 3.4. Info on Nodes

To see more information on a specific active node, use the command:
```bash
ros2 node info <node_name>
```

---

## 4. Understanding Topics

**Topics** are named channels used for communication between nodes. Nodes can publish messages to a topic or subscribe to receive messages from a topic.

### Listing Active Topics

To see all active topics in your ROS2 environment, run:

```bash
ros2 topic list
```

- **Example Output**: You might see topics like `/turtle1/cmd_vel` and `/turtle1/pose`.

### Getting Information About a Topic

To get details about a specific topic, such as the message type used, run:

```bash
ros2 topic info /turtle1/cmd_vel
```

- **Explanation**: This provides information about the `/turtle1/cmd_vel` topic, which is used to control the turtle's movement.

---

## 5. Running All Nodes in the Turtlesim Package

In the `turtlesim` package, there are multiple nodes that you can run to see different functionalities.

### 5.1. Turtlesim Node

To run the `turtlesim_node`, which creates a graphical window with a turtle that you can control, use:

`ros2 run turtlesim turtlesim_node`

This node is the core of the turtlesim package and creates a 2D simulation of a turtle in a window.

### 5.2. Turtlesim Teleop Node

To control the turtle interactively, run the `turtlesim_teleop` node:

`ros2 run turtlesim turtle_teleop_key`

- **Explanation**: This node listens for keyboard input and sends commands to the `turtlesim_node` to move the turtle around. You can use the arrow keys to control its movement.

#### Investigate

Using the things you have already learned, examine the `turtlesim` package and see what other nodes there are and how they interact with each other.

---

## 6. Publishers and Subscribers

In ROS2, nodes communicate with each other using **publishers** and **subscribers** over topics.

### 6.1. What is a Publisher?

A **publisher** is a node that sends messages to a specific topic.

- **Example**: The `turtlesim_node` publishes messages to the `/turtle1/pose` topic, which contains information about the turtle’s position.

### 6.2. What is a Subscriber?

A **subscriber** is a node that receives messages from a topic.

- **Example**: A node that subscribes to `/turtle1/pose` would receive the position and orientation data of the turtle.

### 6.3. Example: Publishing to `/turtle1/cmd_vel`

The `/turtle1/cmd_vel` topic is used to send commands to control the turtle's velocity. You can publish messages to this topic to move the turtle.

To publish a message to `/turtle1/cmd_vel` using the ROS2 CLI, run:

`ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}"`

- **Explanation**: This command publishes a velocity message to move the turtle forward with a linear velocity of `2.0` and a rotation (angular velocity) of `1.8` radians per second.

---

## 7. Python Example: Publisher and Subscriber

In this section, we'll write Python scripts to create a **publisher** and a **subscriber** for the `/turtle1/cmd_vel` topic.

### 7.1. Writing a Publisher in Python

To create a simple publisher that sends velocity commands to `/turtle1/cmd_vel`, follow these steps:

1. Create a new Python file called `turtle_publisher.py`.
2. Add the following code:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtlePublisher(Node):
    def __init__(self):
        super().__init__('turtle_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  # Publish every 1 second
        self.get_logger().info('Turtle Publisher started.')

    def publish_message(self):
        msg = Twist()
        msg.linear.x = 2.0  # Move forward at 2 m/s
        msg.angular.z = 1.8  # Rotate at 1.8 radians per second
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    turtle_publisher = TurtlePublisher()
    rclpy.spin(turtle_publisher)

    turtle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- **Explanation**: This Python script creates a ROS2 node that publishes `Twist` messages to `/turtle1/cmd_vel`. The turtle moves forward and rotates every second.

### 7.2. Writing a Subscriber in Python

To create a simple subscriber that listens to the `/turtle1/pose`, follow these steps:

1. Create a new Python file called `turtle_subscriber.py`.
2. Add the following code:

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtleSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10
        )
        self.get_logger().info('Turtle Subscriber started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: Position ({msg.x}, {msg.y}), Rotation: {msg.theta}')

def main(args=None):
    rclpy.init(args=args)
    turtle_subscriber = TurtleSubscriber()
    rclpy.spin(turtle_subscriber)

    turtle_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- **Explanation**: This Python script creates a ROS2 node that subscribes to `/turtle1/pose` and prints the turtle's position and rotation.

---

## 8. Summary

In this tutorial, we covered the following topics:

- **DOMAIN ID** and **ROS_LOCALHOST_ONLY** environment variables.
- How to run and understand the ROS2 `turtlesim` package.
- How to create and run ROS2 nodes and understand topics.
- The concept of **publishers** and **subscribers** in ROS2 and how to implement them using Python.
- How to create a simple publisher and subscriber to control the turtle's movement and receive its position.

By following this tutorial, you should now have a good understanding of ROS2 core concepts, including nodes, topics, and communication between nodes. You can further extend this knowledge by experimenting with different ROS2 packages and nodes.

---

## 9. Challenge

Complete the `EXERCISE.md` and push you solution. If you are confident you have done it well, open a PR and your solution will be included in the repo.