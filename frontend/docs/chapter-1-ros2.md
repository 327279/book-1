---
title: "Chapter 1 - The Robotic Nervous System (ROS 2)"
sidebar_position: 1
---

# The Robotic Nervous System (ROS 2)

## Introduction to ROS 2

Robot Operating System 2 (ROS 2) provides the communication framework for robotic systems. Unlike the original ROS, ROS 2 is designed for production environments with improved security, real-time capabilities, and multi-robot systems.

ROS 2 uses the Data Distribution Service (DDS) for communication between nodes. This provides a middleware that enables different parts of a robotic system to communicate reliably, even when written in different programming languages or running on different machines.

## Nodes, Topics, and Services Architecture

### Nodes

Nodes are the fundamental building blocks of ROS 2. A node is an executable that uses ROS 2 to communicate with other nodes. Each node should perform a specific function and communicate with other nodes to achieve complex robotic behaviors.

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

### Topics

Topics enable asynchronous communication through a publish-subscribe model. Publishers send messages to a topic, and subscribers receive messages from that topic. Multiple nodes can publish to the same topic or subscribe to it.

The publish-subscribe pattern allows for decoupling of nodes - publishers don't need to know which subscribers exist, and subscribers don't need to know which publishers exist.

### Services

Services provide synchronous request-response communication. A client sends a request to a service, and the service returns a response. This is useful for tasks that require immediate results.

## Bridging Python Agents to ROS Controllers using rclpy

The `rclpy` library is the Python client library for ROS 2. It provides the interface for Python programs to interact with the ROS 2 system.

### Creating a Simple Publisher

To bridge Python agents with ROS controllers, you'll often need to publish data from your AI system to ROS controllers:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile

class AIBridgePublisher(Node):
    def __init__(self):
        super().__init__('ai_bridge_publisher')
        qos_profile = QoSProfile(depth=10)
        self.ai_publisher = self.create_publisher(String, 'ai_commands', qos_profile)

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.ai_publisher.publish(msg)
        self.get_logger().info(f'AI Command Sent: {command}')
```

### Creating a Simple Subscriber

You can also receive data from ROS controllers to inform your AI decisions:

```python
class AIBridgeSubscriber(Node):
    def __init__(self):
        super().__init__('ai_bridge_subscriber')
        qos_profile = QoSProfile(depth=10)
        self.ai_subscriber = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Robot Status Received: {msg.data}')
        # Process the status in your AI system
        self.process_robot_status(msg.data)
```

## URDF (Unified Robot Description Format) for Humanoids

URDF is an XML format for representing a robot model. For humanoid robots, URDF describes the robot's physical structure, including links, joints, and their relationships.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint connecting base to torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>
</robot>
```

URDF files can be complex for humanoid robots with many degrees of freedom. For humanoid robotics, you'll often need to define:
- Links for each body part (head, torso, arms, legs)
- Joints connecting these parts
- Physical properties (mass, inertia)
- Visual and collision models
- Transmission elements for actuators

## Summary

This chapter covered the fundamentals of ROS 2, which serves as the "nervous system" for robotic systems. You learned about:

- The node-based architecture of ROS 2
- Communication patterns using topics and services
- How to bridge Python AI agents with ROS controllers
- The basics of robot description using URDF

In the next chapter, we'll explore simulation environments where you can test these concepts safely before applying them to real hardware.