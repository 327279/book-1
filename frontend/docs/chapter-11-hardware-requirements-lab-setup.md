---
title: "Chapter 11 - Hardware Requirements"
sidebar_position: 11
---

# Hardware Requirements and Lab Setup

::warning
This course is technically demanding. It sits at the intersection of Physics Simulation, Visual Perception, and Generative AI.
::

## 1. The "Digital Twin" Workstation (Required per Student)

This is the most critical component. NVIDIA Isaac Sim is an Omniverse application that requires "RTX" (Ray Tracing) capabilities. Standard laptops will not work.

*   **GPU (The Bottleneck)**: NVIDIA RTX 4070 Ti (12GB VRAM) or higher.
    *   *Ideal*: RTX 3090 or 4090 (24GB VRAM).
*   **CPU**: Intel Core i7 (13th Gen+) or AMD Ryzen 9.
*   **RAM**: 64 GB DDR5 (32 GB is the absolute minimum).
*   **OS**: Ubuntu 22.04 LTS.

## 2. The "Physical AI" Edge Kit

Since a full humanoid robot is expensive, students learn "Physical AI" by setting up the nervous system on a desk before deploying it to a robot.

*   **The Brain**: NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB).
*   **The Eyes (Vision)**: Intel RealSense D435i or D455.
*   **The Inner Ear (Balance)**: Generic USB IMU (BNO055).
*   **Voice Interface**: ReSpeaker or generic USB mic for Whisper integration.

## 3. The Robot Lab (Options)

### Option A: The "Proxy" Approach (Recommended)
Use a quadruped (dog) or robotic arm.
*   **Robot**: Unitree Go2 Edu (~$1,800 - $3,000).

### Option B: The "Miniature Humanoid" Approach
*   **Robot**: Unitree G1 (~$16k) or Robotis OP3 (~$12k).
*   **Budget**: Hiwonder TonyPi Pro (~$600) - *Limited capability.*

### Option C: The "Premium" Lab
*   **Robot**: Unitree G1 Humanoid.
*   **Why**: Capable of dynamic walking and has an open SDK.

## 4. Cost Breakdown (Economy Student Kit)

| Component | Model | Price (Approx.) |
| :--- | :--- | :--- |
| **The Brain** | NVIDIA Jetson Orin Nano Super Dev Kit (8GB) | $249 |
| **The Eyes** | Intel RealSense D435i | $349 |
| **The Ears** | ReSpeaker USB Mic Array v2.0 | $69 |
| **Power/Misc** | SD Card (128GB) + Jumper Wires | $30 |
| **TOTAL** | | **~$700** |

::info
**Latency Warning**: Simulating in the cloud works, but controlling a real robot from the cloud is dangerous due to latency. We recommend: **Train in Cloud -> Flash to Local Jetson**.
::