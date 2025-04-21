---
title: "Ackermann Steering Vehicle Simulation"
date: 2025-04-20 10:00:00
description: Modeling and simulation of a vehicle with Ackermann steering geometry using the Gazebo Simulator and the ROS 2 framework for robotics applications.
author: lucasmazz
keywords: robot, simulation, gazebo, ros, robotic simulation, autonomous robots, Gazebo simulation, ROS 2, ROS robot simulation, computer vision in robotics, self-driving car simulation, autonomous vehicle technology, simulation environments for robots, real-time robotics simulation, robotic perception
---

This article presents the development and simulation of an Ackermann steering vehicle using the Gazebo Harmonic simulator and the ROS 2 Jazzy Jalisco robotics framework. The vehicle model includes steering angle and velocity control, along with an embedded front camera that streams live images for vision-based tasks. This setup provides a safe environment for testing autonomous driving without the risks associated with real-world trials and serves as a platform for researchers and developers aiming to advance mobile robots algorithms.​

## Introduction 

When developing mobile robots, safety is critical, especially during the early stages of design and testing. Testing directly on real hardware can be risky and expensive. A single mistake can lead to accidents, damage to valuable equipment, or even personal injury. That’s where simulation comes in.

Beyond safety, simulation also plays a key role in training artificial intelligence systems. By creating virtual environments that mimic real-world physics and constraints, we can develop and refine control algorithms and train machine learning models for tasks like path planning, navigation, obstacle avoidance, and localization. 

In this article, we'll explore the modeling and simulation of a vehicle using Ackermann steering geometry, a steering mechanism widely used in conventional vehicles. Simulating this steering configuration is needed to ensure realistic movement and precise maneuvering. This project was developed using the Gazebo Harmonic simulator and the ROS 2 Jazzy Jalisco robotics framework, providing a flexible environment for testing and development.

The source code used in this project is available on [GitHub](https://github.com/workabotic/gazebo_ackermann_steering_vehicle). Feel free to explore it, use it in your own projects, or contribute to its development.


## Requirements

While other versions of the tools and software may work, adjustments might be required. To ensure proper functionality, it’s recommended to use the versions listed below. Also, ensure that the environment is set up correctly to avoid any issues.

### Recommended Software Versions

To get started with the project, you will need to install the following software on your computer. Below are the required tools and installation instructions:

- **Linux Ubuntu 24.04**: Download and install from the official [Ubuntu website](https://ubuntu.com/blog/tag/ubuntu-24-04-lts).
- **ROS 2 Jazzy Jalisco**: Follow the instructions on the [ROS 2 release page](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html) to install.
- **Gazebo Harmonic**: Use the [Gazebo website](https://gazebosim.org/docs/harmonic/getstarted/) guide to install the simulator.

If your operating system is not Linux Ubuntu 24.04, and you still want to use it to replicate this project, you may need to use a container, such as [Docker](https://www.docker.com/), to set up the environment.

### Installing ROS Packages

Once ROS 2 Jazzy Jalisco is installed, add the required packages by running the following command in your terminal. These packages are necessary for controlling the robot, bridging Gazebo and ROS 2, and managing robot states and configurations:

```bash
sudo apt install -y \
     ros-jazzy-ros2-controllers \
     ros-jazzy-gz-ros2-control \
     ros-jazzy-ros-gz \
     ros-jazzy-ros-gz-bridge \
     ros-jazzy-joint-state-publisher \
     ros-jazzy-robot-state-publisher \
     ros-jazzy-xacro \
     ros-jazzy-joy                           
```

### Setup the Project

Set up a ROS 2 workspace on your computer if you don't have one. For detailed instructions, refer to the [ROS 2 workspace tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html). Here's an example:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/workspace/src
```

Create a ROS 2 package named **gazebo_ackermann_steering_vehicle** that will use **CMake** as its build system. This package will handle the vehicle's modeling and control, as well as launching the vehicle into Gazebo worlds. You can follow the [ROS 2 package tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) for guidance.  For example, the code below shows a script you can run to create the package and set things up as needed:

```bash
cd ~/workspace/src
ros2 pkg create --build-type ament_cmake gazebo_ackermann_steering_vehicle
```

Create the following directories inside the **gazebo_ackermann_steering_vehicle** package:

- **config**: This folder will store configuration files, including parameters for the vehicle and the Gazebo bridge configuration.
- **launch**: This folder will contain the launch files required to start the simulation, load the vehicle model, and set up the required nodes.
- **model**: This folder will house the robot description files, such as the .xacro or .urdf files, defining the Ackermann steering vehicle's structure and properties.

These directories help organize the package and separate configurations, execution scripts, and model definitions for development and maintenance. For example, you can run the following script to create these directories within the package:

```bash
cd ~/workspace/src/gazebo_ackermann_steering_vehicle
mkdir -p config launch model
```

Therefore, the **gazebo_ackermann_steering_vehicle** package should include the following directory structure for proper organization:

```text
gazebo_ackermann_steering_vehicle/
├── config/
├── include/
├── launch/
├── model/
├── src/
├── CMakeLists.txt
├── package.xml
```

Now, update the **package.xml** to include the project dependencies as follows:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>gazebo_ackermann_steering_vehicle</name>
  <version>0.0.0</version>
  <description>A ROS 2 package for simulating an Ackermann steering vehicle. Includes robot modeling, control, and Gazebo simulation integration.</description>
  <maintainer email="lucasrmazzetto@gmail.com">Lucas Mazzetto</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>gz_ros2_control</exec_depend>
  <exec_depend>ros2_control</exec_depend>
  <exec_depend>joy</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
Also, update the **CMakeLists.txt** file to include the necessary dependencies as well:

```cmake
cmake_minimum_required(VERSION 3.8)
project(gazebo_ackermann_steering_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

install(
  DIRECTORY launch model config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

With the environment set up, we're now ready to start modeling the vehicle and implementing the control system.

## Methods

This section details the steps to simulate a vehicle with Ackermann steering geometry in Gazebo Harmonic with ROS 2. It covers modeling the vehicle's physical and dynamic properties, implementing ROS 2 nodes to manage the steering angles and wheel velocities, and adding a video game joystick interface for manual operation.

### Vehicle Modeling

The vehicle model can be designed to be easy to customize and adapt to potential changes throughout the project lifecycle. This project features a scaled version of a full-sized vehicle equipped with an embedded camera. By parametrizing the model's dimensions, it is easily modified for the current project or reused in future applications. The following image shows the vehicle model with these parametrized values:

![Vehicle Parameters]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/vehicle_model_parameters.webp)
*Figure 1 — Vehicle Model Parameters.*

To define its physical and kinematic properties, the model can be represented using the Unified Robot Description Format (URDF), an XML-based format that specifies components like links, joints, and collision geometries. However, URDF files can become verbose and difficult to maintain due to their lack of support for reusable components. To address this, Xacro (XML Macros) is used to generate the URDF, enabling reusable macros, parameterized definitions, and constants. This approach simplifies maintenance, reduces redundancy, and ensures the model remains flexible for future needs.

As previously mentioned, parameters could be used to define the model’s dimensions, but they can also be used for computing the Ackermann steering angles and rear wheel velocities. For consistency and easy access, these parameters can be defined in a shared format used by both the Xacro files and ROS 2 nodes. In this project, create a file named **params.yaml** inside the package’s **config/** directory. Then, add the following content to define the vehicle’s dimensions, kinematic and dynamic properties, and settings for the onboard camera:

```yaml
/**:
  ros__parameters:
    # Body params
    body_length: 0.3 # Length of the vehicle's body [m]
    body_width: 0.18 # Width of the vehicle's body [m]
    body_height: 0.05 # Height of the vehicle's body [m]
    body_density: 7850.0 # Density of the vehicle's body material, e.g., steel [kg/m^3]

    # Wheel params
    wheel_radius: 0.04 # Radius of each wheel [m]
    wheel_width: 0.02 # Width of each wheel [m]
    wheel_density: 900.0 # Density of the wheel material, e.g., rubber [kg/m^3]

    # Kinematics and dynamics params
    max_steering_angle: 0.6108652 # Maximum steering angle of the vehicle [rad]
    max_steering_angular_velocity: 1.570796 # Maximum steering angular velocity [rad/s]
    max_steering_effort: 1.0 # Maximum steering torque [Nm]
    max_velocity: 2.0 # Maximum wheel velocity [m/s]
    max_effort: 10.0 # Maximum wheel torque [Nm]

    # Camera and image params
    camera_box_size: 0.05 # Size of the camera enclosure [m]
    camera_stick_size: 0.02 # Size of the camera stick [m]
    camera_height: 0.2 # Height of the camera above the body_link [m]
    camera_pitch: 0.698131 # Pitch angle of the camera relative to body_link [rad]
    camera_fov: 1.3962634 # Field of view of the camera [rad]
    camera_fps: 30 # Frames per second for camera capture [Hz]
    image_width: 640 # Width of the camera's image output [pixels]
    image_height: 480 # Height of the camera's image output [pixels]
```

Create a Xacro file named **vehicle.xacro** in the **model/** directory to define the vehicle's structure and physical properties. It includes details such as mass, inertia, collision geometry, friction, damping, wheel placement, and a mechanism that models front-wheel steering with rear-wheel propulsion. Parameters from **config/params.yaml** will be passed to the model at runtime via the launch file. The file also integrates and configures plugins to interface with external packages and the Gazebo simulator. The contents are shown below:

```xml
<?xml version="1.0"?>

<robot name="ackermann_steering_vehicle" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Arguments -->
  <xacro:arg name="body_length" default="2.0" />
  <xacro:arg name="body_width" default="1.0" />
  <xacro:arg name="body_height" default="0.5" />
  <xacro:arg name="body_density" default="1000.0" />
  <xacro:arg name="wheel_radius" default="0.3" />
  <xacro:arg name="wheel_width" default="0.1" />
  <xacro:arg name="wheel_density" default="1000.0" />
  <xacro:arg name="max_steering_angle" default="0.6108" />
  <xacro:arg name="max_steering_angular_velocity" default="1.570796" />
  <xacro:arg name="max_steering_effort" default="1.0" />
  <xacro:arg name="max_velocity" default="1.0" />
  <xacro:arg name="max_effort" default="10.0" />
  <xacro:arg name="camera_box_size" default="0.1" />
  <xacro:arg name="camera_stick_size" default="0.2" />
  <xacro:arg name="camera_height" default="1.5" />
  <xacro:arg name="camera_pitch" default="0.1" />
  <xacro:arg name="camera_fov" default="90" />
  <xacro:arg name="camera_fps" default="30" />
  <xacro:arg name="image_width" default="640" />
  <xacro:arg name="image_height" default="480" />

  <!-- Mathematical Constants -->
  <xacro:property name="PI" value="3.14159265"/>

  <!-- Vehicle Properties -->
  <xacro:property name="body_length" value="$(arg body_length)"/>
  <xacro:property name="body_width" value="$(arg body_width)"/>
  <xacro:property name="body_height" value="$(arg body_height)"/>
  <xacro:property name="body_density" value="$(arg body_density)"/>
  <xacro:property name="wheel_radius" value="$(arg wheel_radius)"/>
  <xacro:property name="wheel_width" value="$(arg wheel_width)"/>
  <xacro:property name="wheel_density" value="$(arg wheel_density)"/>
  <xacro:property name="max_steering_angle" value="$(arg max_steering_angle)"/>
  <xacro:property name="max_steering_angular_velocity" value="$(arg max_steering_angular_velocity)"/>
  <xacro:property name="max_steering_effort" value="$(arg max_steering_effort)"/>
  <xacro:property name="max_velocity" value="$(arg max_velocity)"/>
  <xacro:property name="max_effort" value="$(arg max_effort)"/>
  <xacro:property name="camera_box_size" value="$(arg camera_box_size)"/>
  <xacro:property name="camera_stick_size" value="$(arg camera_stick_size)"/>
  <xacro:property name="camera_height" value="$(arg camera_height)"/>
  <xacro:property name="camera_pitch" value="$(arg camera_pitch)"/>
  <xacro:property name="camera_fov" value="$(arg camera_fov)"/>
  <xacro:property name="camera_fps" value="$(arg camera_fps)"/>
  <xacro:property name="image_width" value="$(arg image_width)"/>
  <xacro:property name="image_height" value="$(arg image_height)"/>
  
  <xacro:property name="body_mass" value="${body_density * body_length * body_height * body_width}"/>
  <xacro:property name="body_inertia_x" value="${1.0/12.0 * body_mass * (body_height*body_height + body_width*body_width)}"/>
  <xacro:property name="body_inertia_y" value="${1.0/12.0 * body_mass * (body_length*body_length + body_height*body_height)}"/>
  <xacro:property name="body_inertia_z" value="${1.0/12.0 * body_mass * (body_length*body_length + body_width*body_width)}"/>
  <xacro:property name="wheel_separation" value="${body_width + wheel_width}"/>
  <xacro:property name="wheel_offset" value="${body_length/2 - wheel_radius}"/>
  <xacro:property name="wheel_mass" value="${wheel_density * PI * wheel_radius * wheel_radius * wheel_width}"/>
  <xacro:property name="wheel_inertia_x" value="${1.0/12.0 * wheel_mass * (3*wheel_radius*wheel_radius + wheel_width*wheel_width)}"/>
  <xacro:property name="wheel_inertia_y" value="${1.0/12.0 * wheel_mass * (3*wheel_radius*wheel_radius + wheel_width*wheel_width)}"/>
  <xacro:property name="wheel_inertia_z" value="${1.0/2.0 * wheel_mass * wheel_radius * wheel_radius}"/>
  <xacro:property name="max_wheel_angular_velocity" value="${max_velocity / wheel_radius}"/>

  <!-- Links -->

  <!-- Body Link -->
  <link name="body_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
      <material name="body_material">
        <color rgba="0.25 0.25 0.25 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${body_mass}"/>
      <inertia ixx="${body_inertia_x}" ixy="0.0" ixz="0.0" iyy="${body_inertia_y}" iyz="0" izz="${body_inertia_z}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </inertial>
  </link>

  <!-- Front Left Wheel -->
  <link name="front_left_wheel_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="wheel_material">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${wheel_mass}"/>
      <inertia ixx="${wheel_inertia_x}" ixy="0.0" ixz="0.0" iyy="${wheel_inertia_y}" iyz="0" izz="${wheel_inertia_z}"/>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
    </inertial>
  </link>

    <!-- Front Left Wheel Steering -->
  <link name="front_left_steering_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
      <geometry>
        <cylinder length="0.001" radius="0.01"/>
      </geometry>
      <material name="wheel_material">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
      <geometry>
        <cylinder length="0.001" radius="0.01"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
    </inertial>
  </link>

  <!-- Front Right Wheel -->
  <link name="front_right_wheel_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="wheel_material">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${wheel_mass}"/>
      <inertia ixx="${wheel_inertia_x}" ixy="0.0" ixz="0.0" iyy="${wheel_inertia_y}" iyz="0" izz="${wheel_inertia_z}"/>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
    </inertial>
  </link>

  <!-- Front Right Wheel Steering -->
  <link name="front_right_steering_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
      <geometry>
        <cylinder length="0.001" radius="0.01"/>
      </geometry>
      <material name="wheel_material">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
      <geometry>
        <cylinder length="0.001" radius="0.01"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
    </inertial>
  </link>

  <!-- Rear Left Wheel -->
  <link name="rear_left_wheel_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="wheel_material">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="${wheel_inertia_x}" ixy="0.0" ixz="0.0" iyy="${wheel_inertia_y}" iyz="0" izz="${wheel_inertia_z}"/>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
    </inertial>
  </link>

  <!-- Rear Right Wheel -->
  <link name="rear_right_wheel_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="wheel_material">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="${wheel_inertia_x}" ixy="0.0" ixz="0.0" iyy="${wheel_inertia_y}" iyz="0" izz="${wheel_inertia_z}"/>
      <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
    </inertial>
  </link>

  <!-- Camera Stick Link -->
  <link name="camera_stick_link">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${camera_stick_size} ${camera_stick_size} ${camera_height}"/>
      </geometry>
    </collision>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${camera_stick_size} ${camera_stick_size} ${camera_height}"/>
      </geometry>

      <material name="camera_stick_material">
        <color rgba="0.25 0.25 0.25 1.0"/>
      </material>
    </visual>

    <inertial>
      <mass value="1e-5" />
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
    </inertial>
  </link>

  <!-- Camera Link -->
  <link name="camera_link">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${camera_box_size} ${camera_box_size} ${camera_box_size}"/>
      </geometry>
    </collision>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${camera_box_size} ${camera_box_size} ${camera_box_size}"/>
      </geometry>

      <material name="camera_material">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>

    <inertial>
      <mass value="1e-5" />
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
    </inertial>
  </link>

  <!-- Joints -->

  <!-- Front Left Steering Joint -->
  <joint name="front_left_steering_joint" type="revolute">
    <origin xyz="${wheel_offset} ${wheel_separation/2} -${wheel_radius/2}" rpy="0 0 0"/>
    <parent link="body_link"/>
    <child link="front_left_steering_link"/>
    <axis xyz="0 0 1"/>
    <limit effort="${max_steering_effort}" lower="${-max_steering_angle}" upper="${max_steering_angle}" velocity="${max_steering_angular_velocity}"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- Front Left Wheel Joint -->
  <joint name="front_left_wheel_joint" type="continuous">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="front_left_steering_link"/>
    <child link="front_left_wheel_link"/>
    <axis xyz="0 1 0"/> 
    <limit effort="${max_effort}" velocity="${max_wheel_angular_velocity}"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- Front Right Steering Joint -->
  <joint name="front_right_steering_joint" type="revolute">
    <origin xyz="${wheel_offset} ${-wheel_separation/2} -${wheel_radius/2}" rpy="0 0 0"/>
    <parent link="body_link"/>
    <child link="front_right_steering_link"/>
    <axis xyz="0 0 1"/>
    <limit effort="${max_steering_effort}" lower="${-max_steering_angle}" upper="${max_steering_angle}" velocity="${max_steering_angular_velocity}"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- Front Right Wheel Joint -->
  <joint name="front_right_wheel_joint" type="continuous">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="front_right_steering_link"/>
    <child link="front_right_wheel_link"/>
    <axis xyz="0 1 0"/> 
    <limit effort="${max_effort}" velocity="${max_wheel_angular_velocity}"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- Rear Left Wheel Joint -->
  <joint name="rear_left_wheel_joint" type="continuous">
    <origin xyz="-${wheel_offset} ${wheel_separation/2} -${wheel_radius/2}" rpy="0 0 0"/>
    <parent link="body_link"/>
    <child link="rear_left_wheel_link"/>
    <axis xyz="0 1 0"/>
    <limit effort="${max_effort}" velocity="${max_wheel_angular_velocity}"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- Rear Right Wheel Joint -->
  <joint name="rear_right_wheel_joint" type="continuous">
    <origin xyz="-${wheel_offset} ${-wheel_separation/2} -${wheel_radius/2}" rpy="0 0 0"/>
    <parent link="body_link"/>
    <child link="rear_right_wheel_link"/>
    <axis xyz="0 1 0"/>
    <limit effort="${max_effort}" velocity="${max_wheel_angular_velocity}"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- Camera Stick Joint -->
  <joint name="camera_stick_joint" type="fixed">
    <axis xyz="0 1 0" />
    <origin xyz="${body_length/2 - camera_stick_size/2} ${camera_box_size/2 + camera_stick_size/2} ${camera_height/2}" rpy="0.0 0.0 0.0"/>
    <parent link="body_link"/>
    <child link="camera_stick_link"/>
  </joint>

  <!-- Camera Joint -->
  <joint name="camera_joint" type="fixed">
    <axis xyz="0 1 0" />
    <origin xyz="0.0 ${-camera_box_size/2 - camera_stick_size/2} ${camera_height/2}" rpy="0 ${camera_pitch} 0"/>
    <origin xyz="0.0 0.0 0.0" rpy="0 ${camera_pitch} 0"/>
    <parent link="camera_stick_link"/>
    <child link="camera_link"/>
  </joint>

  <!-- Gazebo Parameters -->
  <gazebo reference="body_link">
    <mu1>0.0001</mu1>
    <mu2>0.0001</mu2>
  </gazebo>

  <gazebo reference="front_right_wheel_link">
    <mu1>100.0</mu1>
    <mu2>100.0</mu2>
  </gazebo>

  <gazebo reference="front_left_wheel_link">
    <mu1>100.0</mu1>
    <mu2>100.0</mu2>
  </gazebo>
    
  <gazebo reference="rear_right_wheel_link">
    <mu1>100.0</mu1>
    <mu2>100.0</mu2>
  </gazebo>

  <gazebo reference="rear_left_wheel_link">
    <mu1>100.0</mu1>
    <mu2>100.0</mu2>
  </gazebo>

  <!-- Gazebo Control -->
  <ros2_control name="control" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
          
    <joint name="front_left_steering_joint">
      <command_interface name="position">
        <param name="min">${-max_steering_angle}</param>
        <param name="max">${max_steering_angle}</param>
      </command_interface>

      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="front_right_steering_joint">
      <command_interface name="position">
        <param name="min">${-max_steering_angle}</param>
        <param name="max">${max_steering_angle}</param>
      </command_interface>

      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="rear_left_wheel_joint">
      <command_interface name="velocity">
        <param name="min">${-max_wheel_angular_velocity}</param>
        <param name="max">${max_wheel_angular_velocity}</param>
      </command_interface>

      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="rear_right_wheel_joint">
      <command_interface name="velocity">
        <param name="min">${-max_wheel_angular_velocity}</param>
        <param name="max">${max_wheel_angular_velocity}</param>
      </command_interface>

      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>

  <!-- Gazebo Plugins -->
  <gazebo>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <topic>joint_states</topic>
      <joint_name>front_left_steering_joint</joint_name>
      <joint_name>front_right_steering_joint</joint_name>
      <joint_name>front_left_wheel_joint</joint_name>
      <joint_name>front_right_wheel_joint</joint_name>
      <joint_name>rear_left_wheel_joint</joint_name>
      <joint_name>rear_right_wheel_joint</joint_name>
    </plugin>

    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find gazebo_ackermann_steering_vehicle)/config/gz_ros2_control.yaml</parameters>
    </plugin>

    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </gazebo>

  <!-- Camera -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="vehicle_front_camera">
      <update_rate>${camera_fps}</update_rate>
      <visualize>true</visualize>
      <topic>camera</topic>

      <camera name="head">
        <horizontal_fov>${camera_fov}</horizontal_fov>
        <image>
          <width>${image_width}</width>
          <height>${image_height}</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>vehicle_front_camera</cameraName>
        <frameName>camera_link</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

Notice the **gazebo** and **ros2_control** tags that define the simulation properties, camera sensor, and the configured plugins in use. Also, note to the arguments in the Xacro file, as they parameterize the vehicle model, with some being essential for vehicle control. For example, specific parameters are crucial to calculate the Ackermann steering angle for each wheel and determine the velocity of the rear wheels.

A configuration file is also required for complete integration with Gazebo. This file is used by the **ros_gz** plugin to map ROS topics to Gazebo topics and vice versa, simplifying communication between the simulator and the ROS 2 framework. To set this up, create a file named **ros_gz_bridge.yaml** inside the **config/** directory of your package. Then, add the following content to configure the topic mappings:

```yaml
- ros_topic_name: "clock"
  gz_topic_name: "clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: "GZ_TO_ROS"

- ros_topic_name: "joint_states"
  gz_topic_name: "joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: "GZ_TO_ROS"

- ros_topic_name: "camera/image_raw"
  gz_topic_name: "camera"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: "GZ_TO_ROS"

- ros_topic_name: "camera/info"
  gz_topic_name: "camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: "GZ_TO_ROS"
  
- ros_topic_name: "tf"
  gz_topic_name: "tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: "GZ_TO_ROS"
```

In addition, another configuration file is needed for this package. It is used with the **gz_ros2_control** package to enable feedforward controllers for both motion and steering. This file defines the controllers and broadcasters responsible for joint control and state feedback, specifying velocity control for the rear wheels and position control for the front steering joints. To set this up, create a file named **gz_ros2_control.yaml** inside the **config/** directory of your package. Then, add the following content to configure the controllers:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100 # Hz

    forward_velocity_controller:
      type: forward_command_controller/ForwardCommandController

    forward_position_controller:
      type: forward_command_controller/ForwardCommandController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

forward_velocity_controller:
  ros__parameters:
    joints:
      - rear_left_wheel_joint
      - rear_right_wheel_joint
    interface_name: velocity
    command_interfaces:
      - velocity
    state_interfaces:
      - position
      - velocity

forward_position_controller:
  ros__parameters:
    joints:
      - front_left_steering_joint
      - front_right_steering_joint
    interface_name: position
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

To bring everything together, a ROS launch file can be used. A ROS launch file automates the process of starting nodes and setting parameters in a ROS environment. It defines nodes, parameters, and other settings in a structured manner, simplifying the coordination of various components within a robotic system. In ROS 2, launch files are typically written in Python, offering greater flexibility and the ability to include dynamic behavior, such as conditionals and loops.

To set up the simulation, you can create a launch file that reads configuration files, loads the robot description, and starts the Gazebo simulator. To make the file more flexible and reusable, you can allow the world used in the simulation and the vehicle's initial pose to be passed as arguments. This way, the launch file becomes adaptable to different simulation scenarios. Create a **vehicle.launch.py** file inside the **launch/** directory to integrate everything. Add the following content to launch the vehicle in Gazebo:

```python
import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            RegisterEventHandler, ExecuteProcess)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def load_robot_description(robot_description_path, vehicle_params_path):
    """
    Loads the robot description from a Xacro file, using parameters from a YAML file.

    @param robot_description_path: Path to the robot's Xacro file.
    @param vehicle_params_path: Path to the YAML file containing the vehicle parameters.
    @return: A string containing the robot's URDF XML description.
    """
    # Load parameters from YAML file
    with open(vehicle_params_path, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']

    # Process the Xacro file to generate the URDF representation of the robot
    robot_description = xacro.process_file(
        robot_description_path,
        mappings={key: str(value) for key, value in vehicle_params.items()})

    return robot_description.toxml()


def start_vehicle_control():
    """
    Starts the necessary controllers for the vehicle's operation in ROS 2.

    @return: A tuple containing ExecuteProcess actions for the joint state, forward velocity, 
             and forward position controllers.
    """
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'joint_state_broadcaster'],
        output='screen')

    forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_velocity_controller'],
        output='screen')

    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen')

    return (joint_state_controller,
            forward_velocity_controller,
            forward_position_controller)


def generate_launch_description():
    # Define a launch argument for the world file, defaulting to "empty.sdf"
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Specify the world file for Gazebo (e.g., empty.sdf)')

    # Define launch arguments for initial pose
    x_arg = DeclareLaunchArgument('x', default_value='0.0',
                                  description='Initial X position')

    y_arg = DeclareLaunchArgument('y', default_value='0.0',
                                  description='Initial Y position')

    z_arg = DeclareLaunchArgument('z', default_value='0.1',
                                  description='Initial Z position')

    roll_arg = DeclareLaunchArgument('R', default_value='0.0',
                                     description='Initial Roll')

    pitch_arg = DeclareLaunchArgument('P', default_value='0.0',
                                      description='Initial Pitch')

    yaw_arg = DeclareLaunchArgument('Y', default_value='0.0',
                                    description='Initial Yaw')

    # Retrieve launch configurations
    world_file = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    # Define the robot's and package name
    robot_name = "gazebo_ackermann_steering_vehicle"
    robot_pkg_path = get_package_share_directory(robot_name)

    # Set paths to Xacro model and configuration files
    robot_description_path = os.path.join(robot_pkg_path, 'model',
                                          'vehicle.xacro')

    gz_bridge_params_path = os.path.join(robot_pkg_path, 'config',
                                         'ros_gz_bridge.yaml')

    vehicle_params_path = os.path.join(robot_pkg_path, 'config',
                                       'parameters.yaml')
    # Load URDF
    robot_description = load_robot_description(robot_description_path,
                                               vehicle_params_path)

    # Prepare to include the Gazebo simulation launch file
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'),
                     'launch',
                     'gz_sim.launch.py'))

    # Include the Gazebo launch description
    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        launch_arguments={'gz_args': [f'-r -v 4 ', world_file],
                          'on_exit_shutdown': 'true'}.items())

    # Create node to spawn robot model in the Gazebo world
    spawn_model_gazebo_node = Node(package='ros_gz_sim',
                                   executable='create',
                                   arguments=['-name', robot_name,
                                              '-string', robot_description,
                                              '-x', x,
                                              '-y', y,
                                              '-z', z,
                                              '-R', roll,
                                              '-P', pitch,
                                              '-Y', yaw,
                                              '-allow_renaming', 'false'],
                                   output='screen')

    # Create node to publish the robot state
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description,
                                          'use_sim_time': True}],
                                      output='screen')

    # Create a node for the ROS-Gazebo bridge to handle message passing
    gz_bridge_node = Node(package='ros_gz_bridge',
                          executable='parameter_bridge',
                          arguments=['--ros-args', '-p',
                                     f'config_file:={gz_bridge_params_path}'],
                          output='screen')

    # Create the launch description
    launch_description = LaunchDescription([world_arg,
                                            gazebo_launch,
                                            x_arg,
                                            y_arg,
                                            z_arg,
                                            roll_arg,
                                            pitch_arg,
                                            yaw_arg,
                                            spawn_model_gazebo_node,
                                            robot_state_publisher_node,
                                            gz_bridge_node])

    return launch_description
```

With all configuration and launch files in place, the vehicle is fully modeled and integrated with the simulator and control systems. The simulation environment is now ready to run.

To set up and launch the ROS 2 application, start by sourcing the ROS 2 Jazzy setup file. Navigate to the workspace, build it to compile the code and dependencies, and then source the workspace’s setup file to make the packages accessible. Finally, launch the application to start the simulation. Below is the script for these steps:

```bash
source /opt/ros/jazzy/setup.bash

cd ~/workspace

colcon build

source install/setup.bash

ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py
```

If everything runs correctly, after launching **vehicle.launch.py** the Gazebo simulator will open and spawn the vehicle described by the Xacro file. The vehicle should then be visible in the simulation, as shown in the following image:

![Ackermann Steering Vehicle Model in Gazebo Simulation]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/ackermann_steering_vehicle_model_in_gazebo_simulation.gif)
*Figure 2 — Ackermann Steering Vehicle Model in Gazebo Simulation.*

To launch the robot in a specified world with a custom initial pose, the **vehicle.launch.py** file should be executed, providing the world path and robot pose arguments. These arguments include the world file path, the initial x, y, and z coordinates of the robot, as well as the initial roll (R), pitch (P), and yaw (Y) orientations. For example, in the given setup bellow, the robot starts at position (x, y, z) = (1.0, 2.0, 0.5) with a yaw of 1.57 radians in the specified world:

``` bash
ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py world:=world.sdf x:=1.0 y:=2.0 z:=0.5 R:=0.0 P:=0.0 Y:=1.57
```

### Ackermann Steering Geometry

Ackermann steering geometry is a steering system that ensures smooth turning by aligning the wheels' angles so the inside and outside wheels follow different circular paths. This is achieved by turning the inside wheel more than the outside, accounting for their varying radii during a turn. To achieve this, the geometry adjusts the steering angles so that all wheels point toward a common turning center, reducing tire wear and preventing slipping. This is particularly important for vehicles with four wheels, as improper alignment during turns can compromise stability and handling.


![Ackermann Steering Geometry]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/ackermann_steering.webp)
*Figure 3 — Ackermann Steering Geometry.*


Considering that the base steering angle $$\phi$$ represents desired wheel steering angle in the bicycle model of the vehicle, the individual steering angles of the inner and outer wheels can be determined by examining the geometry of three triangles. These triangles are formed using the following vehicle parameters: track width $$w$$ (lateral wheel distance), wheelbase $$l$$ (longitudinal axle distance), base $$\phi$$, inner $$\phi_{i}$$​ and outer $$\phi_{o}$$ wheel steering angles, and the distance $$r$$ from the instantaneous center of curvature to the vehicle's center. 

$$
tan(\phi) = \frac{l}{r}
$$

$$
tan(\phi_{i}) = \frac{l}{(r - \frac{w}{2})}
$$

$$
tan(\phi_{o}) = \frac{l}{(r + \frac{w}{2})}
$$

Then, subtracting the reciprocals of the latter two equations leads to the derivation of the Ackermann steering equation:

$$
\frac{1}{tan(\phi_{o})} - \frac{1}{tan(\phi_{i})} = cot(\phi_{o}) - cot(\phi_{i}) = \frac{(r + \frac{w}{2})}{l} - \frac{(r - \frac{w}{2})}{l} = \frac{w}{l}
$$

Alternatively, the two cotangents can be represented in terms of the base angle $$\phi$$ as follows:

$$
cot(\phi_{i}) - cot(\phi) = \frac{(r - \frac{w}{2})}{l} - \frac{r}{l} = -\frac{w}{2l} \Longleftrightarrow cot(\phi_{i}) = cot(\phi) - \frac{w}{2l}
$$

$$
cot(\phi_{o}) - cot(\phi) = \frac{(r + \frac{w}{2})}{l} - \frac{r}{l} = +\frac{w}{2l} \Longleftrightarrow cot(\phi_{o}) = cot(\phi) + \frac{w}{2l}
$$

These equations encounter a limitation when $$\phi=0$$, since $$cot⁡(0)$$ is undefined. To address this issue, it is possible use the trigonometric identity $$cot(⁡α)=\frac{cos⁡(α)}{sin⁡(α)}$$ to reformulate the equations, ensuring they remain valid under this condition. The revised equations for the inner and outer wheel steering angles are:

$$
\phi_{i} = tan^{-1} \left( \frac{2 \cdot l \cdot sin(\phi)}{2 \cdot l \cdot cos(\phi) - w \cdot sin(\phi)} \right)
$$

$$
\phi_{o} = tan^{-1} \left( \frac{2 \cdot l \cdot sin(\phi)}{2 \cdot l \cdot cos(\phi) + w \cdot sin(\phi)} \right)
$$

Hence, it becomes possible to calculate the steering angles of the inner and outer wheels based on the desired base angle, following Ackermann steering geometry principles.

### Rear Differential Model

Furthermore, it is also important to consider the velocity of the rear wheels for an accurate steering model. Each rear wheel follows a circular path with a different radius, meaning that while the angular velocity of both wheels remains the same, the linear velocity of the outer wheel will be greater than that of the inner wheel. In real vehicles, this is usually addressed by a mechanical rear differential, which allows the rear wheels to rotate at different speeds, preventing tire slippage.

![Rear Differential Model]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/rear_differential.webp)
*Figure 4 — Rear Differential Model.*

Considering the base velocity $$v$$ of the vehicle as a reference, the velocities of the inner ($$v_i$$​) and outer ($$v_o$$​) rear wheels are derived based on the geometry of circular motion. Thus, using the bicycle model, the radius of curvature $$r$$ is defined as:

$$
r = \frac{l}{tan(\phi)}
$$

Therefore, the inner and outer rear wheel paths lie on circles of radii:

$$
r_i = r - \frac{w}{l}
$$

$$
r_o = r + \frac{w}{l}
$$

Considering $$\omega$$ as the angular velocity of the vehicle, then the linear velocities $$v_i$$ and $$v_o$$ of the rear wheels are directly related to $$\omega$$. The relationship is expressed as:

$$
\omega = \frac{v}{r} = \frac{v_i}{r_i} = \frac{v_o}{r_o} \Longleftrightarrow \omega = \frac{v_i}{(r - \frac{w}{l})} = \frac{v_o}{(r + \frac{w}{l})}
$$

Then the linear velocities $$v_i$$ and $$v_o$$ of the rear wheels can be defined as:

$$
v_i = \omega \cdot \left(r - \frac{w}{2}\right) = \frac{v}{r} \cdot \left(r - \frac{w}{2}\right) = \frac{v}{(\frac{l}{tan(\phi)})} \cdot \left(\frac{l}{tan(\phi)} - \frac{w}{2}\right)
$$

$$
v_o = \omega \cdot \left(r + \frac{w}{2} \right) = \frac{v}{r} \cdot \left(r + \frac{w}{2}\right) = \frac{v}{(\frac{l}{tan(\phi)})} \cdot \left(\frac{l}{tan(\phi)} + \frac{w}{2}\right)
$$

This analysis enables the calculation of each rear wheel's velocity based on the steering angle and vehicle speed, ensuring proper steering by considering the differing radii of their circular paths. With this rear differential model, combined with the Ackermann steering geometry derived earlier, the vehicle simulation will avoid tire slippage and steer as a good real vehicle would.

### Vehicle Control

As referenced earlier, the **gz_ros2_control** package is used to control the vehicle's joints with feedforward controllers. These controllers predict and adjust the system’s behavior proactively using input data, applying corrective actions before changes occur. This package allows developers to focus on developing the vehicle behaviour, without the need for direct control of each joint. By passing the required values to the **gz_ros2_control** interface, the package handles the low-level control.

The **gz_ros2_control.yaml** file, located in the the **config/** directory, sets up two feedforward controllers: **forward_velocity_controller**, responsible for managing the rear wheel motion, and **forward_position_controller**, which handles the front steering joints. These controllers are connected to the ROS 2 by the topics **/forward_velocity_controller/commands** and **/forward_position_controller/commands**, allowing direct control of vehicle motion and steering within the ROS 2 interface. Each topic receives a **Float64MultiArray** containing the desired position or desired velocity values for each joint, in the order specified in the file. Since the vehicle follows the Ackermann steering geometry with a differential rear model, a control interface is needed to convert the desired vehicle angle and vehicle velocity into individual desired joint positions and joint velocities. 

Therefore, a node must be implemented to subscribe to two new topics, **/steering_angle** and **/velocity**, which provide the desired steering angle (in radians) and velocity (in m/s) for the vehicle, respectively. This node will then convert these values into the required joint positions and velocities based on the Ackermann steering geometry and the differential rear model derived earlier. Finally, the node will publish the calculated results to the **/forward_position_controller/commands** and **/forward_velocity_controller/commands** topics from  **gz_ros2_control** package.

Moving on, the track width and wheelbase are necessary to calculate the Ackermann steering angles for each wheel. These values are determined using parameters from the **config/params.yaml** file, which define the vehicle's body width, body length, wheel width, and wheel radius. Additionally, the maximum steering angle and maximum velocity are also needed, to constrain the controller computations to these values. So, the parameters must also be provided to the node to load these values.

Another feature needed for safety is a timeout mechanism. If the subscribed topics stop receiving a desired angle or velocity after a specified period, the vehicle must stop until the topics are  published again. While this is a simulation where system failures are not a concern, implementing this feature is still important because it ensures consistency if the same code is ever shared or reused with a real vehicle.

To implement this node, start by navigating to the **include/** folder and creating the **vehicle_controller.hpp** header file. In this file, declare the function signatures for **ackermann_steering_angle**, which calculates the steering angle for each wheel using Ackermann geometry, and **rear_differential_velocity**, which computes the rear wheel velocities based on the differential model. Also include the **timer_callback** function, which publishes these values to the **gz_ros2_control** package, and the **steering_angle_callback** and **velocity_callback** functions, which handle incoming messages from the **/steering_angle** and **/velocity** topics, respectively. The complete header file code is shown next:

```cpp
#ifndef VEHICLE_CONTROLLER_HPP
#define VEHICLE_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class VehicleController : public rclcpp::Node
{

public:
  VehicleController(const double timer_period = 1e-2, const double timeout_duration = 1e9);

private:
  /**
   * @brief Calculates the steering angles for the left and right wheels using the 
   * Ackermann steering geometry.
   * 
   * @details This function computes the angles for the left and right front wheels based on the 
   * input steering angle and the vehicle's dimensions:
   * - `wheel_base_` (distance between front and rear axles).
   * - `track_width_` (distance between the left and right wheels).
   * 
   * @return A pair of left and right steering angles [rad].
   */
  std::pair<double, double> ackermann_steering_angle();

  /**
   * @brief Calculates the rear differential wheel velocities for a vehicle with a rear 
   * differential axle, considering the steering angle and limiting the velocities to a maximum 
   * allowable value.
   * 
   * @details This function computes rear wheel velocities based on Ackermann steering geometry:
   * - Both wheels have the same velocity for straight-line motion.
   * - For turning, it computes individual wheel velocities based on the turning radius.
   * - If any wheel velocity exceeds the maximum velocity, both are scaled proportionally.
   * 
   * @return A pair of left and right wheel velocities [m/s].
   */
  std::pair<double, double> rear_differential_velocity();

  /**
   * @brief Periodically checks for timeout and publishes steering angles and wheel velocities.
   * 
   * @details This timer callback checks if a timeout has occurred since the last received 
   * message for desired steering angle or velocity. If a timeout is detected, it resets 
   * the wheel steering angles and angular velocities to zero. The function also 
   * publishes the current steering positions and wheel velocities to the appropriate topics.
   */
  void timer_callback();

  /**
   * @brief Callback function for receiving and handling desired steering angle.
   * 
   * @details This callback updates the steering angle based on the input message and clamps 
   * the value to the maximum allowable steering angle. The function calculates the 
   * corresponding Ackermann steering angles for the left and right wheels and updates 
   * the internal state of the controller.
   * 
   * @param msg A shared pointer to the incoming message containing the steering angle [rad].
   */
  void steering_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * @brief Callback function for receiving and handling desired velocity.
   * 
   * @details This callback updates the vehicle velocity based on the input message and clamps 
   * the value to the maximum allowable velocity. The function calculates the wheel 
   * velocities based on the rear differential model and updates the internal state 
   * with the corresponding angular velocities of the wheels.
   * 
   * @param msg A shared pointer to the incoming message containing the velocity [m/s].
   */
  void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg);

  double timeout_duration_;
  rclcpp::Time last_velocity_time_;
  rclcpp::Time last_steering_time_;

  double body_width_;
  double body_length_;
  double wheel_radius_;
  double wheel_width_;
  double max_steering_angle_;
  double max_velocity_;
  double wheel_base_;
  double track_width_;

  double steering_angle_;
  double velocity_;

  std::vector<double> wheel_angular_velocity_;
  std::vector<double> wheel_steering_angle_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_angle_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // VEHICLE_CONTROLLER_HPP
```

With the header file and function signatures defined, it's time to create the node implementation. Begin by creating a new file named **vehicle_controller.cpp** inside the **src/** folder of the package. This file contains the actual logic behind the **VehicleController** class methods declared earlier. In this implementation, the node must retrieve the necessary parameters for the computations, process incoming steering angle and velocity, compute the appropriate steering angles and wheel speeds based on the Ackermann steering model, and publish the commands at a fixed rate. A timeout mechanism also stop the vehicle safely if updates stop coming in. These features come together in the code below:

```cpp
#include "vehicle_controller.hpp"

VehicleController::VehicleController(const double timer_period, const double timeout_duration) :
  Node{"vehicle_controller"},
  timeout_duration_{timeout_duration},
  last_velocity_time_{get_clock()->now()},
  last_steering_time_{get_clock()->now()},
  body_width_{0.0},
  body_length_{0.0},
  wheel_radius_{0.0},
  wheel_width_{0.0},
  max_steering_angle_{0.0},
  max_velocity_{0.0},
  wheel_base_{0.0},
  track_width_{0.0},
  steering_angle_{0.0},
  velocity_{0.0},
  wheel_angular_velocity_{0.0, 0.0},
  wheel_steering_angle_{0.0, 0.0}
{
  // Declare the used parameters
  declare_parameter<double>("body_width", 0.0);
  declare_parameter<double>("body_length", 0.0);
  declare_parameter<double>("wheel_radius", 0.0);
  declare_parameter<double>("wheel_width", 0.0);
  declare_parameter<double>("max_steering_angle", 0.0);
  declare_parameter<double>("max_velocity", 0.0);

  // Get parameters on startup
  get_parameter("body_width", body_width_);
  get_parameter("body_length", body_length_);
  get_parameter("wheel_radius", wheel_radius_);
  get_parameter("wheel_width", wheel_width_);
  get_parameter("max_steering_angle", max_steering_angle_);
  get_parameter("max_velocity", max_velocity_);

  // Set the track width and wheel base
  track_width_ = body_width_ + (2 * wheel_width_ / 2);
  wheel_base_ = body_length_ - (2 * wheel_radius_);

  // Subscribers
  steering_angle_subscriber_ = create_subscription<std_msgs::msg::Float64>(
    "/steering_angle", 10,
    std::bind(&VehicleController::steering_angle_callback, this, std::placeholders::_1));

  velocity_subscriber_ = create_subscription<std_msgs::msg::Float64>(
    "/velocity", 10, std::bind(&VehicleController::velocity_callback, this, std::placeholders::_1));

  // Publishers
  position_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_position_controller/commands", 10);

  velocity_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_velocity_controller/commands", 10);

  // Timer loop
  timer_ = create_wall_timer(std::chrono::duration<double>(timer_period),
                             std::bind(&VehicleController::timer_callback, this));
}

std::pair<double, double> VehicleController::ackermann_steering_angle()
{
  double left_wheel_angle{0.0};
  double right_wheel_angle{0.0};

  // Steering angle is not zero nor too small
  if (abs(steering_angle_) > 1e-3) {
    const double sin_angle = sin(abs(steering_angle_));
    const double cos_angle = cos(abs(steering_angle_));

    if (steering_angle_ > 0.0) {
      // Left and right wheel angles when steering left
      left_wheel_angle = atan((2 * wheel_base_ * sin_angle) /
                              (2 * wheel_base_ * cos_angle - track_width_ * sin_angle));

      right_wheel_angle = atan((2 * wheel_base_ * sin_angle) /
                               (2 * wheel_base_ * cos_angle + track_width_ * sin_angle));
    } else {
      // Left and right wheel angles when steering right (mirror left with negative signs)
      left_wheel_angle = -atan((2 * wheel_base_ * sin_angle) /
                               (2 * wheel_base_ * cos_angle + track_width_ * sin_angle));

      right_wheel_angle = -atan((2 * wheel_base_ * sin_angle) /
                                (2 * wheel_base_ * cos_angle - track_width_ * sin_angle));
    }
  }

  return std::make_pair(left_wheel_angle, right_wheel_angle);
}

std::pair<double, double> VehicleController::rear_differential_velocity()
{
  double left_wheel_velocity{velocity_};
  double right_wheel_velocity{velocity_};

  // Steering angle is not zero nor too small
  if (abs(steering_angle_) > 1e-3) {
    // Calculate turning radius and angular velocity
    const double turning_radius = wheel_base_ / tan(abs(steering_angle_));  // [m]
    const double vehicle_angular_velocity = velocity_ / turning_radius;     // [rad/s]

    // Compute inner and outer wheel radius
    const double inner_radius = turning_radius - (track_width_ / 2.0);
    const double outer_radius = turning_radius + (track_width_ / 2.0);

    if (steering_angle_ > 0.0) {
      // Vehicle turning left
      left_wheel_velocity = vehicle_angular_velocity * inner_radius;
      right_wheel_velocity = vehicle_angular_velocity * outer_radius;
    } else {
      // Vehicle turning right
      left_wheel_velocity = vehicle_angular_velocity * outer_radius;
      right_wheel_velocity = vehicle_angular_velocity * inner_radius;
    }

    // Determine the maximum wheel velocity
    const double max_wheel_velocity = std::max(abs(left_wheel_velocity), 
                                               abs(right_wheel_velocity));

    // Scale both wheel velocities proportionally if the maximum is exceeded
    if (max_wheel_velocity > max_velocity_) {
      const double scaling_factor = max_velocity_ / max_wheel_velocity;
      left_wheel_velocity *= scaling_factor;
      right_wheel_velocity *= scaling_factor;
    }
  }

  return std::make_pair(left_wheel_velocity, right_wheel_velocity);
}

void VehicleController::timer_callback()
{
  const auto current_time{get_clock()->now()};
  const auto velocity_elapsed_time{(current_time - last_velocity_time_).nanoseconds()};
  const auto steering_elapsed_time{(current_time - last_steering_time_).nanoseconds()};

  // Reset velocity to zero if timeout
  if (velocity_elapsed_time > timeout_duration_) {
    wheel_angular_velocity_ = {0.0, 0.0};
  }

  // Reset steering angle to zero if timeout
  if (steering_elapsed_time > timeout_duration_) {
    wheel_steering_angle_ = {0.0, 0.0};
  }

  // Publish steering position
  std_msgs::msg::Float64MultiArray position_msg;
  position_msg.data = wheel_steering_angle_;
  position_publisher_->publish(position_msg);

  // Publish wheels velocity
  std_msgs::msg::Float64MultiArray velocity_msg;
  velocity_msg.data = wheel_angular_velocity_;
  velocity_publisher_->publish(velocity_msg);
}

void VehicleController::steering_angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  last_steering_time_ = get_clock()->now();  // Update timestamp

  if (msg->data > max_steering_angle_) {
    steering_angle_ = max_steering_angle_;
  } else if (msg->data < -max_steering_angle_) {
    steering_angle_ = -max_steering_angle_;
  } else {
    steering_angle_ = msg->data;
  }

  const auto wheel_angles{ackermann_steering_angle()};

  wheel_steering_angle_ = {wheel_angles.first, wheel_angles.second};
}

void VehicleController::velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  last_velocity_time_ = get_clock()->now();  // Update timestamp

  if (msg->data > max_velocity_) {
    velocity_ = max_velocity_;
  } else if (msg->data < -max_velocity_) {
    velocity_ = -max_velocity_;
  } else {
    velocity_ = msg->data;
  }

  const auto wheel_velocity{rear_differential_velocity()};

  // Convert wheel linear velocity to wheel angular velocity
  wheel_angular_velocity_ = {(wheel_velocity.first / wheel_radius_),
                             (wheel_velocity.second / wheel_radius_)};
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleController>());
  rclcpp::shutdown();
  return 0;
}
```

To ensure proper integration, it is necessary to update the **launch/vehicle.launch.py** file to launch the **vehicle_controller** node alongside the vehicle simulation. Additionally, the launch file must be modified to pass the parameters to the **vehicle_controller** node. These changes are shown below: 

```python
import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            RegisterEventHandler, ExecuteProcess)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def load_robot_description(robot_description_path, vehicle_params_path):
    """
    Loads the robot description from a Xacro file, using parameters from a YAML file.

    @param robot_description_path: Path to the robot's Xacro file.
    @param vehicle_params_path: Path to the YAML file containing the vehicle parameters.
    @return: A string containing the robot's URDF XML description.
    """
    # Load parameters from YAML file
    with open(vehicle_params_path, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']

    # Process the Xacro file to generate the URDF representation of the robot
    robot_description = xacro.process_file(
        robot_description_path,
        mappings={key: str(value) for key, value in vehicle_params.items()})

    return robot_description.toxml()


def start_vehicle_control():
    """
    Starts the necessary controllers for the vehicle's operation in ROS 2.

    @return: A tuple containing ExecuteProcess actions for the joint state, forward velocity, 
             and forward position controllers.
    """
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'joint_state_broadcaster'],
        output='screen')

    forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_velocity_controller'],
        output='screen')

    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen')

    return (joint_state_controller,
            forward_velocity_controller,
            forward_position_controller)


def generate_launch_description():
    # Define a launch argument for the world file, defaulting to "empty.sdf"
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Specify the world file for Gazebo (e.g., empty.sdf)')

    # Define launch arguments for initial pose
    x_arg = DeclareLaunchArgument('x', default_value='0.0',
                                  description='Initial X position')

    y_arg = DeclareLaunchArgument('y', default_value='0.0',
                                  description='Initial Y position')

    z_arg = DeclareLaunchArgument('z', default_value='0.1',
                                  description='Initial Z position')

    roll_arg = DeclareLaunchArgument('R', default_value='0.0',
                                     description='Initial Roll')

    pitch_arg = DeclareLaunchArgument('P', default_value='0.0',
                                      description='Initial Pitch')

    yaw_arg = DeclareLaunchArgument('Y', default_value='0.0',
                                    description='Initial Yaw')

    # Retrieve launch configurations
    world_file = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    # Define the robot's and package name
    package_name = "gazebo_ackermann_steering_vehicle"
    package_path = get_package_share_directory(package_name)

    # Set paths to Xacro model and configuration files
    robot_description_path = os.path.join(package_path, 'model',
                                          'vehicle.xacro')

    gz_bridge_params_path = os.path.join(package_path, 'config',
                                         'ros_gz_bridge.yaml')

    vehicle_params_path = os.path.join(package_path, 'config',
                                       'parameters.yaml')
    # Load URDF
    robot_description = load_robot_description(robot_description_path,
                                               vehicle_params_path)

    # Prepare to include the Gazebo simulation launch file
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'),
                     'launch',
                     'gz_sim.launch.py'))

    # Include the Gazebo launch description
    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        launch_arguments={'gz_args': [f'-r -v 4 ', world_file],
                          'on_exit_shutdown': 'true'}.items())

    robot_name = "ackermann_steering_vehicle"
    
    # Create node to spawn robot model in the Gazebo world
    spawn_model_gazebo_node = Node(package='ros_gz_sim',
                                   executable='create',
                                   arguments=['-name', robot_name,
                                              '-string', robot_description,
                                              '-x', x,
                                              '-y', y,
                                              '-z', z,
                                              '-R', roll,
                                              '-P', pitch,
                                              '-Y', yaw,
                                              '-allow_renaming', 'false'],
                                   output='screen')

    # Create node to publish the robot state
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description,
                                          'use_sim_time': True}],
                                      output='screen')

    # Create a node for the ROS-Gazebo bridge to handle message passing
    gz_bridge_node = Node(package='ros_gz_bridge',
                          executable='parameter_bridge',
                          arguments=['--ros-args', '-p',
                                     f'config_file:={gz_bridge_params_path}'],
                          output='screen')

    # Start controllers
    joint_state, forward_velocity, forward_position = start_vehicle_control()

    # Load vehicle controller node
    vehicle_controller_node = Node(package='gazebo_ackermann_steering_vehicle',
                                   executable='vehicle_controller',
                                   parameters=[vehicle_params_path],
                                   output='screen')

    # Create the launch description
    launch_description = LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=spawn_model_gazebo_node,
                                        on_exit=[joint_state])),
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=joint_state,
                                        on_exit=[forward_velocity,
                                                 forward_position])),
        world_arg,
        gazebo_launch,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        spawn_model_gazebo_node,
        robot_state_publisher_node,
        vehicle_controller_node,
        gz_bridge_node])

    return launch_description
```

Finally, it is necessary to update the **CMakeLists.txt** file to include this node as a dependency to the **gazebo_ackermann_steering_vehicle** package:

```cmake
cmake_minimum_required(VERSION 3.8)
project(gazebo_ackermann_steering_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(vehicle_controller src/vehicle_controller.cpp)
ament_target_dependencies(vehicle_controller rclcpp std_msgs)

install(
  DIRECTORY launch model config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  vehicle_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

With this implementation, the vehicle control interface is complete. All the considerations discussed are now in place to effectively control the vehicle using the Ackermann steering model. To proceed, build the **gazebo_ackermann_steering_vehicle** package and launch **vehicle.launch.py** by running the following commands in your workspace:


```bash
source /opt/ros/jazzy/setup.bash

cd ~/workspace

colcon build

source install/setup.bash

ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py
```

In conclusion, if all steps were followed correctly, after launching the vehicle using the **vehicle.launch.py** file, the **/steering_angle** and **/velocity** topics will be available for controlling the vehicle. At this point, by opening a new command line and sourcing the ros environtment, it is possible to send commands to the vehicle via these topics. 

For instance, open two new command lines and in each one source the ROS 2 Jazzy setup file and source the environment setup file:

```bash
source /opt/ros/jazzy/setup.bash

cd ~/workspace

source install/setup.bash
```

After that, in the first command line, publish the desired vehicle's steering angle:

```bash
ros2 topic pub /steering_angle std_msgs/msg/Float64 "{data: 0.5}" --rate 10
```

In the other one, publish the desired vehicle's velocity:

```bash
ros2 topic pub /velocity std_msgs/msg/Float64 "{data: 0.5}" --rate 10
```

After publishing the desired steering angle and velocity to the vehicle controller node, the vehicle begins to follow a circular path, as illustrated in the following image:

![Ackermann Steering Vehicle Control in Simulation]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/ackermann_steering_vehicle_control.gif)
*Figure 5 — Ackermann Steering Vehicle Control in Simulation.*

### Joystick Controller

In many robotics and simulation workflows, having manual control over the vehicle is just as important as autonomous functionality. Whether you're testing control algorithms, validating system behavior, or collecting training data for Deep Learning projects, the ability to drive the vehicle manually adds flexibility to your setup.

To support this, a joystick control interface is provided, allowing users to steer and adjust the vehicle’s velocity using an Xbox One wireless controller. The analog sticks "L3" and "R3" are mapped to steering and throttle control, respectively, as illustrated in the image below. If you're using a different joystick model, you may need to adapt the software to account for variations in key mappings, which can differ across devices.

![Ackermann Steering Vehicle Control in Simulation]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/xbox_joystick_controller.webp)
*Figure 6 — Xbox Joystick Controller.*

To implement joystick-based control, navigate to the **include/** directory and create the **joystick_controller.hpp** header file. In this file, define the **JoystickController** class, which will subscribe to joystick input messages and publish the corresponding steering and velocity commands. This class defines the node responsible for interpreting joystick input and translating it into steering and velocity commands for the vehicle. The complete header file code is shown below:

```cpp
#ifndef JOY_CONTROLLER_HPP
#define JOY_CONTROLLER_HPP

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

class JoystickController : public rclcpp::Node
{

public:
  JoystickController(const double timer_period = 1e-2);

private:
  /**
   * @brief Callback function to process joystick inputs and update control states.
   * 
   * @details This function listens to joystick messages and updates the desired steering angle 
   * and the desired velocity based on the joystick inputs.
   * - Axis 0 is used to calculate the desired steering angle as a fraction of the maximum steering angle.
   * - Axis 3 is used to calculate the desired velocity as a fraction of the maximum velocity.
   * 
   * @param msg Pointer to the received Joy message containing button and axis states.
   */
  void listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  /**
   * @brief Timer callback function to publish desired steering angle and velocity.
   * 
   * @details This function periodically publishes the desired steering angle and velocity 
   * to their respective topics.
   */
  void timer_callback();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_angle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double max_steering_angle_;
  double max_velocity_;

  double steering_angle_;
  double velocity_;
};

#endif  // JOY_CONTROLLER_HPP
```

Now, go to the **src/** directory and create the **joystick_controller.cpp** file to implement the joystick control node. This node subscribes to the **/joy** topic to read joystick inputs and loads parameters from **config/params.yaml** during initialization. The left and right stick axes control the steering angle and velocity as fractions of their maximum values. A **timer_callback** function periodically publishes these commands to the **/steering_angle** and **/velocity** topics. The complete source code is shown below:

```cpp
#include "joystick_controller.hpp"

JoystickController::JoystickController(const double timer_period) :
  Node{"joystick_controller"},
  max_steering_angle_{0.0},
  max_velocity_{0.0},
  steering_angle_{0.0},
  velocity_{0.0}
{
  // Declare the used parameters
  declare_parameter<double>("max_steering_angle", 0.0);
  declare_parameter<double>("max_velocity", 0.0);

  // Get parameters on startup
  get_parameter("max_steering_angle", max_steering_angle_);
  get_parameter("max_velocity", max_velocity_);

  // Subscription to the 'joy' topic
  subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1, std::bind(&JoystickController::listener_callback, this, std::placeholders::_1));

  // Publishers for /angle and /velocity topics
  steering_angle_publisher_ = create_publisher<std_msgs::msg::Float64>("/steering_angle", 1);
  velocity_publisher_ = create_publisher<std_msgs::msg::Float64>("/velocity", 1);

  // Timer to periodically publish the desired angle and velocity
  timer_ = create_wall_timer(std::chrono::duration<double>(timer_period),
                             std::bind(&JoystickController::timer_callback, this));
}

void JoystickController::listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Set the desired angle and desired velocity based on the joystick's axis
  steering_angle_ = msg->axes[0] * max_steering_angle_;
  velocity_ = msg->axes[3] * max_velocity_;
}

void JoystickController::timer_callback()
{
  // Publish the desired angle
  auto angle_msg{std::make_shared<std_msgs::msg::Float64>()};
  angle_msg->data = steering_angle_;
  steering_angle_publisher_->publish(*angle_msg);

  // Publish the desired velocity
  auto velocity_msg{std::make_shared<std_msgs::msg::Float64>()};
  velocity_msg->data = velocity_;
  velocity_publisher_->publish(*velocity_msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickController>());
  rclcpp::shutdown();
  return 0;
}
```

Next, create a launch file inside the **launch/** directory named **joystick.launch.py**. This file starts two nodes: the **joy_node** from the **joy** package, which reads input from the joystick and publishes it on the **/joy** topic, and the **joystick_controller** node from your package, which translates joystick input into steering and velocity commands. The **joystick_controller** also loads parameters from **config/parameters.yaml** to ensure proper control behavior. The code below defines the launch setup. It first retrieves the parameter file path, then launches both nodes with the required configuration:

```python
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "gazebo_ackermann_steering_vehicle"

    vehicle_params_path = os.path.join(get_package_share_directory(package_name),
                                       'config', 'parameters.yaml')
    
    joy_node = Node(package="joy", executable="joy_node")
    
    joystick_controller_node = Node(package=package_name,
                                    executable='joystick_controller',
                                    parameters=[vehicle_params_path],
                                    output='screen')

    return LaunchDescription([joy_node,
                              joystick_controller_node])
```

Additionally, modify the **CMakeLists.txt** file to include the required dependencies:

```cmake
cmake_minimum_required(VERSION 3.8)
project(gazebo_ackermann_steering_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(vehicle_controller src/vehicle_controller.cpp)
ament_target_dependencies(vehicle_controller rclcpp std_msgs)

add_executable(joystick_controller src/joystick_controller.cpp)
ament_target_dependencies(joystick_controller rclcpp std_msgs sensor_msgs)

install(
  DIRECTORY launch model config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  vehicle_controller
  joystick_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

With this node implemented, the vehicle simulation can now be manually controlled using a joystick. To use the controller, first launch the **vehicle.launch.py** file. Then, in a separate terminal, launch the **joystick.launch.py** file as follows:

```bash
source /opt/ros/jazzy/setup.bash

cd ~/workspace

colcon build

source install/setup.bash

ros2 launch gazebo_ackermann_steering_vehicle joystick.launch.py
```

It is now possible to control the vehicle in the simulation using an Xbox One controller, employing its buttons and joysticks designed for handling the robot. The **gazebo_ackermann_steering_vehicle**  package is now complete, managing vehicle simulation, control, and manual operation. The final package structure should look like the following:

```text
gazebo_ackermann_steering_vehicle/
├── config/
│   ├── gz_ros2_control.yaml
│   ├── parameters.yaml
│   ├── ros_gz_bridge.yaml
├── include/
│   ├── joystick_controller.hpp
│   ├── vehicle_controller.hpp
├── launch/
│   ├── joystick.launch.py
│   ├── vehicle.launch.py
├── model/
│   ├── vehicle.xacro
├── src/
│   ├── joystick_controller.cpp
│   ├── vehicle_controller.cpp
├── CMakeLists.txt
├── package.xml
```

## Results

While setting up the simulation required configuring many files and integrating various packages, the result is a fully functional system. After connecting the joystick controller, the vehicle can now be controlled using a video game joystick.

![Vehicle Simulation and Control]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/vehicle_simulation_and_control.gif)
*Figure 7 — Vehicle Simulation and Control.*

There are a few additional topics not covered earlier, but which play an important role in the system. The **joint_states** topic publishes a **sensor_msgs/msg/JointState** message, representing the robot's state, and can be used for various applications, such as visualizing the robot in software like **RViz**. Additionally, the **camera/image_raw** topic publishes a **sensor_msgs/msg/Image** message, which contains the image captured by the onboard camera. This can be leveraged to test computer vision algorithms or to train machine learning models for tasks such as object detection or navigation.

![Vehicle Visualization in RViz]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/vehicle_visualization_in_rviz.gif)
*Figure 8 — Vehicle Visualization in RViz.*

## Conclusion

The simulation of an Ackermann steering vehicle using Gazebo Harmonic and ROS 2 Jazzy Jalisco highlights the advantages of modern robotics tools. Gazebo Harmonic provides a lightweight yet powerful open-source simulation environment, while the ROS 2 framework simplifies the integration of various software packages, making it easy to build modular and scalable robotic systems. 

By modeling the vehicle in a virtual environment, this project makes it easy to test control algorithms and simulate realistic driving without physical hardware. It can also be used in future projects to develop and test AI and navigation systems, serving as a starting point for more advanced applications.