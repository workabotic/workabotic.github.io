---
title: "Ackermann Steering Vehicle Simulation"
date: 2025-04-20 10:00:00
description: Modeling and simulation of a vehicle with Ackermann steering geometry using the Gazebo Simulator and the ROS 2 framework for robotics applications.
author: lucasmazzetto
keywords: robot, simulation, gazebo, ros, robotic simulation, autonomous robots, Gazebo simulation, ROS 2, ROS robot simulation, computer vision in robotics, self-driving car simulation, autonomous vehicle technology, simulation environments for robots, real-time robotics simulation, robotic perception
---

This article presents the development and simulation of an Ackermann steering vehicle using the Gazebo Harmonic simulator and the ROS 2 Jazzy Jalisco robotics framework. The vehicle model includes steering angle and velocity control, along with an embedded front camera that streams live images for vision-based tasks. This setup provides a safe environment for testing autonomous driving without the risks associated with real-world trials and serves as a platform for researchers and developers aiming to advance mobile robots' algorithms.​

## Introduction 

When developing mobile robots, safety and cost are critical, especially in the early stages. Simulation provides a safe and efficient alternative to testing on real hardware while also enabling the creation of realistic virtual environments to develop control algorithms and systems for tasks such as navigation, path planning, obstacle avoidance, and localization.

In this article, we'll explore the modeling and simulation of a vehicle using Ackermann steering geometry, a steering mechanism widely used in conventional vehicles. Simulating this steering configuration is needed to ensure realistic movement and precise maneuvering. This project was developed using the Gazebo Harmonic simulator and the ROS 2 Jazzy Jalisco robotics framework, providing a flexible environment for testing and development.

The source code used in this project is available on [GitHub](https://github.com/workabotic/gazebo_ackermann_steering_vehicle). Feel free to explore it, use it in your own projects, or contribute to its development.

### Ackermann Steering Geometry

Ackermann steering geometry is a steering system that ensures smooth turning by aligning the wheels' angles so the inside and outside wheels follow different circular paths. This is achieved by turning the inside wheel more than the outside, accounting for their varying radii during a turn. To achieve this, the geometry adjusts the steering angles so that all wheels point toward a common turning center, reducing tire wear and preventing slipping. This is particularly important for vehicles with four wheels, as improper alignment during turns can compromise stability and handling.


![Ackermann Steering Geometry]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/ackermann_steering.webp)
*Figure 3 — Ackermann Steering Geometry.*


Considering that the base steering angle $$\phi$$ represents the desired wheel steering angle in the bicycle model of the vehicle, the individual steering angles of the inner and outer wheels can be determined by examining the geometry of three triangles. These triangles are formed using the following vehicle parameters: track width $$w$$ (lateral wheel distance), wheelbase $$l$$ (longitudinal axle-to-axle distance), base $$\phi$$, inner $$\phi_{i}$$​ and outer $$\phi_{o}$$ wheel steering angles, and the distance $$r$$ from the instantaneous center of curvature to the vehicle's center. 

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

These equations encounter a limitation when $$\phi=0$$, since $$cot⁡(0)$$ is undefined. To address this issue, it is possible to use the trigonometric identity $$cot(⁡α)=\frac{cos⁡(α)}{sin⁡(α)}$$ to reformulate the equations, ensuring they remain valid under this condition. The revised equations for the inner and outer wheel steering angles are:

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

This analysis enables the calculation of each rear wheel's velocity based on the steering angle and vehicle speed, ensuring proper steering by considering the differing radii of their circular paths. With this rear differential model, combined with the Ackermann steering geometry derived earlier, the vehicle model can avoid tire slippage and steer as effectively as a real vehicle would.


## Requirements

For the development of this project, the following software dependencies were installed and used as part of the simulation setup:

- [**Linux Ubuntu 24.04**](https://ubuntu.com/blog/tag/ubuntu-24-04-lts) was used as the operating system.
- [**ROS 2 Jazzy Jalisco**](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html) was used as the robotics framework.
- [**Gazebo Harmonic**](https://gazebosim.org/docs/harmonic/getstarted/) was used as the simulation environment.

After the installation of ROS 2 Jazzy Jalisco, the required packages were installed by executing the following command in the terminal. These packages were used to enable robot control, establish communication between Gazebo and ROS 2, and manage robot state information and configuration parameters:

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

Then, a [ROS 2 workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) was created by executing the following commands:

```bash
mkdir -p ~/workspace/src
```

Next, a ROS 2 package named **gazebo_ackermann_steering_vehicle** was created using **CMake** as the build system. This package was used to define the vehicle model, implement control logic, and launch the vehicle within Gazebo simulation environments. The package structure was generated by executing the following commands:

```bash
cd ~/workspace/src

ros2 pkg create --build-type ament_cmake gazebo_ackermann_steering_vehicle
```

Afterward, a set of directories was created within the **gazebo_ackermann_steering_vehicle** package to organize configuration files, launch descriptions, and vehicle model definitions. The directory structure included the following components:

- **config**, which was used to store configuration files, including vehicle parameters and Gazebo bridge settings.
- **launch**, which was used to contain launch files responsible for starting the simulation, loading the vehicle model, and initializing the required nodes.
- **model**, which was used to store robot description files, such as `.xacro` or `.urdf`, defining the structure and physical properties of the Ackermann steering vehicle.

These directories were created by executing the following commands within the package directory:

```bash
cd ~/workspace/src/gazebo_ackermann_steering_vehicle

mkdir -p config launch model
```

At this point, the **gazebo_ackermann_steering_vehicle** package had the following directory structure:

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

In addition, the **CMakeLists.txt** file was updated to include the required dependencies and directories:

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

After that, the **package.xml** file was also updated to declare the required project dependencies:

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

With these steps completed, the environment setup was finalized, and the project was ready to proceed with vehicle modeling and control system development.

## Methods

In this project, a simulated vehicle with Ackermann steering geometry was developed using Gazebo Harmonic and ROS 2. Initially, the vehicle’s structure and dynamic properties were modeled to achieve realistic driving behavior. Subsequently, dedicated ROS 2 nodes were implemented to control steering angles and wheel velocities. Finally, manual operation was enabled through the integration of a video game joystick interface. The following sections describe each stage of the system implementation in detail.

### Vehicle Modeling

The vehicle model was designed to be easily customizable and adaptable to potential changes throughout the project lifecycle. A scaled version of a full-sized vehicle equipped with an embedded camera was implemented, and its main dimensions were defined through parameters to facilitate modification and reuse in future applications. The following image illustrates the vehicle model with the parametrized values applied.

![Vehicle Parameters]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/vehicle_model_parameters.webp)
*Figure 1 — Vehicle Model Parameters.*

To define the vehicle’s physical and kinematic properties, the model was represented using the Unified Robot Description Format (URDF), an XML-based format used to describe elements such as links, joints, and collision geometries. Due to the limited support for reuse in standard URDF files, Xacro (XML Macros) was employed to generate the final description. This approach enabled the use of reusable macros, parameterized definitions, and constants, thereby simplifying maintenance, reducing redundancy, and preserving flexibility for future modifications.

Parameters were used not only to define the model’s dimensions but also to compute the Ackermann steering angles and rear-wheel velocities. To ensure consistency and centralized access, these parameters were defined in a shared configuration format consumed by both the Xacro descriptions and the ROS 2 nodes. For this purpose, a **params.yaml** file was created within the **gazebo_ackermann_steering_vehicle/config** directory to specify the vehicle’s dimensional, kinematic, and dynamic properties, as well as the configuration of the onboard camera. The parameter definitions are provided below:


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

A Xacro file named **vehicle.xacro** was created and organized within the **gazebo_ackermann_steering_vehicle/model** directory to define the vehicle’s structural and physical properties, including mass, inertia, collision geometry, friction, damping, wheel placement, and the kinematic configuration for front-wheel steering with rear-wheel traction. Parameters specified in **gazebo_ackermann_steering_vehicle/config/params.yaml** were provided to the model at runtime through the launch system. Additionally, the file integrated and configured the required plugins to interface with external packages and the Gazebo simulator. The file contents are shown below:

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
  <xacro:arg name="camera_fov" default="1.396263" />
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

The **gazebo** and **ros2_control** tags were used to define the simulation properties, camera sensor, and the configured plugins. In addition, the Xacro file arguments were employed to parameterize the vehicle model, with several of them playing a key role in vehicle control. In particular, specific parameters were required to compute the Ackermann steering angle for each wheel and to determine the rear wheel velocities.

A configuration file was also required to enable full integration with Gazebo. This file was used by the **ros_gz** plugin to map ROS topics to Gazebo topics and vice versa, simplifying communication between the simulator and the ROS 2 framework. For this purpose, a file named **ros_gz_bridge.yaml** was created within the **gazebo_ackermann_steering_vehicle/config** directory to define the required topic mappings. The corresponding configuration is shown below:

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

In addition, another configuration file was required for this package to support the **gz_ros2_control** integration. This file was used to configure feedforward controllers for both vehicle motion and steering. It defined the controllers and broadcasters responsible for joint control and state feedback, specifying velocity control for the rear wheels and position control for the front-steering joints. For this purpose, a file named **gz_ros2_control.yaml** was also created within the **gazebo_ackermann_steering_vehicle/config** directory to declare the controller configurations. The contents of this file are shown below:

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

To bring the entire system together, a ROS 2 launch file was created to automate node startup, parameter loading, and the initialization of the Gazebo simulator. The launch file organized nodes and runtime settings in a clear and structured way, simplifying overall system integration. Implemented in Python, it also allowed additional flexibility, such as passing the simulation world and the vehicle’s initial pose as launch arguments. For this purpose, a **vehicle.launch.py** file was created and organized within the **gazebo_ackermann_steering_vehicle/launch** directory, and its contents used to start the simulation are shown below:

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

With all configuration and launch files in place, the vehicle model was fully integrated with the simulator and control systems, completing the simulation setup. To run the ROS 2 application, the environment was initialized by sourcing the ROS 2 Jazzy setup file, building the workspace to compile the required packages, and sourcing the workspace setup file to make the packages available. Finally, the application was launched to start the simulation. The commands used for these steps are shown below:


```bash
source /opt/ros/jazzy/setup.bash

cd ~/workspace

colcon build

source install/setup.bash

ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py
```

Once the setup was executed successfully, launching **vehicle.launch.py** started the Gazebo simulator and spawned the vehicle defined by the Xacro model. The vehicle then appeared in the simulation environment and could be observed within the Gazebo world, as shown in the image below:

![Ackermann Steering Vehicle Model in Gazebo Simulation]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/ackermann_steering_vehicle_model_in_gazebo_simulation.gif)
*Figure 2 — Ackermann Steering Vehicle Model in Gazebo Simulation.*

To start the vehicle in a specific simulation world with a custom initial pose, the **vehicle.launch.py** file was executed with the appropriate launch arguments. These arguments specified the world file path, the initial **x**, **y**, and **z** position of the vehicle, and the initial roll (**R**), pitch (**P**), and yaw (**Y**) orientation. In the example shown below, the vehicle was initialized at position (**x**, **y**, **z**) = (1.0, 2.0, 0.5) with a yaw angle of 1.57 radians in the selected world:

``` bash
ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py world:=world.sdf x:=1.0 y:=2.0 z:=0.5 R:=0.0 P:=0.0 Y:=1.57
```

### Vehicle Control

The **gz_ros2_control** package was used to control the vehicle’s joints through feedforward controllers, allowing vehicle behavior to be implemented without direct management of individual joints, as low-level actuation was handled internally by the **gz_ros2_control**.


The **gz_ros2_control.yaml** file, located in the **gazebo_ackermann_steering_vehicle/config** directory, was configured to define two different feedforward controllers: **forward_velocity_controller**, responsible for controlling the rear wheel motion, and **forward_position_controller**, which managed the front steering joints. These controllers were connected to the ROS 2 interface through the **/forward_velocity_controller/commands** and **/forward_position_controller/commands** topics, enabling direct control of vehicle motion and steering. Each topic received a **Float64MultiArray** message containing the desired velocity or position values for the corresponding joints, ordered as specified in the configuration file. Because the vehicle followed an Ackermann steering geometry with a differential rear-wheel model, a control interface was also needed to convert the desired vehicle steering angle and velocity into the appropriate joint positions and joint velocities.

As a result, a dedicated ROS 2 node was implemented to subscribe to the **/steering_angle** and **/velocity** topics, which provided the desired vehicle steering angle (in radians) and linear velocity (in m/s), respectively. The node processed these inputs to compute the corresponding joint positions and joint velocities according to the Ackermann steering geometry and the differential rear-wheel model defined earlier. The computed joint positions and velocities were then published to the **/forward_position_controller/commands** and **/forward_velocity_controller/commands** topics exposed by the **gz_ros2_control** package.

Moving on, the track width and wheelbase were required to compute the Ackermann steering angles for each wheel. These values were derived from parameters defined in the **gazebo_ackermann_steering_vehicle/config/params.yaml** file, which specified the vehicle body width, body length, wheel width, and wheel radius. In addition, the maximum steering angle and maximum allowable velocity were used to constrain the controller computations. For this reason, these parameters were also loaded by the node to ensure consistent and bounded control behavior.

An additional safety feature was implemented in the form of a timeout mechanism. If updated steering angle or velocity commands were not received within a specified time interval, the vehicle was commanded to stop until new commands became available. Although the system operated in a simulated environment where hardware failures were not a concern, this mechanism was included to maintain consistent behavior and to support potential reuse of the same control logic on a real vehicle.

To implement this node, a header file named **vehicle_controller.hpp** was created within the **gazebo_ackermann_steering_vehicle/include** directory. This file declared the function interfaces **ackermann_steering_angle**, used to compute the steering angle for each wheel based on Ackermann geometry, and **rear_differential_velocity**, responsible for calculating the rear wheel velocities according to the differential model. It also defined the **timer_callback** function for publishing control commands to the **gz_ros2_control** interface, along with the **steering_angle_callback** and **velocity_callback** functions for processing incoming messages from the **/steering_angle** and **/velocity** topics, respectively. The complete header file is shown below:

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

With the header file and function interfaces defined, the node implementation was completed. A source file named **vehicle_controller.cpp** was created within the **gazebo_ackermann_steering_vehicle/src** directory to implement the logic of the **VehicleController** class. In this implementation, the node loaded the required parameters, processed incoming steering angle and velocity commands, computed the corresponding steering angles and wheel speeds using the Ackermann steering model, and published control commands at a fixed rate. The timeout mechanism was also included to safely stop the vehicle when command updates were no longer received. These elements are combined in the implementation shown below:

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

To ensure proper integration of the system, the **gazebo_ackermann_steering_vehicle/launch/vehicle.launch.py** file was updated to start the **vehicle_controller** node alongside the vehicle simulation. In addition, the launch configuration was modified to pass the required parameters to the **vehicle_controller** node. These updates are shown below:

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

Finally, the **CMakeLists.txt** file was updated to include the **vehicle_controller** node as a build dependency of the **gazebo_ackermann_steering_vehicle** package:

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

With this implementation, the vehicle control interface was completed, incorporating all the elements required to control the vehicle using the Ackermann steering model. To proceed, the **gazebo_ackermann_steering_vehicle** package was built and **vehicle.launch.py** was executed by running the following commands in the workspace:

```bash
source /opt/ros/jazzy/setup.bash

cd ~/workspace

colcon build

source install/setup.bash

ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py
```

In conclusion, once the vehicle is launched using the **vehicle.launch.py** file, the **/steering_angle** and **/velocity** topics become available to control the vehicle. From there, opening a new terminal, sourcing the ROS environment, and publishing messages to these topics allows direct control of the vehicle’s motion and steering.

For instance, two new command lines were opened, and in each one the ROS 2 Jazzy setup file and the workspace environment setup file were sourced, as shown below:

```bash
source /opt/ros/jazzy/setup.bash

cd ~/workspace

source install/setup.bash
```

After that, in the first command line, the desired vehicle steering angle was published.

```bash
ros2 topic pub /steering_angle std_msgs/msg/Float64 "{data: 0.5}" --rate 10
```

In the other one, the desired vehicle's velocity was published:

```bash
ros2 topic pub /velocity std_msgs/msg/Float64 "{data: 0.5}" --rate 10
```

After the desired steering angle and velocity were published to the vehicle controller node, the vehicle began moving and followed a circular path in the simulation, as illustrated in the image below:

![Ackermann Steering Vehicle Control in Simulation]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/ackermann_steering_vehicle_control.gif)
*Figure 5 — Ackermann Steering Vehicle Control in Simulation.*

### Joystick Controller

In many robotics and simulation workflows, having manual control over the vehicle is just as important as autonomous functionality. Whether you're testing control algorithms, validating system behavior, or collecting training data for Deep Learning projects, the ability to drive the vehicle manually adds flexibility to the setup.

To support this, a joystick-based control interface was implemented, enabling users to steer the vehicle and adjust its velocity using an Xbox One wireless controller. In this setup, the **L3** analog stick was mapped to steering input, while the **R3** stick was used for throttle control, as shown in the image below. When using a different joystick model, software adjustments may be required, as button and axis mappings can vary between devices.

![Ackermann Steering Vehicle Control in Simulation]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/xbox_joystick_controller.webp)
*Figure 6 — Xbox Joystick Controller.*

To add joystick-based control, the **gazebo_ackermann_steering_vehicle/include** directory was updated by creating the **joystick_controller.hpp** header file. This file defined the **JoystickController** class, responsible for subscribing to joystick input messages and publishing the corresponding steering angle and velocity commands. In practice, this node handled the interpretation of joystick inputs and translated them into commands used to drive the vehicle. The complete header file implementation is shown below.


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

Next, the **gazebo_ackermann_steering_vehicle/src** directory was updated by creating the **joystick_controller.cpp** file to implement the joystick control node. This node subscribed to the **/joy** topic to read joystick input messages and loaded the required parameters from **gazebo_ackermann_steering_vehicle/config/params.yaml** during initialization. The left and right analog stick axes were used to control the steering angle and vehicle velocity as scaled fractions of their respective maximum values. A **timer_callback** function was also implemented to periodically publish these commands to the **/steering_angle** and **/velocity** topics. The complete source code implementation is shown below.


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

Then, a launch file named **joystick.launch.py** was created inside the **gazebo_ackermann_steering_vehicle/launch** directory. This file was used to start two nodes: the **joy_node** from the **joy** package, responsible for reading joystick input and publishing it on the **/joy** topic, and the **joystick_controller** node from this package, which converted joystick input into steering and velocity commands. The **joystick_controller** node also loaded parameters from **gazebo_ackermann_steering_vehicle/config/parameters.yaml** to ensure correct control behavior. The launch configuration retrieved the parameter file path and then started both nodes with the required settings, as shown below.

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

Additionally, the **CMakeLists.txt** file was updated to include the required dependencies needed for the joystick control functionality.

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

With this node in place, the vehicle simulation could now be driven manually using a joystick. To use the controller, the **vehicle.launch.py** file was launched first. Then, in a separate terminal, the **joystick.launch.py** file was started to enable joystick-based control, as shown below:

```bash
source /opt/ros/jazzy/setup.bash

cd ~/workspace

colcon build

source install/setup.bash

ros2 launch gazebo_ackermann_steering_vehicle joystick.launch.py
```

It was now possible to control the vehicle in the simulation using an Xbox One controller, taking advantage of its joysticks to drive and steer the robot. At this stage, the **gazebo_ackermann_steering_vehicle** package was complete, bringing together vehicle simulation, control logic, and manual operation. The final structure of the package is shown below:

```text
gazebo_ackermann_steering_vehicle/
├── config/
│   ├── gz_ros2_control.yaml
│   ├── parameters.yaml
│   └── ros_gz_bridge.yaml
├── include/
│   ├── joystick_controller.hpp
│   └── vehicle_controller.hpp
├── launch/
│   ├── joystick.launch.py
│   └── vehicle.launch.py
├── model/
│   └── vehicle.xacro
├── src/
│   ├── joystick_controller.cpp
│   └── vehicle_controller.cpp
├── CMakeLists.txt
└── package.xml
```

## Results

Although setting up the simulation required configuring multiple files and integrating several packages, the final result was a fully functional system. Once the joystick controller was connected, the vehicle could be driven interactively using a video game controller.

![Vehicle Simulation and Control]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/vehicle_simulation_and_control.gif)
*Figure 7 — Vehicle Simulation and Control.*

In addition to the topics discussed earlier, the system also provides the **joint_states** topic, which publishes **sensor_msgs/msg/JointState** messages describing the current state of the robot’s joints and can be used for monitoring or visualization in tools such as **RViz**, as well as the **camera/image_raw** topic, which publishes **sensor_msgs/msg/Image** messages from the onboard camera and can be leveraged for computer vision experiments or for training machine learning models for tasks like object detection and navigation.

![Vehicle Visualization in RViz]({{ site.url }}{{ site.baseurl }}/public/images/ackermann-steering-vehicle-simulation/vehicle_visualization_in_rviz.gif)
*Figure 8 — Vehicle Visualization in RViz.*

## Conclusion

The simulation of an Ackermann steering vehicle using Gazebo Harmonic and ROS 2 Jazzy Jalisco demonstrates the strengths of robotics tools, combining a lightweight yet capable simulation environment with a modular and scalable framework. By modeling the vehicle in a virtual setting, the project enables safe and efficient testing of control algorithms and realistic driving scenarios without physical hardware, while also providing a solid foundation for future work in perception and autonomous navigation.
