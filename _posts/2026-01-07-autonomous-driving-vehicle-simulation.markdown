---
title: "Autonomous Driving Vehicle Simulation"
date: 2026-01-07 19:00:00
description: A self-driving vehicle simulation in Gazebo and ROS 2, using a neural network to predict steering angles and velocity only from front camera images.
author: lucasmazzetto
keywords: robot, simulation, gazebo, ros, neural network, convolutional neural network, deep learning, robotic simulation, autonomous robots, Gazebo simulation, ROS 2, ROS robot simulation, neural network training, AI in robotics, CNN, deep neural networks, machine learning models, robotics AI, reinforcement learning for robots, computer vision in robotics, self-driving car simulation, neural network-based control, autonomous vehicle technology, robotics and AI integration, simulation environments for robots, real-time robotics simulation, robotic perception
---

This project presents an end-to-end autonomous driving system developed using Gazebo Harmonic and ROS 2 Jazzy Jalisco, in which a Convolutional Neural Network (CNN) predicts steering angles and velocity from camera images. Manual driving data was collected to establish an imitation learning pipeline. The trained model demonstrated generalization by autonomously navigating a previously unseen track in simulation. While the experiments were conducted in a simulated environment, the proposed methodology can be directly applied to physical robotic platforms, providing a framework for autonomous driving systems.

## Introduction 

In the advancing field of autonomous robots, simulation plays a crucial role in providing a controlled environment to test algorithms, evaluate performance, and refine designs without the risks and costs of physical prototyping. Since real-world accidents can also occur, simulations offer a way to identify potential issues early and reduce the chances of unsafe situations during deployment. These simulations enable researchers to replicate real-world scenarios, accelerate development cycles, and explore novel solutions in robotics. 

Another important topic in the field of robotics is artificial intelligence. Neural networks are enabling systems to process data and make intelligent decisions where traditional techniques could fail, such as in object detection, motion control, and even navigation. Among them, Convolutional Neural Networks (CNNs) are especially useful for computer vision because they learn features like edges, textures, and patterns to interpret images. This improves a robot’s perception, helping it adapt to its surroundings and make better decisions when navigating complex environments.

Building from these ideas, the project presented in this article simulates a self-driving vehicle controlled by a CNN to follow lanes using images from an embedded camera. The simulation was made with Gazebo, a popular robotics simulator, with ROS 2 serving as the robotics framework. 

The complete project source code, including all files and implementation, is available in the [project’s repository](https://github.com/workabotic/gazebo_autonomous_driving). You are free to clone it, suggest improvements, contribute, or use it in any way you find useful.


## Requirements

For the development of this project, the following software versions were used:

- [**Linux Ubuntu 24.04**](https://ubuntu.com/blog/tag/ubuntu-24-04-lts) was used as the operating system.
- [**ROS 2 Jazzy Jalisco**](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html) was used as the robotics framework.
- [**Gazebo Harmonic**](https://gazebosim.org/docs/harmonic/getstarted/) was used as the simulation environment.

Once ROS 2 Jazzy Jalisco was installed, the required packages were added by running the following command in the terminal. These packages were necessary for controlling the robot, bridging Gazebo and ROS 2, and managing robot states and configurations:

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

Then, a [ROS 2 workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) was set up on the computer. To do that, the next command was executed to create the workspace directory:

```bash
mkdir -p ~/workspace/src
```

Another required package was [gazebo_ackermann_steering_vehicle](https://github.com/workabotic/gazebo_ackermann_steering_vehicle), which was used to provide the vehicle simulation for this project. This package had been developed in a previous project, and its documentation could be found on the [Ackermann Steering Vehicle Simulation]({{ site.url }}{{ site.baseurl }}/2025/ackermann-steering-vehicle-simulation/) page. To add the required package to the project, the repository was cloned directly into the workspace, following the commands shown below:

```bash
cd ~/workspace/src

git clone git@github.com:workabotic/gazebo_ackermann_steering_vehicle.git
```

At this stage, it was necessary to install the Python dependencies required by the project. To achieve this, a Python virtual environment named **.venv** was created and activated. Unlike a fully isolated virtual environment, this environment was configured to allow access to system-wide Python packages, enabling proper integration with the ROS 2 Python ecosystem. This approach was required because some ROS 2 dependencies, such as **cv_bridge** and **OpenCV**, are provided by the system and must be accessible at runtime alongside the Python packages installed within the virtual environment. The virtual environment was created and activated using the commands shown below:


```bash
python3 -m venv ~/.venv --system-site-packages
```

After activating the virtual environment, the required Python dependencies were installed using **pip**. These dependencies were used for data processing tasks as well as for handling neural network training and inference within the project. The commands presented below were used to install the dependencies:

```bash
source ~/.venv/bin/activate

pip install numpy==1.26.4 \
            torch==2.9.1 \
            torchvision==0.24.1 \
            tensorboard==2.20.0 \
            tqdm==4.67.1 \
            PyYAML==6.0.3 \
            typeguard==4.4.4 \
            rosdep==0.26.0 \
            setuptools==79.0.1 \
            colcon-common-extensions==0.3.0
```

Finally, the virtual environment was activated together with the ROS 2 environment, and the workspace was built. The following commands were used to perform these steps:

```bash
source /opt/ros/jazzy/setup.bash

source ~/.venv/bin/activate

cd ~/workspace

colcon build
```

## Methods

In this project, multiple racing tracks were created to generate different driving scenarios, with one track reserved exclusively for validation to ensure proper generalization of the neural network model. Data were collected by manually driving the simulated vehicle using a video game controller, recording both camera images and control actions. After collecting and preparing the dataset, a neural network was designed to predict steering angle and velocity directly from the images. It was trained using the recorded data and later validated on the reserved track to ensure reliable performance. Finally, an inference node was implemented to enable real-time inference, allowing the vehicle to operate autonomously. The following sections describe each stage of the development in detail.

### Racing Tracks

Four racing tracks with different shapes were designed using [Blender](https://www.blender.org/) and integrated into the Gazebo simulator to provide different driving environments. To increase variability in the collected data, each track was created with distinct background textures, including dirt, sand, snow, and grass. In addition to varying the track shapes and background, modifications were also applied to the boundary stripe colors, enhancing dataset diversity. After modeling, the tracks were exported from Blender as **.obj** and **.mtl** files, along with their corresponding background texture assets, and were placed in the [project repository](https://github.com/lucasmazzetto/gazebo_racing_tracks/tree/main/worlds/meshes) for download.

![Racing tracks designed with Blender]({{ site.url }}{{ site.baseurl }}/public/images/autonomous-driving-vehicle-simulation/racing-tracks-designed-with-blender.webp)
*Figure 1 — Racing tracks designed with Blender.*

To manage these environments within the simulation, a new ROS 2 package named **gazebo_racing_tracks** was created inside the workspace. This package was added to the workspace’s src directory ensuring that all resources could be used within the project. To do this, the following commands were executed to create the package:

```bash
cd ~/workspace/src

ros2 pkg create gazebo_racing_tracks --build-type ament_cmake
```

Then, a directory named **worlds/meshes** was created inside the **gazebo_racing_tracks** package, and the racing track files were moved into **gazebo_racing_tracks/worlds/meshes** for proper use in the project.

```bash
mkdir -p ~/workspace/src/gazebo_racing_tracks/worlds/meshes

mv ~/meshes/* ~/workspace/src/gazebo_racing_tracks/worlds/meshes
```

As a result, the **gazebo_racing_tracks** package then had the following directory structure:


```text
gazebo_racing_tracks/
├── worlds/
│   └── meshes/
│       ├── dirt.jpg
│       ├── dirt_track.mtl
│       ├── dirt_track.obj
│       ├── grass.jpg
│       ├── grass_track.mtl
│       ├── grass_track.obj
│       ├── sand.jpg
│       ├── sand_track.mtl
│       ├── sand_track.obj
│       ├── snow.jpg
│       ├── snow_track.mtl
│       ├── snow_track.obj
├── CMakeLists.txt
└── package.xml
```

To use the tracks in Gazebo, it was necessary to create corresponding world files and link them to the track assets. This required four world files representing each track in the simulator: **dirt_track.sdf**, **sand_track.sdf**, **snow_track.sdf**, and **grass_track.sdf**. These files defined the environment, including textures and any additional elements, ensuring that Gazebo could load and simulate each track correctly.

The **dirt_track.sdf** file was created inside the **gazebo_racing_tracks/worlds** folder, and the following Gazebo world definition was inserted:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="racing_track">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <light type="directional" name="sun">
        <cast_shadows>true</cast_shadows>
        <pose>0 0 10 0 0 0</pose>
        <diffuse>0.8 0.8 0.8 1</diffuse>
        <specular>0.2 0.2 0.2 1</specular>
        <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
        </attenuation>
        <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="dirt_track">
      <pose>0 0 0.001 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>meshes/dirt_track.obj</uri>
            </mesh>
          </geometry>
        </visual>

        <collision name="collision">
          <geometry>
            <mesh>
              <uri>meshes/dirt_track.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

For the sand track, a file named **sand_track.sdf** was placed in the **gazebo_racing_tracks/worlds** folder, and the track was defined using the following world definition:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="racing_track">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <light type="directional" name="sun">
        <cast_shadows>true</cast_shadows>
        <pose>0 0 10 0 0 0</pose>
        <diffuse>0.8 0.8 0.8 1</diffuse>
        <specular>0.2 0.2 0.2 1</specular>
        <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
        </attenuation>
        <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="sand_track">
      <pose>0 0 0.001 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>meshes/sand_track.obj</uri>
            </mesh>
          </geometry>
        </visual>

        <collision name="collision">
          <geometry>
            <mesh>
              <uri>meshes/sand_track.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

To set up the snow track, a **snow_track.sdf** file was created within **gazebo_racing_tracks/worlds**, and the following world details were inserted for Gazebo:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="racing_track">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <light type="directional" name="sun">
        <cast_shadows>true</cast_shadows>
        <pose>0 0 10 0 0 0</pose>
        <diffuse>0.8 0.8 0.8 1</diffuse>
        <specular>0.2 0.2 0.2 1</specular>
        <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
        </attenuation>
        <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="snow_track">
      <pose>0 0 0.001 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>meshes/snow_track.obj</uri>
            </mesh>
          </geometry>
        </visual>

        <collision name="collision">
          <geometry>
            <mesh>
              <uri>meshes/snow_track.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

Finally, for the grass track, a **grass_track.sdf** file was added inside the **gazebo_racing_tracks/worlds** directory, and it was filled with the following world definition:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="racing_track">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <light type="directional" name="sun">
        <cast_shadows>true</cast_shadows>
        <pose>0 0 10 0 0 0</pose>
        <diffuse>0.8 0.8 0.8 1</diffuse>
        <specular>0.2 0.2 0.2 1</specular>
        <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
        </attenuation>
        <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="grass_track">
      <pose>0 0 0.001 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>meshes/grass_track.obj</uri>
            </mesh>
          </geometry>
        </visual>

        <collision name="collision">
          <geometry>
            <mesh>
              <uri>meshes/grass_track.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```


It was also necessary to modify the **CMakeLists.txt** file to include the **gazebo_racing_tracks/worlds** directory. This ensured that the track files could be properly located by other ROS packages within the same workspace, simplifying their integration across different packages. Consequently, the **CMakeLists.txt** content was updated as follows:

```cmake
cmake_minimum_required(VERSION 3.8)
project(gazebo_racing_tracks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

Once all steps had been carried out correctly, the **gazebo_racing_tracks** package directory presented the following structure:

```text
gazebo_racing_tracks/
├── worlds/
│   ├── meshes/
│   │   ├── dirt.jpg
│   │   ├── dirt_track.mtl
│   │   ├── dirt_track.obj
│   │   ├── grass.jpg
│   │   ├── grass_track.mtl
│   │   ├── grass_track.obj
│   │   ├── sand.jpg
│   │   ├── sand_track.mtl
│   │   ├── sand_track.obj
│   │   ├── snow.jpg
│   │   ├── snow_track.mtl
│   │   └── snow_track.obj
│   ├── dirt_track.sdf
│   ├── grass_track.sdf
│   ├── sand_track.sdf
│   └── snow_track.sdf
├── CMakeLists.txt
└── package.xml
```

After the **gazebo_racing_tracks** package was set up, the workspace was accessed, and the project was built using colcon build. This step compiled all packages in the workspace and ensured that the tracks and associated files were properly recognized by ROS 2. In summary, the command used was:

```bash
source /opt/ros/jazzy/setup.bash

source ~/.venv/bin/activate

cd ~/workspace

colcon build
```

### Dataset Collection

For the dataset creation, after the **gazebo_racing_tracks** package had been set up, the **gazebo_ackermann_steering_vehicle** package was also required. As mentioned before, this package is available on the [project’s repository](https://github.com/workabotic/gazebo_ackermann_steering_vehicle) page and could be cloned directly into the workspace folder. Alternatively, it is possible to look at the [documentation](https://workabotic.com/2025/ackermann-steering-vehicle-simulation/) and follow it to develop a custom vehicle model instead.

Anyway, after these dependencies were in place, it was time to create a new package responsible for handling data collection and managing the neural network related tasks, such as training and inference. This package was named **autopilot_neural_network**, as it brought all these functions together. The entire package, including its nodes and useful scripts, was developed in **Python**. From that point onward, this package was used and updated continuously until the end of this project.

To create the **autopilot_neural_network** package, the workspace source directory was first accessed. Then, a new ROS 2 Python package was generated using **ament_python** as the build system, while explicitly declaring its runtime dependencies. The commands executed to perform these steps are shown below:

```bash
source /opt/ros/jazzy/setup.bash

source ~/.venv/bin/activate

cd ~/workspace/src

ros2 pkg create autopilot_neural_network \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs std_msgs cv_bridge
```

The following directory structure was then created, with the **autopilot_neural_network/autopilot_neural_network** folder serving as the default location for the Python nodes:

```bash
autopilot_neural_network/
├── autopilot_neural_network/
│   └── __init__.py
├── resource/
│   └── autopilot_neural_network
├── package.xml
├── setup.py
└── setup.cfg
```

Because ROS 2 Python nodes may attempt to use the system Python interpreter instead of the one provided by the active virtual environment, the colcon configuration was adjusted to ensure that the active environment was used. In summary, this guaranteed that both the build and execution processes relied on the Python interpreter from the sourced virtual environment rather than the system default. To do this, **autopilot_neural_network/setup.cfg** was updated by adding the appropriate configuration section, as shown below, ensuring that the correct Python interpreter was used during execution and installation:

```bash
[develop]
script_dir=$base/lib/autopilot_neural_network
[install]
install_scripts=$base/lib/autopilot_neural_network
[build_scripts]
executable=/usr/bin/env python3
```

After that, the main idea was to create a data collector software. Its primary goal was to gather data from the vehicle while it was being manually driven. Additionally, it needed to receive vehicle parameters, such as maximum velocity and maximum steering angle, so that the collected data can later be normalized during the training process.

However, these parameters were already defined in the **gazebo_ackermann_steering_vehicle** package, specifically within the **vehicle_controller** node. To avoid duplicating configuration and to improve maintainability, it was decided to retrieve these parameters directly from the **vehicle_controller** node. Furthermore, this design was intended to be extensible for any other parameters, not just maximum velocity or maximum steering angle. As a result, a class was created to request any desired parameter from other nodes, which could then be inherited by other classes, serving as a generic parameters client node.

Building on this approach, the **ParametersClientNode** class was designed to retrieve parameters from other ROS 2 nodes through synchronous service calls to the **GetParameters** service. The decision to use synchronous calls to the **GetParameters** service was motivated by the need to access these values during node initialization, before the main execution logic could safely proceed. The class creates a client for the target node, waits for the service to become available, and blocks execution until the requested parameters are returned or a timeout occurs.

Once a response is received, the returned **ParameterValue** messages are converted into native Python types through the **_extract_parameter_value** method, which supports both scalar and array parameter types. To keep the interface flexible while remaining convenient to use, the **request_parameters** method was designed to return a single value when only one parameter is requested, or a tuple of values when multiple parameters are requested, preserving the original request order. With this structure, other nodes can simply inherit from **ParametersClientNode** and synchronously fetch any required parameters at startup without duplicating logic. The **ParametersClientNode** class was implemented in the **parameters_client_node.py** file, located inside the **autopilot_neural_network/autopilot_neural_network** directory, and the full implementation is shown below:


```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType
from typing import Any, List, Tuple


class ParametersClientNode(Node):
    """
    Base class for ROS 2 nodes that need to communicate with another node
    to retrieve parameters.

    This class simplifies the process of querying parameters from other ROS 2 nodes.
    It provides synchronous service calls to the GetParameters service and returns
    the requested values.
    """

    def __init__(self, node_name: str) -> None:
        """
        @brief Initializes the parameter client node.

        @param node_name Name to assign to this node in the ROS graph.
        """
        super().__init__(node_name)

    def request_parameters(self, target_node_name: str, param_names: List[str], 
                           timeout_sec: float = 2.0) -> Any | Tuple[Any, ...]:
        """
        @brief Synchronously requests parameters from a remote ROS 2 node.

        This method blocks until the GetParameters service responds or the
        specified timeout expires. Returned values are converted into native
        Python types and returned as a tuple in the same order as requested.

        @param target_node_name Name of the node providing the parameters.
        @param param_names List of parameter names to request.
        @param timeout_sec Maximum time to wait for the service response.

        @return Parameter value or tuple of parameter values.
        """
        # Create a client for the GetParameters service
        service_name = f"/{target_node_name}/get_parameters"
        client = self.create_client(GetParameters, service_name)

        # Wait for the service to become available
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError(f"GetParameters service not available: {service_name}")

        # Create the service request and set parameter names
        request = GetParameters.Request()
        request.names = param_names

        # Send the request asynchronously
        future = client.call_async(request)

        # Spin the node until the service response arrives or times out
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        # Fail if the service did not respond in time
        if not future.done():
            raise TimeoutError(f"Timeout while requesting parameters {param_names} "
                               f"from node '{target_node_name}'")

        # Get the result response
        response = future.result()
        
        # Validate response size matches requested parameters
        if response is None or len(response.values) != len(param_names):
            raise RuntimeError(f"Invalid parameter response from node '{target_node_name}'")

        # Convert ROS ParameterValue messages into native Python values
        values = tuple(self._extract_parameter_value(value) for value in response.values)

        # Return a single value or a tuple depending on request size
        return values[0] if len(values) == 1 else values

    def _extract_parameter_value(self, value: ParameterValue) -> Any:
        """
        @brief Converts a ROS 2 ParameterValue message into a Python-native value.

        @param value ParameterValue message returned by GetParameters.
        @return Corresponding Python value, or None if the type is unsupported.
        """
        match value.type:
            case ParameterType.PARAMETER_BOOL:
                return value.bool_value
            case ParameterType.PARAMETER_INTEGER:
                return value.integer_value
            case ParameterType.PARAMETER_DOUBLE:
                return value.double_value
            case ParameterType.PARAMETER_STRING:
                return value.string_value
            case ParameterType.PARAMETER_BYTE_ARRAY:
                return bytes(value.byte_array_value)
            case ParameterType.PARAMETER_INTEGER_ARRAY:
                return list(value.integer_array_value)
            case ParameterType.PARAMETER_DOUBLE_ARRAY:
                return list(value.double_array_value)
            case ParameterType.PARAMETER_BOOL_ARRAY:
                return list(value.bool_array_value)
            case ParameterType.PARAMETER_STRING_ARRAY:
                return list(value.string_array_value)
            case _:
                self.get_logger().warn(f"Unknown parameter type received: {value.type}")
                return None
```

As a result, the **DataCollector** class was then implemented as a node inheriting from **ParametersClientNode**. Its purpose was to retrieve the vehicle’s parameters, subscribe to the vehicle’s camera, steering, and velocity topics, and continuously gather synchronized state information. By using the **request_parameters** method from the parent class, the **DataCollector** could access values such as maximum velocity and steering limits while focusing on monitoring incoming data and determining when a valid sample was available for storage.

The data collection loop worked by continuously updating the latest steering, velocity, and image messages as they arrived, each tagged with its own timestamp. Instead of forcing strict synchronization, the node simply checked during the periodic update whether all fields had been refreshed recently enough and whether the vehicle was moving above a minimum speed threshold. When these conditions were met, the current state was treated as a synchronized snapshot of the system, and the sample was saved. This ensured that the steering angle, velocity, and camera frame all referred to roughly the same moment in time, so the model could learn from the commands that the vehicle was actually following, keeping the dataset consistent for training.

To keep this node flexible for whatever future projects might need it, the **DataCollector** was also designed to load some parameters for itself at startup, allowing the redefinition of which node provides vehicle data and which topics should be monitored. Parameters like **vehicle_node**, **image_topic**, **velocity_topic**, and **steering_angle_topic** made the node easy to adjust for setups that didn’t use the **vehicle_controller** node or that required receiving data under different topic names.

In addition, the parameters **max_velocity_parameter** and **max_steering_angle_parameter** were added to allow changing the vehicle parameter names used to fetch the maximum velocity and maximum steering angle when necessary, keeping the node flexible across different configurations. Additional parameters also provided control over the data collection process: **dataset_path** defined where images and CSV metadata should be stored, **update_period** set how frequently the main loop evaluated the current state, **timeout_time** specified how long data could remain unrefreshed before being considered invalid, and **min_velocity_factor** determined the minimum percentage of the maximum velocity required for a sample to be stored.

Also, to handle the incoming data in an organized way, a **VehicleState** dataclass was added to group together the latest camera frame, steering angle, velocity, and their timestamps into one data structure. This state object was what got passed to the storage layer. Saving each sample was handled by the **CsvDataStorage** class, which made sure the dataset directory existed, gave each image a sequential ID, and stored them as PNG files. It also wrote a matching row in a CSV file with the state values, timestamps, and vehicle limits.

The complete implementation of all this logic was placed in the **data_collector.py** file inside the **autopilot_neural_network/autopilot_neural_network** directory, with the full code shown below:

```python
import os
import rclpy
import csv
import cv2

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from autopilot_neural_network.parameters_client_node import ParametersClientNode
from dataclasses import dataclass
from cv_bridge import CvBridge


@dataclass
class VehicleState:
    steering_angle = None
    velocity = None
    image = None
    steering_angle_time = None
    velocity_time = None
    image_time = None
    

class CsvDataStorage:
    def __init__(self, dataset_path: str):
        """
        @brief Initializes the CSV data storage handler.

        Creates the dataset directory if it does not exist, and prepares the CSV file path.

        @param dataset_path Path to the dataset directory where images and CSV files are stored.
        """
        self.dataset_path = dataset_path
        self.csv_file_path = os.path.join(self.dataset_path, 'dataset.csv')
        self.bridge = CvBridge()

        if not os.path.exists(self.dataset_path):
            os.makedirs(self.dataset_path)

    def _get_next_image_id(self) -> int:
        """
        @brief Retrieves the next available image ID for saving a new image.

        Scans the dataset directory for existing images and determines the next sequential ID.

        @return The next image ID as an integer.
        """
        os.makedirs(self.dataset_path, exist_ok=True)  # Ensure directory exists

        existing_images = [f for f in os.listdir(self.dataset_path) if f.endswith('.png')]
        if not existing_images:
            return 0

        existing_ids = [int(os.path.splitext(f)[0]) for f in existing_images if f.split('.')[0].isdigit()]
        
        return max(existing_ids) + 1

    def _save_image(self, image_id: int, image: Image) -> str:
        """
        @brief Saves a ROS Image message as a PNG file.

        Converts the ROS Image message to an OpenCV image and writes it to the dataset directory.

        @param image_id The numeric ID of the image to save.
        @param image The Image message received from ROS.
        @return The full path to the saved image file.
        """
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        image_path = os.path.join(self.dataset_path, f'{image_id}.png')
        cv2.imwrite(image_path, cv_image)
        return image_path

    def _append_csv_row(self, image_id: int, state: VehicleState, 
                        max_velocity: float, max_steering_angle: float): 
        """
        @brief Appends a new row of vehicle state data to the dataset CSV file.

        If the CSV file does not exist yet, a header row is written before appending data.

        @param image_id The ID associated with the image for this data sample.
        @param state The VehicleState object containing the state of the vehicle.
        """
        write_header = not os.path.exists(self.csv_file_path) 
        
        with open(self.csv_file_path, 'a', newline='') as csvfile: 
            writer = csv.writer(csvfile) 
            
            if write_header: 
                writer.writerow(['image_id', 'velocity', 'steering_angle', 
                                 'image_time', 'velocity_time', 'steering_angle_time',
                                 'max_velocity', 'max_steering_angle']) 
                
            writer.writerow([image_id, 
                             state.velocity, 
                             state.steering_angle, 
                             state.image_time.nanoseconds / 1e9, # To UNIX timestamp
                             state.velocity_time.nanoseconds / 1e9, # To UNIX timestamp
                             state.steering_angle_time.nanoseconds / 1e9, # To UNIX timestamp
                             max_velocity, 
                             max_steering_angle])

    def save_sample(self, state: VehicleState, max_velocity: float, max_steering_angle: float):
        """
        @brief Saves a complete sample, including the image and corresponding state data.

        This method assigns an image ID, saves the image file, and appends metadata to the CSV file.

        @param state The VehicleState object containing vehicle state data.
        """
        image_id = self._get_next_image_id()
        self._save_image(image_id, state.image)
        self._append_csv_row(image_id, state, max_velocity, max_steering_angle)
    
    
class DataCollector(ParametersClientNode):
    """
    This node subscribes to vehicle-related topics, retrieves parameters from the 
    vehicle controller, and periodically saves synchronized image and state data to a dataset.
    """
        
    def __init__(self):
        super().__init__("data_collector")

        # Load ROS parameters
        self.declare_parameter("min_velocity_factor", 0.25)
        self.declare_parameter("timeout_time", 1_000_000_000)
        self.declare_parameter("update_period", 0.5)
        self.declare_parameter("dataset_path", "/tmp/autopilot_neural_network/dataset")
        
        self.declare_parameter("vehicle_node", "vehicle_controller")
        self.declare_parameter("velocity_topic", "vehicle_controller")
        self.declare_parameter("steering_angle_topic", "vehicle_controller")
        self.declare_parameter("image_topic", "vehicle_controller")
        
        self.declare_parameter("max_velocity_parameter", "max_velocity")
        self.declare_parameter("max_steering_angle_parameter", "max_steering_angle")

        # Retrieve parameters
        self.min_velocity_factor = self.get_parameter("min_velocity_factor").value
        self.timeout_time = self.get_parameter("timeout_time").value
        self.update_period = self.get_parameter("update_period").value
        self.dataset_path = self.get_parameter("dataset_path").value
       
        self.vehicle_node = self.get_parameter("vehicle_node").value
        self.velocity_topic = self.get_parameter("velocity_topic").value
        self.steering_angle_topic = self.get_parameter("steering_angle_topic").value
        self.image_topic = self.get_parameter("image_topic").value
        
        max_velocity_parameter = self.get_parameter("max_velocity_parameter").value
        max_steering_angle_parameter = self.get_parameter("max_steering_angle_parameter").value

        # Vehicle controller parameters
        self.max_velocity = None
        self.max_steering_angle = None

        # Request vehicle controller parameters (max_velocity, max_steering_angle)
        self.max_velocity, self.max_steering_angle = self.request_parameters(
            self.vehicle_node, [max_velocity_parameter, max_steering_angle_parameter])

        # Storage layer
        self.storage = CsvDataStorage(self.dataset_path)
        self.last_state = VehicleState()

        # QoS: keep only the latest message, replacing older ones
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1)

        # Subscribers
        self.create_subscription(Float64, 
                                 self.steering_angle_topic, 
                                 self._steering_callback, qos_profile)
        
        self.create_subscription(Float64, 
                                 self.velocity_topic, 
                                 self._velocity_callback, qos_profile)
        
        self.create_subscription(Image, 
                                 self.image_topic, 
                                 self._image_callback, qos_profile)

        # Timer for periodic global loop execution
        self.timer = self.create_timer(self.update_period, self._update)

    def _has_invalid_state(self) -> bool:
        """
        @brief Checks if the current state is invalid.

        Evaluates timeout, missing data, or low-speed conditions to determine
        if the current vehicle state is invalid for recording.

        @retval True If any invalid condition is met (timeout, missing data, or low speed).
        @retval False If the current state is valid.
        """
        now = self.get_clock().now()

        # Missing data check (return immediately)
        required_fields = (
            self.last_state.image,
            self.last_state.image_time,
            self.last_state.steering_angle,
            self.last_state.steering_angle_time,
            self.last_state.velocity,
            self.last_state.velocity_time,
            self.max_velocity,
            self.max_steering_angle,
        )

        if any(value is None for value in required_fields):
            return True

        # Timeout check (safe because timestamps are guaranteed valid if we reached here)
        timestamps = [
            self.last_state.image_time,
            self.last_state.steering_angle_time,
            self.last_state.velocity_time,
        ]

        if (now - min(timestamps)).nanoseconds > self.timeout_time:
            return True

        # Low-speed check
        if (self.max_velocity is not None and
            self.last_state.velocity <
            self.min_velocity_factor * self.max_velocity):
            return True

        # All good
        return False
        
    def _steering_callback(self, msg: Float64):
        """
        @brief Callback for receiving steering angle data.

        @param msg Float64 message containing the current steering angle.
        """
        self.last_state.steering_angle = msg.data
        self.last_state.steering_angle_time = self.get_clock().now()

    def _velocity_callback(self, msg: Float64):
        """
        @brief Callback for receiving vehicle velocity data.

        @param msg Float64 message containing the current vehicle velocity.
        """
        self.last_state.velocity = msg.data
        self.last_state.velocity_time = self.get_clock().now()

    def _image_callback(self, msg: Image):
        """
        @brief Callback for receiving camera images.

        @param msg Image message from the camera topic.
        """
        self.last_state.image = msg
        self.last_state.image_time = self.get_clock().now()

    def _update(self):
        """
        @brief Periodic update loop executed by the node's timer.

        Checks the validity of the current vehicle state and, if valid,
        saves a synchronized sample (image + state data) to the dataset.
        """
        if self._has_invalid_state():
            return

        self.storage.save_sample(self.last_state, self.max_velocity, self.max_steering_angle) 

        self.get_logger().info(f"Velocity: {self.last_state.velocity}, " 
                               f"Steering Angle: {self.last_state.steering_angle}")


def main(args=None):
    rclpy.init(args=args)

    # Instantiate the node with the given vehicle node name
    node = DataCollector()

    try:
        # Spin the node to process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

When the **DataCollector** was added to the system, it became necessary to define a parameters file to centralize all configuration values required by the node. These parameters were declared within the node and loaded from an external file, allowing the node’s behavior to be adjusted without modifying the source code. As a result, configuration details are separated from the implementation logic, making the system easier to maintain and adapt. To support this approach, a **config** directory was created inside the package to store configuration files. The command executed below was used to create this directory:

```bash
mkdir -p ~/workspace/src/autopilot_neural_network/config
```

The parameter values were defined in the **parameters.yaml** file located in the **autopilot_neural_network/config** directory, and its content was defined as follows:


```yaml
/**:
  ros__parameters:
    # Vehicle node and topics
    vehicle_node: "vehicle_controller" # Name of the node from which to retrieve vehicle parameters
    velocity_topic: "/velocity" # Topic for publishing/subscribing to velocity commands
    steering_angle_topic: "/steering_angle" # Topic for publishing/subscribing to steering commands
    image_topic: "/camera/image_raw" # Topic for subscribing to raw camera images
    max_velocity_parameter: "max_velocity" # Vehicle parameter defining max velocity
    max_steering_angle_parameter: "max_steering_angle" # Vehicle parameter defining max steering

    # Data Collector Node Parameters
    min_velocity_factor: 0.25 # Fraction of max velocity below which data is not recorded
    timeout_time: 1000000000 # Timeout for data freshness in nanoseconds [ns]
    update_period: 0.5 # Period for the node's update loop in seconds [s]

    # File Paths
    dataset_path: "/tmp/autopilot_neural_network/dataset" # Path where dataset is saved and loaded
```

The parameters defined in this configuration file must be adjusted according to the intended data collection setup and storage location. In particular, the **dataset_path** parameter specifies where the recorded data are saved and later accessed during training, and it can be modified to match the desired directory structure. For example, during this project the dataset path was changed to **/home/user/autopilot_neural_network/dataset**, ensuring that all collected data was stored in a persistent location suitable for training.

To tie everything together, a launch file was created. This launch file was responsible for starting the **data_collector** node and passing the **parameters.yaml** file at runtime, so all required configuration values are available as soon as the node starts. To organize the system startup files, a dedicated launch directory was created inside the package to store launch scripts. The command executed below was used to create this directory:

```bash
mkdir -p ~/workspace/src/autopilot_neural_network/launch
```

The launch file was placed in the **autopilot_neural_network/launch** directory in a file named **data_collector.launch.py**, and its implementation is shown next:

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launches the data_collector node with parameters loaded from a YAML file.
    """

    package_name = 'autopilot_neural_network'
    package_share = get_package_share_directory(package_name)

    # Path to the YAML configuration file
    params_file = os.path.join(package_share, 'config', 'parameters.yaml')

    # Define the node to launch
    data_collector_node = Node(package=package_name,
                               executable='data_collector',
                               name='data_collector',
                               output='screen',
                               parameters=[params_file])

    return LaunchDescription([data_collector_node])
```

Finally, it was necessary to update the **autopilot_neural_network/setup.py** file to properly integrate the new node into the package. This update ensures that the node can be executed as a console script, while also installing the launch and configuration files so they are available at runtime through the package share directory. To make the launch files and parameter definitions accessible through the ROS 2 package, the **autopilot_neural_network/launch** and **autopilot_neural_network/config** directories were added to the package data files, ensuring they are properly installed alongside the package. The updated **setup.py** file is shown below.


```python
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'autopilot_neural_network'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # includes scripts/ automatically
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Mazzetto',
    maintainer_email='workabotic@gmail.com',
    description='An end-to-end deep learning pipeline for autonomous driving in ROS 2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'data_collector = autopilot_neural_network.data_collector:main',
        ],
    },
)
```

From this point on, the **autopilot_neural_network** directory had the following structure:

```bash
autopilot_neural_network/
├── autopilot_neural_network/
│   ├── __init__.py
│   ├── data_collector.py
│   └── parameters_client_node.py
├── config/
│   └── parameters.yaml
├── launch/
│   └── data_collector.launch.py
├── resource/
│   └── autopilot_neural_network
├── package.xml
├── setup.cfg
└── setup.py
```

After the package structure was defined for the dataset collection, the workspace directory was accessed and the package was built using colcon, as shown by the commands below.

```bash
source /opt/ros/jazzy/setup.bash

source ~/.venv/bin/activate

cd ~/workspace

colcon build
```

Next, the vehicle was manually driven on the dirt, sand, and snow tracks for many hours, ensuring that a sufficiently large and diverse dataset was gathered to train the model. The grass track was intentionally excluded from the training process and reserved for testing and evaluating the model’s ability to generalize to unseen environments.

To launch the setup for collecting training data, the workspace environment was first accessed in a terminal. The ROS 2 environment was then initialized by sourcing both the global ROS installation and the local workspace setup files. After the environment was properly configured, the simulation was started by launching the **vehicle.launch.py** file from the **gazebo_ackermann_steering_vehicle** package, while passing the desired world file as a parameter. The world path was obtained using **$(ros2 pkg prefix gazebo_racing_tracks)**, ensuring that the track description was used, as shown by the commands below:

```bash
source /opt/ros/jazzy/setup.bash 

source ~/.venv/bin/activate

cd ~/workspace 

source install/setup.bash 

ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py \
  world:=$(ros2 pkg prefix gazebo_racing_tracks)/share/gazebo_racing_tracks/worlds/dirt_track.sdf
```

In parallel, an Xbox One game controller was connected to the computer to enable manual drive of the vehicle. In a separate terminal, the workspace was accessed again and both the global ROS 2 and local workspace environments were sourced. With the environment properly set up, the **joystick.launch.py** from the **gazebo_ackermann_steering_vehicle** package was launched to establish the connection between the joystick and the simulation, as demonstrated by the commands that follow:


```bash
source /opt/ros/jazzy/setup.bash 

source ~/.venv/bin/activate

cd ~/workspace 

source install/setup.bash 

ros2 launch gazebo_ackermann_steering_vehicle joystick.launch.py
```

In a third terminal, the workspace was accessed once more and both the global ROS 2 installation and the local workspace environment were sourced again. With the environment fully initialized, the **data_collector.launch.py** file from the **autopilot_neural_network** package was launched, completing the system setup and enabling the collection of vehicle data for gathering the dataset, as detailed in the commands presented below:

```bash
source /opt/ros/jazzy/setup.bash 

source ~/.venv/bin/activate

cd ~/workspace 

source install/setup.bash 

ros2 launch autopilot_neural_network data_collector.launch.py
```

Finally, **RViz** was opened and an **Image** display was added, subscribing to the **/camera/image_raw** topic to visualize the view from the vehicle’s front camera. This visualization played an important role during dataset collection, as it was used to keep the vehicle aligned within the track boundaries. Special care was taken to drive as accurately as possible, avoiding situations where the vehicle would leave the track.

![Gathering data while driving the vehicle]({{ site.url }}{{ site.baseurl }}/public/images/autonomous-driving-vehicle-simulation/rviz-visualization-while-driving-the-vehicle.gif)
*Figure 2 — RViz visualization while driving the vehicle.*

As mentioned before, the same procedure was then repeated for the other tracks. To switch tracks, it was only necessary to update the **vehicle.launch.py** command by replacing **dirt_track.sdf** with either **snow_track.sdf** or **sand_track.sdf**. During this process, more than 5,000 frames were collected, along with their associated data.

### Neural Network Training 

After the data were collected, the design of the neural network was initiated. All training was performed offline, with the complete dataset available from the beginning. To accomplish this, a model architecture was designed to receive an image as input and to output both the steering angle and the desired velocity. In addition, a custom data loader was implemented to load the dataset and apply data augmentation techniques, increasing data variability and improving generalization. A dedicated loss function was defined, and a training script was developed, with the hyperparameters empirically tuned to achieve satisfactory performance.

A new directory named **scripts/** was created inside the **autopilot_neural_network** package to separate the training scripts from the ROS 2 nodes and to keep the project organized. All training scripts, along with other utility scripts required, were placed in this directory. To create this directory, the following command was used:

```bash
mkdir -p ~/workspace/src/autopilot_neural_network/scripts
```

It was also necessary to create an empty **__init__.py** file inside the scripts directory. This was required because some of the code located in this directory was used by the ROS 2 package, and the presence of this file ensured that the directory was recognized as a Python module. To accomplish this, the next command was executed:

```bash
touch ~/workspace/src/autopilot_neural_network/scripts/__init__.py
```

#### Model Architecture

The proposed model architecture was inspired by the [LeNet](http://vision.stanford.edu/cs598_spring07/papers/Lecun98.pdf), a very simple and lightweight Convolutional Neural Network. The network was designed around a shared convolutional backbone responsible for extracting visual features from the input image, followed by a split into two independent regression heads.

The convolutional backbone applied successive convolution, pooling, and ReLU operations to extract and compress visual information, while dropout was used to improve generalization. The resulting feature maps were flattened and processed by shared fully connected layers to form a compact representation, which was then fed into two regression heads for steering angle and velocity estimation. The implementation of this model was placed in the **autopilot_neural_network/scripts/model.py** file, with the corresponding code shown below:


```python
import torch
import torch.nn as nn


class AutopilotNet(nn.Module):
    """
    This model takes a single-channel image (e.g., grayscale) as input and
    outputs two values: the predicted velocity and steering angle. It uses a
    multi-head architecture where a shared convolutional base extracts features
    from the input image, and two separate fully connected heads predict the
    final outputs.
    """
    def __init__(self, h=96, w=128, inputs=1):
        """
        @brief Initializes the AutopilotNet model layers.

        @param h The height of the input images.
        @param w The width of the input images.
        @param inputs The number of channels in the input images.
        """
        super(AutopilotNet, self).__init__()

        # Convolutional base for feature extraction
        self.convolutional_layers = nn.Sequential(nn.Conv2d(inputs, 6, kernel_size=5, stride=1),
                                                  nn.ReLU(),
                                                  nn.MaxPool2d(kernel_size=2, stride=2),
                                                  nn.Conv2d(6, 16, kernel_size=5, stride=1),
                                                  nn.ReLU(),
                                                  nn.MaxPool2d(kernel_size=2, stride=2),
                                                  nn.Dropout(0.5))

        # Get the number of features produced by the convolutional block
        conv_output_size = self._get_conv_output_size(h, w, inputs)

        # Shared fully connected layers to process features from the convolutional base
        self.shared_layers = nn.Sequential(nn.Linear(conv_output_size, 120),
                                           nn.ReLU())

        # Head for predicting velocity
        self.velocity_head = nn.Sequential(nn.Linear(120, 84),
                                           nn.ReLU(),
                                           nn.Linear(84, 1))
        
        # Head for predicting steering angle
        self.steering_head = nn.Sequential(nn.Linear(120, 84),
                                           nn.ReLU(),
                                           nn.Linear(84, 1))

    def _get_conv_output_size(self, h, w, inputs):
        """
        @brief Computes the number of output features produced by the convolutional layers.

        This method generates a dummy tensor with the same shape as the actual
        input images and feeds it through the convolutional base. By examining
        the resulting tensor size, it determines the exact number of features
        required by the first fully connected layer.

        @param h The input image height.
        @param w The input image width.
        @param inputs The number of input channels.
        @return The flattened feature size after all convolutional layers.
        """
        # Create a dummy tensor
        x = torch.zeros(1, inputs, h, w)
        
        # Pass dummy input through the convolutional layers
        x = self.convolutional_layers(x)
        
        # Get the total number of elements in x
        return x.numel()
    
    def forward(self, x):
        """
        @brief Defines the forward pass of the AutopilotNet.

        @param x The input tensor, which is a batch of images.
        @return A tensor containing the concatenated predictions for velocity and steering angle.
        """
        # Pass input through the convolutional layers
        x = self.convolutional_layers(x)
        
        # Flatten the output for the fully connected layers
        x = x.view(x.size(0), -1)
        
        # Pass flattened features through the shared layers
        x = self.shared_layers(x)

        # Get predictions from each head
        vel = self.velocity_head(x)
        steer = self.steering_head(x)

        # Concatenate the outputs into a single tensor
        return torch.cat([vel, steer], dim=1)
```

The model was opened in [Netron](https://github.com/lutzroeder/netron) to visualize its computational graph. This tool provides an interface for inspecting neural network architectures by displaying their structure. The image below shows the graph representation of the proposed model as rendered by the tool:

![Neural network architecture]({{ site.url }}{{ site.baseurl }}/public/images/autonomous-driving-vehicle-simulation/neural-network-architecture.webp)
*Figure 3 — Neural network architecture.*

#### Data Loader

The data loading pipeline was built using a custom PyTorch **Dataset** that read image paths and driving commands from a CSV file and returned preprocessed samples for training and validation. For each sample, the image was loaded from disk, while the velocity and steering values were extracted and normalized using the maximum values provided in the CSV.

The images were processed through a fixed preprocessing step that converted them to grayscale, resized them to a fixed resolution, transformed them into tensors, and applied normalization. During training, data augmentation was used to improve generalization. This included changes in brightness and contrast, along with small geometric transformations such as rotations, perspective distortion, Gaussian blur, and random horizontal flipping, with the steering angle inverted to keep the labels consistent.

The **preprocessing_transform** function was defined outside the **Data** class because it was also used during the inference stage. In this case, input data arrived in a ROS 2 format and needed to be preprocessed before being passed through the model. By keeping this transformation separate, the same resizing, grayscale conversion, tensor conversion, and normalization steps used during training could be applied during inference, ensuring that the model received inputs in the same format for both training and inference.

All of this code was implemented in the **autopilot_neural_network/scripts/dataset.py** file, with the corresponding implementation shown next:

```python
import os
import torch
import csv
import numpy as np
import torchvision.transforms as T
from torch.utils.data import Dataset
from PIL import Image


def preprocessing_transform(height, width):
    """
    @brief Creates the preprocessing transformation pipeline.
    
    @param height Desired image height after resizing.
    @param width Desired image width after resizing.

    @return A torchvision Compose object with the preprocessing transforms.
    """
    return T.Compose([T.Grayscale(),
                      T.Resize((height, width)),
                      T.ToTensor(),
                      T.Normalize((0.5,), (0.5,))])


class Data(Dataset):
    """
    @brief Custom PyTorch dataset for autonomous driving images and labels.

    This dataset loads image paths and target values from a CSV file,
    applies optional data augmentation, normalizes steering and velocity using their 
    max values from the CSV, and returns (image_tensor, normalized_velocity, normalized_steering).

    @param dataset_path Path to the folder containing images and dataset.csv.
    @param height Preprocessing image height.
    @param width Preprocessing image width.
    @param augment Whether to apply online augmentation during training.
    """

    def __init__(self, dataset_path, height=96, width=128, augment=True):
        # Path to the dataset CSV file
        self.dataset_path = dataset_path
        self.csv_file_path = os.path.join(self.dataset_path, 'dataset.csv')
        
        self.augment = augment

        # Image augmentation pipeline (used only if augment=True)
        self.augmentation = T.Compose([
            T.ColorJitter(brightness=(0.5, 1.5),
                          contrast=(0.5, 1.5),
                          saturation=(0.5, 1.5)),
            
            T.RandomPerspective(distortion_scale=0.1, p=0.1),
            T.RandomRotation(degrees=(-3, 3)),
            T.GaussianBlur(kernel_size=5, sigma=(0.1, 0.8)),
        ])

        self.transform = preprocessing_transform(height, width)

        # Storage for loaded sample metadata
        self.samples = []
        self.load_samples() # Parse CSV and fill samples list


    def load_samples(self):
        """
        @brief Loads sample metadata from dataset.csv and populates self.samples.
        """
        if not os.path.exists(self.csv_file_path):
            raise FileNotFoundError(f"CSV file not found: {self.csv_file_path}")

        # Open and parse the CSV file
        with open(self.csv_file_path, 'r') as f:
            reader = csv.DictReader(f)

            for i, row in enumerate(reader, start=2):  # Start at line 2 (after header)

                # Skip rows that are empty or malformed
                if not any(row.values()):
                    continue

                # Build the full path to the corresponding image
                image_path = os.path.join(self.dataset_path, f"{row['image_id']}.png")

                # Append structured sample info
                self.samples.append({
                    'image_path': image_path,
                    'velocity': float(row['velocity']),
                    'steering_angle': float(row['steering_angle']),
                    'max_velocity': float(row['max_velocity']),
                    'max_steering_angle': float(row['max_steering_angle'])
                })


    def random_flip(self, image, steering, p=0.5):
        """
        @brief Horizontally flips the image and inverts steering with probability p.

        @param image Torch tensor image.
        @param steering Steering angle value.
        @param p Probability of flipping.

        @return (image, steering) Possibly flipped image and modified steering.
        """
        if np.random.rand() < p:       
            image = torch.flip(image, (2,))  # Flip along width dimension
            steering = -steering # Invert steering sign
            
        return image, steering


    def __len__(self):
        return len(self.samples)


    def __getitem__(self, idx):
        # Retrieve metadata for this index
        sample = self.samples[idx]

        # Load image from disk (RGB mode)
        image = Image.open(sample['image_path']).convert('RGB')

        # Raw velocity and steering values
        velocity = sample['velocity']
        steering = sample['steering_angle']

        # Apply augmentation if enabled (training only)
        if self.augment:
            image = self.augmentation(image)

        # Apply preprocessing (resize, grayscale, tensor, normalize)
        image = self.transform(image)

        # Apply random horizontal flip augment
        if self.augment:
            image, steering = self.random_flip(image, steering)

        # Normalize values using dataset-provided max values
        max_velocity = sample['max_velocity']
        max_steering_angle = sample['max_steering_angle']

        velocity /= max_velocity if max_velocity != 0 else 1
        steering /= max_steering_angle if max_steering_angle != 0 else 1

        # Convert to tensors
        velocity = torch.tensor(velocity, dtype=torch.float32)
        steering = torch.tensor(steering, dtype=torch.float32)

        return image, velocity, steering
```

#### Loss Function

The loss function was designed to handle an imbalance between the learning difficulty of the two predicted outputs. During training, it was observed that the model learned to predict the vehicle’s velocity much more easily than the steering angle. As a result, the optimization process tended to focus on minimizing the velocity error, while the steering prediction did not improve as much. 

To address this issue, a weighted loss function was introduced based on the mean squared error. The velocity and steering losses were computed separately and then combined using weighting factors. By tuning these factors, it was possible to control the importance of each component, encouraging the model to pay more attention to steering prediction while still maintaining accurate velocity estimation. Therefore, mathematically, the loss function was implemented as a weighted combination of the velocity and steering angle errors as follows:

$$
\mathcal{L} = \alpha \cdot \frac{1}{N}\sum_{i=1}^{N} \left( v_i - \hat{v}_i \right)^2
\;+\;
\beta \cdot \frac{1}{N}\sum_{i=1}^{N} \left( \theta_i - \hat{\theta}_i \right)^2
$$

where $$\hat{v}_i$$ and $$\hat{\theta}_i$$ are the predicted velocity and steering angle, $$v_i$$ and $$\theta_i$$ are the corresponding ground truth values, $$\alpha$$ and $$\beta$$ are weighting factors that control the contribution of each term, and $$N$$ is the number of samples in the batch.

#### Training Loop

An imbalance in the steering angle distribution was observed in the training dataset, primarily because a large portion of the collected data corresponded to the vehicle driving straight ahead. As a result, most samples had steering angles close to zero, which biased the learning process. To address this issue, the **SteeringBalancedSampler** was developed.

This sampler addresses the steering-angle imbalance by first normalizing the steering values using their maximum magnitudes and then separating samples into low steering and high steering groups based on a threshold defined as a fraction of the maximum steering angle. All high steering samples are retained to preserve turning maneuvers, while low steering samples are downsampled to a fixed fraction of the dataset. The resulting set of indices is used during training to produce a more balanced sampling of steering behaviors, encouraging the model to learn steering control more effectively without completely discarding straight driving data. 

The code below shows the implementation of the sampler used during training, placed in the **autopilot_neural_network/scripts/sampler.py** file:


```python
from torch.utils.data import Sampler
import numpy as np


class SteeringBalancedSampler(Sampler):
    """
    This sampler is designed for steering-angle datasets where the majority of samples
    correspond to near-zero steering values (vehicle driving straight).
    """

    def __init__(self, dataset, low_fraction=0.10, threshold_ratio=0.10, shuffle=True):
        """
        @brief Constructor for SteeringBalancedSampler.

        The constructor analyzes steering angles, identifies low-steering 
        and high-steering samples, and keeps: all high-steering samples and at most 
        low_fraction * total dataset size of low-steering samples

        @param dataset The dataset to sample from.
        @param low_fraction The maximum fraction of low-steering samples to keep.
        @param threshold_ratio Fraction of max steering defining "low steering".
        @param shuffle Whether to shuffle the indices after balancing.
        """

        self.low_fraction = low_fraction
        self.shuffle = shuffle
        
        # Extract raw samples from the original dataset
        samples = [dataset.dataset.samples[i] for i in dataset.indices]

        # Collect steering data
        steer = np.array([s["steering_angle"] for s in samples])
        max_steer = np.array([s["max_steering_angle"] for s in samples])

        # Normalize steering angles to [-1, 1]
        norm_steer = steer / max_steer
        abs_norm = np.abs(norm_steer)

        # Identify low and high steering samples using threshold as fraction
        low_indices = np.where(abs_norm < threshold_ratio)[0]
        high_indices = np.where(abs_norm >= threshold_ratio)[0]

        # Keep at most (low_fraction * total_dataset) low-steering samples
        max_low_allowed = int(len(samples) * low_fraction)

        # How many low samples we will actually keep
        keep_low = min(len(low_indices), max_low_allowed)

        # Downsample low-steering indices
        if keep_low > 0:
            low_indices = np.random.choice(low_indices, size=keep_low, replace=False)
        else:
            low_indices = np.array([], dtype=np.int64)

        # Combine all high-steering samples with selected low-steering samples
        self.indices = np.concatenate([high_indices, low_indices])

        # Shuffle final indices if required
        if shuffle:
            np.random.shuffle(self.indices)

    def __iter__(self):
        return iter(self.indices)

    def __len__(self):
        return len(self.indices)
```

Following the introduction of the balanced sampling strategy, the complete training procedure was implemented in a dedicated Python script that integrates all previously described components into a unified workflow. Built on top of PyTorch, the script combines the neural network architecture, custom dataset and preprocessing pipeline, weighted loss function, and the sampler to form the core of the learning process. Standard PyTorch utilities are used for data loading, optimization, and learning rate scheduling, while additional tools such as **tqdm** and **TensorBoard** supported progress visualization and experiment monitoring.

The script initialized the model and constructed the training and validation datasets using the custom **Data** class, which were then split according to a configurable validation fraction. During training, the balanced sampler was applied exclusively to the training subset, while the validation data were processed without augmentation or sampling bias. Optimization was performed using the Adam optimizer together with the weighted mean squared error loss previously described, and a learning rate scheduler dynamically adjusted the learning rate based on validation loss. At each epoch, the model was trained and evaluated, performance metrics were logged, and the best-performing model was saved automatically.

The complete implementation of the training script, bringing together all components of the learning process into a single file, is presented below and was placed in the **autopilot_neural_network/scripts/train.py** file.

```python
import os
import time
import argparse
import multiprocessing

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split
from torch.optim.lr_scheduler import ReduceLROnPlateau
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm

from model import AutopilotNet
from dataset import Data
from sampler import SteeringBalancedSampler


def setup_tensorboard(args):
    """
    @brief Initializes and configures a TensorBoard SummaryWriter.

    This function sets up a directory for TensorBoard logs and records the
    hyperparameters used for the training session.

    @param args Arguments containing hyperparameters.
    @return The configured TensorBoard writer object.
    """
    log_dir = os.path.join("runs", f"autopilot_neural_network_{int(time.time())}")
    writer = SummaryWriter(log_dir=log_dir)
    
    writer.add_text("Hyperparameters", 
                    f"learning_rate: {args.learning_rate}, "
                    f"epochs: {args.epochs}, " 
                    f"batch_size: {args.batch_size}")
    
    return writer


def save_model(model, optimizer, scheduler, epoch, path):
    """
    @brief Saves the model checkpoint.

    Saves the state of the model, optimizer, and scheduler to a file. 
    This function is used to save the best performing model based on validation loss.

    @param model The model to save.
    @param optimizer The optimizer state to save.
    @param scheduler The scheduler state to save.
    @param epoch The current epoch number.
    @param path The file path to save the checkpoint to.
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    
    state = {'epoch': epoch,
             'state_dict': model.state_dict(),
             'optimizer': optimizer.state_dict(),
             'scheduler': scheduler.state_dict()}
    
    torch.save(state, path)
    print(f"Model saved at epoch {epoch} to {path}")


def compute_weighted_loss(predicted_velocities, predicted_steering_angles, 
                          velocities, steering_angles, criterion, alpha=1, beta=1):
    """
    @brief Computes a weighted loss for velocity and steering angle.

    This function calculates the error for both predicted velocities
    and steering angles against their ground truth values, and then combines them
    into a single weighted loss.
    
    @param predicted_velocities The predicted velocities from the model.
    @param predicted_steering_angles The predicted steering angles from the model.
    @param velocities The ground truth velocities.
    @param steering_angles The ground truth steering angles.
    @param criterion The loss function.
    @param alpha The weight for the velocity loss component.
    @param beta The weight for the steering angle loss component.
    
    @return The final computed weighted loss.
    """
    # Compute losses separately
    velocity_loss = criterion(predicted_velocities, velocities)
    steering_loss = criterion(predicted_steering_angles, steering_angles)
    
    # Weighted total loss
    loss = (alpha * velocity_loss) + (beta * steering_loss)
    
    return loss


def validate(model, val_loader, criterion, device, alpha, beta):
    """
    @brief Evaluates the model on the validation dataset.

    This function iterates through the validation set, computes the loss, and
    calculates the mean absolute error for both velocity and steering angle predictions.

    @param model The model to be evaluated.
    @param val_loader DataLoader for the validation data.
    @param criterion The loss function used for evaluation.
    @param device The device (CPU or CUDA) to run evaluation on.
    @param alpha The weight for the velocity loss component.
    @param beta The weight for the steering angle loss component.
    
    @return A tuple containing the mean validation loss, mean velocity error,
            and mean steering angle error.
    """
    # Set the model to evaluation mode
    model.eval()
    
    # Initialize accumulators for loss and error metrics
    val_loss = 0
    velocity_error = 0
    steering_angle_error = 0
    
    # Disable gradient calculations to save memory and computations
    with torch.no_grad():
        for i, data in enumerate(val_loader):
            # Unpack data from the validation loader
            images, velocities, steering_angles = data
            
            # Reshape and cast labels to match the model's output format
            velocities = velocities.float().view(len(velocities), 1)
            steering_angles = steering_angles.float().view(len(steering_angles), 1)

            # Move data to the appropriate device (GPU or CPU)
            if device.type == "cuda":
                images = images.cuda()
                velocities = velocities.cuda()
                steering_angles = steering_angles.cuda()

            # Get model predictions
            outputs = model(images)
            
            # Extract predictions
            predicted_velocities = outputs[:, 0:1]
            predicted_steering_angles = outputs[:, 1:2]
            
            # Compute the weighted loss for the current batch
            loss = compute_weighted_loss(predicted_velocities=predicted_velocities,
                                         predicted_steering_angles=predicted_steering_angles,
                                         velocities=velocities,
                                         steering_angles=steering_angles,
                                         criterion=criterion,
                                         alpha=alpha,
                                         beta=beta)
            
            # Accumulate the validation loss
            val_loss += loss.item()
            
            # Accumulate the mean absolute error for velocity and steering angle
            velocity_error += torch.mean(
                torch.abs(velocities - predicted_velocities)).item()
            
            steering_angle_error += torch.mean(
                torch.abs(steering_angles - predicted_steering_angles)).item()
    
    # Calculate the mean validation loss and errors across all batches
    mean_val_loss = val_loss / (i + 1)
    mean_velocity_error = velocity_error / (i + 1)
    mean_steering_angle_error = steering_angle_error / (i + 1)
    
    return mean_val_loss, mean_velocity_error, mean_steering_angle_error


def train(args, model, train_dataset, val_dataset):
    """
    @brief Main training and validation loop.

    This function orchestrates the entire training process. It sets up the optimizer,
    learning rate scheduler, data loaders (including a balanced sampler for training),
    and the main training loop. For each epoch, it trains the model, evaluates it on
    the validation set, logs metrics to TensorBoard, and saves the best model
    checkpoint based on validation loss.

    @param args Command-line arguments for training configuration.
    @param model The neural network model to train.
    @param train_dataset The dataset for training.
    @param val_dataset The dataset for validation.
    """
    # Determine the device to use for training (GPU if available, otherwise CPU)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    # Move the model to the selected device
    model.to(device)

    # Initialize the Adam optimizer with the model's parameters and learning rate
    optimizer = optim.Adam(model.parameters(), lr=args.learning_rate)
    
    # Initialize a learning rate scheduler that reduces the learning rate when validation loss plateaus
    scheduler = ReduceLROnPlateau(optimizer, mode='min', 
                                  factor=args.lr_factor, 
                                  patience=args.lr_patience)
    
    # Define the loss function
    criterion = nn.MSELoss()
    
    # Set up a balanced sampler for the training data to handle imbalanced steering angles
    train_sampler = SteeringBalancedSampler(dataset=train_dataset,
                                            low_fraction=args.sampler_low_fraction,
                                            threshold_ratio=args.sampler_threshold_ratio,
                                            shuffle=True)
    
    print(f"Using {len(train_sampler)} training samples after balanced sampling.")
    
    # Create a DataLoader for the training set, using the balanced sampler
    train_loader = DataLoader(train_dataset, 
                              batch_size=args.batch_size, 
                              sampler=train_sampler, 
                              pin_memory=True, # Speeds up data transfer to GPU
                              num_workers=args.num_workers)
    
    # Create a DataLoader for the validation set
    val_loader = DataLoader(val_dataset, 
                            batch_size=args.val_batch_size, 
                            shuffle=False, # No need to shuffle validation data
                            pin_memory=True, 
                            num_workers=args.num_workers)

    # Set up TensorBoard for logging
    writer = setup_tensorboard(args)
    
    # Initialize the best validation loss to infinity for tracking the best model
    best_val_loss = float('inf')
    best_model_path = args.model_path
    
    # Start the training loop for the specified number of epochs
    for epoch in range(args.epochs):
        # Set the model to training mode
        model.train()
        
        # Initialize accumulators for training metrics for the current epoch
        train_loss = 0
        velocity_error = 0
        steering_angle_error = 0
    
        # Iterate over the training data in batches
        for i, data in enumerate(tqdm(train_loader, desc=f"Epoch {epoch}")):
            # Unpack data from the training loader
            images, velocities, steering_angles = data
            
            # Reshape and cast labels to match the model's output format
            velocities = velocities.float().view(len(velocities), 1)
            steering_angles = steering_angles.float().view(len(steering_angles), 1)

            # Move data to the appropriate device (GPU or CPU)
            if device.type == "cuda":
                images = images.cuda()
                velocities = velocities.cuda()
                steering_angles = steering_angles.cuda()
            
            # Get model predictions
            outputs = model(images)
            
            # Extract predictions
            predicted_velocities = outputs[:, 0:1]
            predicted_steering_angles = outputs[:, 1:2]
            
            # Compute the weighted loss for the current batch
            loss = compute_weighted_loss(predicted_velocities=predicted_velocities,
                                         predicted_steering_angles=predicted_steering_angles,
                                         velocities=velocities,
                                         steering_angles=steering_angles,
                                         criterion=criterion,
                                         alpha=args.alpha,
                                         beta=args.beta)
            
            # Backpropagate the loss and update model weights
            loss.backward()
            optimizer.step()
            optimizer.zero_grad()

            # Accumulate the training loss and error metrics
            train_loss += loss.item()
            velocity_error += torch.mean(
                torch.abs(velocities - predicted_velocities)).item()
            
            steering_angle_error += torch.mean(
                torch.abs(steering_angles - predicted_steering_angles)).item()
            
        # Calculate the mean training loss and errors for the epoch
        mean_train_loss = train_loss / (i + 1)
        mean_velocity_error = velocity_error / (i + 1)
        mean_steering_angle_error = steering_angle_error / (i + 1)
        
        # Evaluate the model on the validation set
        mean_val_loss, mean_val_velocity_error, mean_val_steering_angle_error = \
            validate(model, val_loader, criterion, device, args.alpha, args.beta)

        print(f"Epoch {epoch} | Train Loss: {mean_train_loss:.4f} | Val Loss: {mean_val_loss:.4f}")

        # Log metrics to TensorBoard
        writer.add_scalar("Loss/Train", mean_train_loss, epoch)
        writer.add_scalar("Loss/Validation", mean_val_loss, epoch)
        writer.add_scalar("Velocity Error/Train", mean_velocity_error, epoch)
        writer.add_scalar("Steering Angle Error/Train", mean_steering_angle_error, epoch)
        writer.add_scalar("Velocity Error/Validation", mean_val_velocity_error, epoch)
        writer.add_scalar("Steering Angle Error/Validation", mean_val_steering_angle_error, epoch)
        writer.add_scalar("Learning Rate", scheduler.get_last_lr()[0], epoch)

        # Update the learning rate based on the validation loss
        scheduler.step(mean_val_loss)

        # Save the model if the validation loss has improved
        if mean_val_loss < best_val_loss:
            best_val_loss = mean_val_loss
            save_model(model, optimizer, scheduler, epoch, best_model_path)

    writer.close()


def main(args):
    """
    @brief Entry point for the training script.

    Parses command-line arguments, initializes the model, creates the datasets,
    splits them into training and validation sets, and then starts the training
    process by calling the train function.

    @param args The parsed command-line arguments.
    """
    # Initialize model
    model = AutopilotNet(h=args.height, w=args.width, inputs=1)  
    
    # Training dataset with augmentation
    train_dataset = Data(args.dataset_path, 
                         width=args.width, 
                         height=args.height, 
                         augment=True)  
    
    # Validation dataset without augmentation
    val_dataset = Data(args.dataset_path,
                       width=args.width, 
                       height=args.height, 
                       augment=False)
    
    # Split the dataset between training and validation
    val_fraction = args.val_fraction # Validation percentage
    val_size = int(len(train_dataset) * val_fraction)  # Number of validation samples
    train_size = len(train_dataset) - val_size  # Number of training samples

    # Generate random train/val indices
    train_indices, val_indices = random_split(range(len(train_dataset)),
                                              [train_size, val_size])
    
    # Apply the generated train and validation indices to create subset datasets
    train_dataset = torch.utils.data.Subset(train_dataset, train_indices)  
    val_dataset = torch.utils.data.Subset(val_dataset, val_indices)

    # Start training
    train(args, model, train_dataset, val_dataset)  


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    
    parser.add_argument('--dataset_path', type=str, required=True,
                        help="Path to the directory containing the dataset.")
    
    parser.add_argument('--model_path', type=str, required=True,
                        help="Path to save the best model checkpoint.")
    
    parser.add_argument('--epochs', type=int, default=100,
                        help="Number of training epochs.")
    
    parser.add_argument('--batch_size', type=int, default=1024,
                        help="Batch size for the training data loader.")
    
    parser.add_argument('--val_batch_size', type=int, default=256,
                        help="Batch size for the validation data loader.")
    
    parser.add_argument('--val_fraction', type=float, default=0.2,
                        help="Fraction of the dataset to use for validation (e.g., 0.2 for 20%).")

    parser.add_argument('--learning_rate', type=float, default=1.0e-3)
    
    parser.add_argument('--lr_patience', type=int, default=2, 
                        help="Epochs to wait for loss improvement before reducing learning rate.")
    
    parser.add_argument('--lr_factor', type=float, default=0.5,
                        help="Factor by which to reduce learning rate on plateau.")
    
    parser.add_argument('--alpha', type=float, default=0.1,
                        help="Weight for the velocity component of the loss function.")
    
    parser.add_argument('--beta', type=float, default=1.0,
                        help="Weight for the steering angle component of the loss function.")
    
    parser.add_argument('--num_workers', type=int, default=(multiprocessing.cpu_count() - 1),
                        help="Number of worker processes for data loading.")
    
    parser.add_argument('--height', type=int, default=96,
                        help="Height to which input images will be resized.")
    
    parser.add_argument('--width', type=int, default=128,
                        help="Width to which input images will be resized.")

    parser.add_argument('--sampler_low_fraction', type=float, default=0.05, 
                        help="Fraction of low-steering samples to keep.")
    
    parser.add_argument('--sampler_threshold_ratio', type=float, default=0.10,
                        help="Steering fraction of max steering to define low steering region.")

    args = parser.parse_args()

    main(args)
```

In summary, the training procedure was configured using a set of hyperparameters that were determined empirically based on observed convergence behavior and validation performance. While these values produced satisfactory results, further optimization could be achieved through systematic hyperparameter tuning, improving training and model accuracy. 

To execute the training process, the script was run from the **scripts/** directory inside the package, invoking the training file from the terminal and providing the required **dataset_path** and **model_path** arguments. The instructions used to launch the training process are shown next:

```bash
source ~/.venv/bin/activate

cd ~/workspace/src/autopilot_neural_network/scripts/

python3 train.py --dataset_path ~/autopilot_neural_network/dataset \
                 --model_path ~/autopilot_neural_network/model.pt
```

### Inference Node

After the training was completed and a model was obtained, an inference node was developed to deploy the trained network within the system. This node, named **autopilot**, was implemented through the **Autopilot** class and was responsible for performing real-time inference using the trained neural network to output velocity and steering commands from incoming camera images. To properly scale the network outputs, the node also required access to the vehicle’s maximum velocity and maximum steering angle parameters, which were provided by the **vehicle_controller** node. For this reason, the **autopilot** node inherited from the **ParametersClientNode**, allowing it to request these parameters at startup.

In addition to the vehicle specific parameters retrieved at startup, the **autopilot** node relied on other configuration parameters to define its runtime behavior and data interfaces. These included the names of the ROS 2 topics used to receive camera images and to publish velocity and steering commands, ensuring proper integration with the rest of the system. Image resolution parameters were also required to match the preprocessing pipeline used during training, guaranteeing consistency between training and inference inputs. Finally, the path to the trained model file was provided as a parameter, allowing the inference node to load the appropriate network weights at initialization.

The Autopilot class structure was designed to support real time operation by separating data reception from model execution, while maintaining consistency with the training configuration in terms of preprocessing and network architecture. Camera images were received through a subscriber configured with a **QoS** profile that kept only the most recent message and used best effort reliability. This choice favored low latency over completeness, ensuring that outdated frames were discarded and that the node always processed the freshest available visual information, which is critical for real time vehicle control.

To further improve it, inference was performed in a dedicated thread, decoupled from the ROS callback that received images. The callback simply stored the latest frame in a shared buffer, while the inference loop continuously checked for new data, processed it, and immediately dropped older frames. This design prevented blocking the ROS executor and avoided backlogs when inference time exceeded the camera frame rate.

Within the inference pipeline, incoming images were converted, preprocessed to match the training setup, and passed through the trained neural network. The resulting normalized predictions were then scaled using the vehicle parameters and published as velocity and steering commands.

The code implementing all of this inference logic was located in **autopilot_neural_network/autopilot_neural_network/autopilot.py**, and is presented next:

```python
import threading
import time

import torch
import PIL

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image

from autopilot_neural_network.parameters_client_node import ParametersClientNode
from scripts.model import AutopilotNet
from scripts.dataset import preprocessing_transform

from cv_bridge import CvBridge
import cv2


class Autopilot(ParametersClientNode):
    """
    Subscribes to camera images and continuously runs inference in a separate thread
    to generate steering and velocity commands. Older frames are dropped to ensure
    real-time processing. Uses parameters from a vehicle controller node to scale network 
    outputs to actual vehicle commands.
    """

    def __init__(self):
        """
        @brief Initializes the Autopilot node.

        Loads the neural network model, sets up image preprocessing, ROS publishers and subscribers,
        and starts the inference thread.
        """
        super().__init__("autopilot")

        # Load ROS parameters
        self.declare_parameter("vehicle_node", "vehicle_controller")
        self.declare_parameter("velocity_topic", "/velocity")
        self.declare_parameter("steering_angle_topic", "/steering_angle")
        self.declare_parameter("image_topic", "/camera/image_raw")
        
        self.declare_parameter("image_width", 128)
        self.declare_parameter("image_height", 96)
        
        self.declare_parameter('model_path', '/tmp/autopilot_neural_network/model.pt')
        
        self.declare_parameter("max_velocity_parameter", "max_velocity")
        self.declare_parameter("max_steering_angle_parameter", "max_steering_angle")
        
        # Retrieve parameters
        self.vehicle_node = self.get_parameter("vehicle_node").value
        self.velocity_topic = self.get_parameter("velocity_topic").value
        self.steering_angle_topic = self.get_parameter("steering_angle_topic").value
        self.image_topic = self.get_parameter("image_topic").value
        self.image_width = self.get_parameter("image_width").value
        self.image_height = self.get_parameter("image_height").value
        self.model_path = self.get_parameter("model_path").value
        
        max_velocity_parameter = self.get_parameter("max_velocity_parameter").value
        max_steering_angle_parameter = self.get_parameter("max_steering_angle_parameter").value

        # Vehicle controller parameters
        self.max_velocity = None
        self.max_steering_angle = None

        # Request vehicle controller parameters (max_velocity, max_steering_angle)
        self.max_velocity, self.max_steering_angle = self.request_parameters(
            self.vehicle_node, [max_velocity_parameter, max_steering_angle_parameter])

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Load neural network model
        self.get_logger().info(f"Loading model from: {self.model_path}")

        self.model = AutopilotNet()
        
        checkpoint = torch.load(self.model_path, map_location=self.device)
        
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model.eval()
        self.model.to(self.device)

        # Image preprocessing
        self.bridge = CvBridge()
        
        # Initialize the image preprocessing pipeline to match the training configuration.
        self.transform = preprocessing_transform(self.image_height, 
                                                 self.image_width)
        
        # QoS for subscriber: keep only the latest message
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1)

        # Publishers
        self.velocity_pub = self.create_publisher(
            Float64, self.velocity_topic, 1)
        
        self.steering_pub = self.create_publisher(
            Float64, self.steering_angle_topic, 1)

        # Subscriber
        self.create_subscription(Image, self.image_topic, 
                                 self._image_callback, qos_profile)

        # Inference management
        self.latest_frame = None # Stores the latest camera frame
        self.lock = threading.Lock() # Protects access to latest_frame
        self.running = True # Controls inference thread
        self.inference_thread = threading.Thread(target=self._inference_loop,
                                                 daemon=True)
        self.inference_thread.start()

        self.get_logger().info("Autopilot initialized.")

    def _image_callback(self, msg: Image):
        """
        @brief Callback for incoming camera images.

        Stores the latest frame and discards older ones, implementing UDP-like behavior.

        @param msg ROS Image message from camera topic
        """
        with self.lock:
            self.latest_frame = msg

    def _inference_loop(self):
        """
        @brief Continuous loop that processes the latest frame.

        Runs in a separate thread, checking for new frames and running inference.
        Older frames are dropped to ensure real-time performance.
        """
        while self.running and rclpy.ok():
            frame = None
            with self.lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame
                    self.latest_frame = None  # Drop older frames

            if frame is not None:
                self._process_frame(frame)
            else:
                time.sleep(0.005)  # Small delay to avoid busy-waiting

    def _process_frame(self, msg: Image):
        """
        @brief Converts the ROS Image message, runs inference, and publishes commands.

        Converts the image to grayscale, applies preprocessing, runs the neural network,
        scales outputs by vehicle parameters, and publishes velocity and steering messages.

        @param msg ROS Image message to process
        """
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        gray_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY) # Convert to grayscale
        pil_gray = PIL.Image.fromarray(gray_image) # Convert to PIL Image
        input_tensor = self.transform(pil_gray).unsqueeze(0).to(self.device)
        
        # Inference
        with torch.no_grad():
            output = self.model(input_tensor)
            pred_velocity = output[0, 0]
            pred_steering = output[0, 1]

        # Scale predictions to actual vehicle commands
        velocity = float(pred_velocity.item()) * self.max_velocity
        steering = float(pred_steering.item()) * self.max_steering_angle

        # Publish results
        self.velocity_pub.publish(Float64(data=velocity))
        self.steering_pub.publish(Float64(data=steering))

        # self.get_logger().info(f"Published - Velocity: {velocity:.3f}, Steering: {steering:.3f}")

    def destroy_node(self):
        """
        @brief Stops the inference thread before destroying the ROS2 node.

        Ensures the inference thread is safely joined to avoid dangling threads.
        """
        self.running = False
        self.inference_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = Autopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Since the **autopilot** node shared several configuration parameters with the **data_collector** node, it was decided to reuse the same configuration file to centralize parameter management and ensure consistency across nodes. As a result, the existing configuration file located at **autopilot_neural_network/config/parameters.yaml** was extended to include the additional parameters required by the Autopilot node. The updated version of this file, reflecting the newly added parameters, is shown next:

```yaml
/**:
  ros__parameters:
    # Vehicle node and topics
    vehicle_node: "vehicle_controller" # Name of the node from which to retrieve vehicle parameters
    velocity_topic: "/velocity" # Topic for publishing/subscribing to velocity commands
    steering_angle_topic: "/steering_angle" # Topic for publishing/subscribing to steering commands
    image_topic: "/camera/image_raw" # Topic for subscribing to raw camera images
    max_velocity_parameter: "max_velocity" # Vehicle parameter defining max velocity
    max_steering_angle_parameter: "max_steering_angle" # Vehicle parameter defining max steering

    # Data Collector Node Parameters
    min_velocity_factor: 0.25 # Fraction of max velocity below which data is not recorded
    timeout_time: 1000000000 # Timeout for data freshness in nanoseconds [ns]
    update_period: 0.5 # Period for the node's update loop in seconds [s]

    # Autopilot (Inference) Node Parameters
    image_height: 96 # Height of the image for model input [px] 
    image_width: 128 # Width of the image for model input [px] 

    # File Paths
    dataset_path: "/tmp/autopilot_neural_network/dataset" # Path where dataset is saved and loaded
    model_path: "/tmp/autopilot_neural_network/model.pt" # Path to the trained model for inference
```

As told before, the parameters in this configuration file must be adapted to the specific setup being used. In particular, **model_path** parameters should be updated to reflect the model location, pointing to the current file generated during training to ensure that the inference node loads the correct trained network.

To integrate the inference node into the system, a launch file was created to start the **autopilot** node and load the shared parameters at runtime, ensuring that all required parameters were available as soon as the node was initialized. This launch file was placed in the **autopilot_neural_network/launch** directory under the name **autopilot.launch.py**, and its implementation is shown next:

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launches the autopilot node with parameters loaded from a YAML file.
    """

    package_name = 'autopilot_neural_network'
    package_share = get_package_share_directory(package_name)

    # Path to the YAML configuration file
    params_file = os.path.join(package_share, 'config', 'parameters.yaml')

    # Define the node to launch
    autopilot_node = Node(package=package_name,
                          executable='autopilot',
                          name='autopilot',
                          output='screen',
                          parameters=[params_file])

    return LaunchDescription([autopilot_node])
```

Finally, the **autopilot_neural_network/setup.py** file was updated to register the **autopilot** node within the package. This change ensured that the node could be executed as a console script and that the associated launch and configuration files were correctly installed and made available at runtime through the package share directory. The updated **setup.py** file is shown below:

```python
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'autopilot_neural_network'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # includes scripts/ automatically
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Mazzetto',
    maintainer_email='workabotic@gmail.com',
    description='An end-to-end deep learning pipeline for autonomous driving in ROS 2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'autopilot = autopilot_neural_network.autopilot:main',
            'data_collector = autopilot_neural_network.data_collector:main',
        ],
    },
)
```

At this stage, the **autopilot_neural_network** package was complete, integrating data collection, model training, and real-time inference to drive the vehicle simulation autonomously. With the full package implemented, the next step was to build the package. The process began by sourcing the environment, after which the package was compiled using **colcon build**, making it ready for execution within the ROS 2 workspace. These steps are illustrated by the command lines shown below.

```bash
source /opt/ros/jazzy/setup.bash

source ~/.venv/bin/activate

cd ~/workspace

source install/setup.bash

colcon build
```

After the package was successfully built, running the inference node required starting the simulation environment first. This was done by launching the vehicle simulation through the **vehicle.launch.py** file, which initialized the Gazebo world and the vehicle controller, with the commands shown below:

```bash
source /opt/ros/jazzy/setup.bash 

source ~/.venv/bin/activate

cd ~/workspace 

source install/setup.bash 

ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py \
  world:=$(ros2 pkg prefix gazebo_racing_tracks)/share/gazebo_racing_tracks/worlds/grass_track.sdf
```

Once the simulation was running, the autopilot node was started in a separate terminal, following the commands shown next:

```bash
source /opt/ros/jazzy/setup.bash 

source ~/.venv/bin/activate

cd ~/workspace 

source install/setup.bash 

ros2 launch autopilot_neural_network autopilot.launch.py
```

Once launched, the autopilot node began publishing velocity and steering commands to the **vehicle_controller** node, successfully controlling the vehicle in real time and enabling it to navigate the simulation environment autonomously.

![Autonomous driving simulation]({{ site.url }}{{ site.baseurl }}/public/images/autonomous-driving-vehicle-simulation/autonomous-driving-simulation.gif)
*Figure 4 — Autonomous driving simulation.*

## Results

The training process showed stable convergence, with both training and validation loss decreasing consistently over the epochs. The alignment between these curves indicates effective learning without overfitting, demonstrating that the proposed model was capable of learning meaningful control behaviors from the collected dataset. The evolution of the loss values is shown in the following plots.

![Training and validation loss]({{ site.url }}{{ site.baseurl }}/public/images/autonomous-driving-vehicle-simulation/training-and-validation-loss.webp)
*Figure 5 — Training and validation loss.*

After completing the training process and integrating the neural network into the inference pipeline, the model’s performance was evaluated in simulation. The primary evaluation was conducted on the **grass_track.sdf** world, which was excluded from the training dataset and therefore had never been seen by the model. This setup allowed the evaluation of the model’s ability to generalize to unseen environments with different visual characteristics. During evaluation, the vehicle was able to autonomously drive the entire track multiple times without leaving the lane, demonstrating that the model successfully learned the driving task and generalized well to unseen conditions.

![Autonomous driving evaluation]({{ site.url }}{{ site.baseurl }}/public/images/autonomous-driving-vehicle-simulation/autonomous-driving-evaluation.gif)
*Figure 6 — Autonomous driving evaluation.*

## Conclusion

In this project, an end-to-end autonomous driving system was developed by combining robotics simulation with deep learning to enable lane-following behavior. The experimental results show that the neural network successfully controlled the vehicle in simulation, navigating racing tracks reliably and generalizing to previously unseen environments, including the grass track.

Moreover, the same end-to-end approach is fully applicable to real robotic platforms. By training the neural network using data acquired from a real camera and executing the resulting commands on a real vehicle, following the same principles used in simulation, the proposed system demonstrates its applicability to real-world autonomous driving scenarios, as illustrated below:

![Real robot driving autonomously]({{ site.url }}{{ site.baseurl }}/public/images/autonomous-driving-vehicle-simulation/real-robot-driving-autonomously.gif)
*Figure 7 — Real robot driving autonomously.*