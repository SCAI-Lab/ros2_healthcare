# ROS2 Healthcare Examples

The `ros2_hc_examples` repository provides a collection of example applications and nodes demonstrating how to use and visualize biosignals through the given `ros2_hc_msgs` interface. These examples are designed to help developers understand the integration of sensors, data processing, and visualization in a ROS 2 environment.

### 1. Applications

The `applications` directory includes example applications that showcase systems using biosignals. The current example focuses on a healthcare wheelchair dashboard, which provides real-time visualization and interaction with biosignal data.

- **`healthcare_wheelchair_dashboard/`**  
  This application serves as a visualization tool for biosignals, tailored for use in a healthcare context, such as monitoring patients in a wheelchair. It includes:
  
  - **`dashboard/`**: Contains the core components and user interface elements for the dashboard.
  - **`dashboard_msg/`**: Houses the custom messages and communication interfaces used by the dashboard to interact with other ROS 2 nodes.

### 2. Nodes

The `nodes` directory contains example publisher and subscriber nodes that demonstrate how to interact with biosignals using the `ros2_hc_msgs` interface. These examples shows the starting points to create their own ROS2 nodes for processing or visualizing healthcare data.

