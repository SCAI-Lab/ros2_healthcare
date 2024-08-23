# ROS2 Healthcare

Welcome to the **ROS2 Healthcare** project! This repository contains ROS2 packages for healthcare applications, including message definitions, drivers, libraries, tools, and examples. **`ros2_hc`** serves as an example of how one can use biosignals coming from wearable medical devices like Vivalink and mbient wristbands to track patients' activities. In addition, we have developed an Activities of Daily Living (ADL) classifier that will help doctors visualize their patients' daily activity patterns and consequently help them recommend a suitable routine. This can be visualised through our `healthcare_wheelchair_dashboard`.


## Repository Structure

- **`ros2_hc_drv`**: Includes drivers for various biosensors.
- **`ros2_hc_examples`**: Example applications and use cases demonstration.
- **`ros2_hc_lib`**: Libraries
- **`ros2_hc_msgs`**: Contains all message definitions and structure for biosensor data.
- **`ros2_hc_tools`**: Tools and utilities for development and operation.



```
ros2-healthcare
├── ros2_hc_drv
│   ├── mbient_ros
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   ├── launch
│   │   ├── mbient_ros
│   │   └── scripts
│   ├── sensomative_ros
│   │   ├── config
│   │   ├── launch
│   │   ├── scripts
│   │   └── sensomative_ros
│   └── vivalink_ros
│       ├── config
│       ├── launch
│       ├── scripts
│       └── vivalink_ros
├── ros2_hc_examples
│   ├── applications
│   │   ├── healthcare_wheelchair_dashboard
│   │   └── topic_visualization
│   └── nodes
│       └── example_nodes
├── ros2_hc_lib
│   └── healthcare_adl_classifier
├── ros2_hc_msgs
│   ├── msg
│   │   ├── biometrics
│   │   │   ├── behavioral
│   │   │   │   └── mood
│   │   │   ├── physiological
│   │   │   │   ├── adl
│   │   │   │   ├── gait
│   │   │   │   └── posture
│   │   ├── biosensing
│   │   │   ├── derived_biosignals
│   │   │   │   ├── co
│   │   │   │   ├── hr
│   │   │   │   ├── hrv
│   │   │   │   ├── rr
│   │   │   │   └── sv
│   │   │   ├── raw_biosignals
│   │   │   │   ├── bcg
│   │   │   │   ├── ecg
│   │   │   │   ├── eda
│   │   │   │   ├── eeg
│   │   │   │   ├── emg
│   │   │   │   ├── eog
│   │   │   │   ├── icg
│   │   │   │   └── ppg
│   │   ├── physical_sensors
│   │   │   ├── derived_signals
│   │   │   │   ├── elevation_angle
│   │   │   │   ├── joint_angles
│   │   │   │   ├── joint_angular_velocity
│   │   │   │   ├── pose
│   │   │   │   └── steps
│   │   │   ├── external_signals
│   │   │   │   ├── force
│   │   │   │   └── pressure
└── ros2_hc_tools
```

```

