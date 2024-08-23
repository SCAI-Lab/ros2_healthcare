# ROS2 Healthcare msgs

This repository contains ROS2 message definitions for biosensor data, organized into subfolders for easy access and management. 


## `ros2_hc_msgs` Directory Structure

The `ros2_hc_msgs` directory contains message definitions organized into various categories. Each category is designed to group related message files, making it easier to manage and use the messages in ROS 2 applications. Below is the detailed hierarchy and description of each folder:

## Directory Structure

```
msgs/
├── biometric/
│   ├── behavioral/
│   │   └── mood/
│   └── physiological/
│       ├── adl/
│       ├── gait/
│       └── posture/
├── biosensing/
│   ├── raw_biosignals/
│   │   ├── bcg/
│   │   ├── ecg/
│   │   ├── eda/
│   │   ├── eeg/
│   │   ├── emg/
│   │   ├── eog/
│   │   ├── icg/
│   │   ├── ppg/
│   └── derived_biosignals/
│       ├── co/
│       ├── hr/
│       ├── hrv/
│       ├── rr/
│       └── sv/
├── hardware/biosensors
│   ├── devices/
│   └── filters/
├── physical_sensor/
│   ├── derived_signals/
│   │   ├── elevation_angle/
│   │   ├── joint_angles/
│   │   ├── joint_angular_velocity/
│   │   ├── joint_moments/
│   │   ├── pose/
│   │   └── steps/
│   └── external_signals/
│       ├── force/
│       └── pressure/

```

The message files are divided into the following directories:

- **Biometrics:**

  This directory includes message files for derived biometrics, higher-level measurements or indicators obtained from the biosignals.

- **Biosensing:** 

  This folder contains message files related to the biosignals captured by the biosensor.

- **Hardware:**

  The hardware folder contains message files related to the physical components and devices used in biosensing.

- **Physical Sensors:** 

  The message files specific to the sensor hardware used for capturing the biosignals are stored in this folder.

## Structure

By organizing the message files into separate subfolders and defining a clear and standardised structure, this repository aims to enhance the clarity and ease of use when working with biosensor data within the ROS2 framework. 

### Header 

Each signal-specific header consists of a standard header, which includes common/mandatory fields within the subfolder. There are slight differences between the standard headers of the subfolders. Additionally, the header is supplemented with signal-specific parameters that provide additional information relevant to the specific biosignal being captured.


**Header Biosignals**
`healthcare_msgs/{bio_signal}Header`
````
std_msgs/Header header

string      device_serialnumber      #Serialnumber of the device of signal origin
string      unit                     #Unit of the main output value 
uint16      sampling_frequency       #Samplingfrequency of the device in [Hz] 	
float32     resolution               #Resolution of the output in [unit]		
float32     accuracy                 #Accuracy of the output signal tested by the manufacturer from 0-1 [%]  
float32     max_range                #Maximal detectable input in [unit]
float32     min_range                #Minimal detectable input in [unit]

````

**Header Derived Biosignals** 
`healthcare_msgs/{derived_bio_signal}Header`
```
std_msgs/Header header

string      device_serialnumber       #Serialnumber of the device of signal origin
string      unit                      #Unit of the main output value
uint16      sampling_frequency        #Samplingfrequency of the origin signal in [Hz]
float32     resolution                #Resolution of the output in [unit]
float32     accuracy                  #Accuracy of the output signal tested by the manufacturer from 0-1 [%]
float32     max_range                 #Maximal detectable input in [unit]
float32     min_range                 #Minimal detectable input in [unit]
float32     stride_length             #Amount window is shifted per change 
float32     window_size               #Window size used to calculate value 
uint16      signal_refresh_rate       #Intervall by which the derived biodignal is refreshed [Hz]
```

**Header Derived Biometrics**
`healthcare_msgs/{derived_biometric}Header`
```
std_msgs/Header header

string       device_serialnumber         #Serialnumber of the device of signal origin
float32[]    prediction_probablility     #Probability of prediction's according to classifier
string[]     glossary                    #possibilities linked to the prediction_probability with same index   
string[]     model_type                  #Machine learning model type
string[]     model_version               #The version the model is trained with/ From what data was the model created 

```
