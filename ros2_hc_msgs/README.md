# healthcare_msgs

This repository contains ROS 2 message definitions for biosensor data, which are organized into subfolders for easy access and management. 

The message files are divided into the following subfolders:

- **Biosignals:** 

  This folder contains message files related to the biosignals captured by the biosensor.

- **Derived Biosignals:**

  Here, you will find message files for derived biosignals. Meaning that they are derived or computed from biosignals.

- **Derived Biometrics:**

  This folder includes message files for derived biometrics, which are higher-level measurements or indicators obtained from the biosignals.

- **Sensors:** 

  The message files specific to the sensor hardware used for capturing the biosignals are stored in this folder.

## Structure

By organizing the message files into separate subfolders and defining a clear and standardised structure, this repository aims to enhance the clarity and ease of use when working with biosensor data within the ROS framework. 

### Header 

Each signal specific header consists of a standart header, which includes common/mandatory fields within the subfolder. There are slight differences between the standard headers of the subfolders. Additionally, the header is supplemented with signal-specific parameters that provide additional information relevant to the specific biosignal being captured.


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


## Usage

To use the healthcare messages in your ROS 2 project: 
- Clone the repo
  ```
  git clone https://github.com/ros-healthcare/healthcare-msgs.git
  ```
- Install python requirements with 
  ```
  pip install -r requirements.txt 
  ```
- Build it and source the installed package:
  ```
  colcon build
  source install/setup.bash
  ```

After this step you can import the desired messages as follows: 

```
from healthcare_msgs.msg import desired_msg
```

You can find publisher and subscriber examples for the messages at the [ROS Healthcare Examples repo](https://github.com/SCAI-Lab/healthcare_examples/).

