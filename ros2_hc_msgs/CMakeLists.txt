cmake_minimum_required(VERSION 3.8)
project(ros_healthcare_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(msg_files

"msg/biosensing/derivedbiosignals/rr/RRHeader.msg"
"msg/biosensing/derivedbiosignals/rr/RR.msg"
"msg/biosensing/derivedbiosignals/hrv/HRV.msg"
"msg/biosensing/derivedbiosignals/hrv/HRVHeader.msg"
"msg/biosensing/derivedbiosignals/co/COHeader.msg"
"msg/biosensing/derivedbiosignals/co/CO.msg"
"msg/biosensing/derivedbiosignals/hr/HRHeader.msg"
"msg/biosensing/derivedbiosignals/hr/HR.msg"
"msg/biosensing/derivedbiosignals/sv/SV.msg"
"msg/biosensing/derivedbiosignals/sv/SVHeader.msg"
"msg/biosensing/biosignals/bcg/BCG.msg"
"msg/biosensing/biosignals/bcg/BCGHeader.msg"
"msg/biosensing/biosignals/ecg/ECGLeads.msg"
"msg/biosensing/biosignals/ecg/ECGLead.msg"
"msg/biosensing/biosignals/ecg/ECGHeader.msg"
"msg/biosensing/biosignals/eeg/EEGHeader.msg"
"msg/biosensing/biosignals/eeg/EEGChannels.msg"
"msg/biosensing/biosignals/eeg/EEGChannel.msg"
"msg/biosensing/biosignals/eda/EDAHeader.msg"
"msg/biosensing/biosignals/eda/EDA.msg"
"msg/biosensing/biosignals/emg/EMG.msg"
"msg/biosensing/biosignals/emg/EMGHeader.msg"
"msg/biosensing/biosignals/ppg/PPG.msg"
"msg/biosensing/biosignals/ppg/PPGHeader.msg"
"msg/biosensing/biosignals/ppg/PPGs.msg"
"msg/biosensing/biosignals/icg/ICGLead.msg"
"msg/biosensing/biosignals/icg/ICGHeader.msg"
"msg/biosensing/biosignals/icg/ICGLeads.msg"
"msg/biosensing/biosignals/eog/EOGHeader.msg"
"msg/biosensing/biosignals/eog/EOG.msg"
"msg/physicalsensing/externalsignals/force/Force.msg"
"msg/physicalsensing/externalsignals/pressure/Pressure.msg"
"msg/physicalsensing/externalsignals/pressure/PressureHeader.msg"
"msg/physicalsensing/derivedsignals/jointangularvelocity/JointAngularVelocity.msg"
"msg/physicalsensing/derivedsignals/jointmoments/JointMoments.msg"
"msg/physicalsensing/derivedsignals/jointtangles/JointAngle.msg"
"msg/physicalsensing/derivedsignals/pose/Pose.msg"
"msg/physicalsensing/derivedsignals/elevationangle/ElevationAngle.msg"
"msg/physicalsensing/derivedsignals/steps/Step.msg"
"msg/hardware/biosensors/device/Device.msg"
"msg/hardware/biosensors/filter/Filter.msg"
"msg/biometrics/physiologicalbiometrics/posture/PostureHeader.msg"
"msg/biometrics/physiologicalbiometrics/posture/Posture.msg"
"msg/biometrics/physiologicalbiometrics/gaitbiometrics/GaitBiometrics.msg"
"msg/biometrics/physiologicalbiometrics/adl/ADLHeader.msg"
"msg/biometrics/physiologicalbiometrics/adl/ADL.msg"
"msg/biometrics/behavioralbiometrics/mood/Mood.msg"

)

#set (srv_files
#   "srv/AddThreeInts.srv"
#)



rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  #${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
  DEPENDENCIES builtin_interfaces geometry_msgs
 )
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
 
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
 
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

