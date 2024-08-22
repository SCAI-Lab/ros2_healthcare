std_msgs/Header header

string 	    device_serial_number 	    #Serialnumber of the device of signal origin
string	    unit                        #Unit of the main output value 
uint8		sampling_frequency 		    #Sampling frequency of the device in [Hz] 	
float32	    resolution 				    #Resolution of the output in [unit]	
float32 	accuracy 			    	#Accuracy of the output signal tested by the manufacturer from 0-1 [%]
float32	    max_range 			        #Maximal detectable input in [unit]
float32 	min_range 			   	    #Minimal detectable input in [unit]

uint16      number_of_sensors           #Number of sensors used for the measurement
string[]    sensor_location             #Location of the sensors used to measure bcg
string      sensing_direction           #Defining along which axis bcg is measured. (E.g. dorso-ventral bcg/longitudinal BCG)
string      sensing_modality            #E.g piezoelectric-films/forceplates/ect.
string      measurement_system          #Types of system used to measure bcg. (E.g. Starr BCG/Nickerson BCG/Dock BCG/ ect.)

      
