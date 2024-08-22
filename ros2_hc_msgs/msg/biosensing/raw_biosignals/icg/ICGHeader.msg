std_msgs/Header header

string 	    device_serial_number 	    #Serialnumber of the device of signal origin
string	    unit                        #Unit of the main output value 
uint16		sampling_frequency 		    #Sampling frequency of the device in [Hz] 	
float32	    resolution 				    #Resolution of the output in [unit]	
float32 	accuracy 				    #Accuracy of the output signal tested by the manufacturer from 0-1 [%]
float32 	max_range 				    #Maximal detectable input in [unit]
float32 	min_range 				    #Minimal detectable input in [unit]
		


uint8	    lead_number 			    #The lead number used to achieve the signal	
string	    location_lead			    #Location where the signal is recorded 
