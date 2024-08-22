std_msgs/Header header

string 	    device_serial_number 	    #Serialnumber of the device of signal origin
string	    unit                        #unit of the main output value 
uint16		sampling_frequency 		    #samplingfrequency of the device in [Hz] 	
float32	    resolution 				    #Resolution of the output in [brpm]
float32 	accuracy 			       	#Accuracy of the output signal tested by the manufacturer from 0-1 [%]
float32 	max_range 				    #maximal detectable input in [brpm]
float32 	min_range 				    #minimal detectable input in [brpm]


float32     stride_length           #Amount window is shifted per change 
float32     window_size             #Window size used to calculate value 
uint16      signal_refresh_rate     #Intervall by which the derived biodignal is refreshed [Hz]