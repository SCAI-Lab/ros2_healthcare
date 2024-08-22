std_msgs/Header header


string 	    device_serial_number 
string      unit                    #Unit of the main output value
uint16		sampling_frequency 		#Samplingfrequency of the origin signal in [Hz]
float32     resolution              #Resolution of the output in [unit]
float32 	accuracy 			   	#Accuracy of the output signal tested by the manufacturer from 0-1 [%]
float32 	max_range 				#Maximal detectable input in [unit]
float32  	min_range 				#Minimal detectable input in [unit]

float32     stride_length           #Amount window is shifted per change 
float32     window_size             #Window size used to calculate value 
uint16      signal_refresh_rate     #Intervall by which the derived biodignal is refreshed [Hz]