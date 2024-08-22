std_msgs/Header header


string 	    device_serial_number 	    #Serialnumber of the device of signal origin
string	    unit                        #Unit of the main output value 
uint8		sampling_frequency 		    #Sampling frequency of the device in [Hz] 	
float32	    resolution 				    #Resolution of the output in [unit]	
float32 	accuracy 			    	#Accuracy of the output signal tested by the manufacturer from 0-1 [%]
float32 	max_range 			        #Maximal detectable input in [unit]
float32 	min_range 			   	    #Minimal detectable input in [unit]

string      electrode_type              #Type of electrode used to measure signal (E.g. surface electrode, needle electrode)
string      electrode_location          #locations where the measuring electrodes are placed 
string      external_devices            #External devices that provide forms of stimulation to provide timing signals through triggers
