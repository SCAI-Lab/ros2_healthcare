std_msgs/Header header


string 	    device_serial_number 	    #Serialnumber of the device of signal origin
string	    unit                        #Unit of the main output value 
uint8		sampling_frequency 		    #Sampling frequency of the device in [Hz] 	
float32	    resolution 				    #Resolution of the output in [unit]	
float32 	accuracy 			    	#Accuracy of the output signal tested by the manufacturer from 0-1 [%]
float32 	max_range 			        #Maximal detectable input in [unit]
float32 	min_range 			   	    #Minimal detectable input in [unit]

string[]    electrode_placement     #Location where the electrodes are placed 
float32     gazefield_background_light_brightness   #The brightness of the gazeflied background light in [cd/m²]
string[]    length_light_dark_phase      #duration of the light phase followed by the dureation of the dark phase 
string      type_lightsource            #Whicht source of adapting light is used for the measurement (E.g. tungsten, halogen, ect.)
