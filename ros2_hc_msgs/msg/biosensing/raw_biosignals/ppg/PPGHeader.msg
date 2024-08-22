std_msgs/Header header

string 	    device_serial_number 	    #Serialnumber of the device of signal origin
string	    unit                        #unit of the main output value 
uint16		sampling_frequency 		    #samplingfrequency of the device in [Hz] 	
float32	    resolution 				    #Resolution of the output in [unit]
float32 	accuracy 				    #Accuracy of the output signal tested by the manufacturer from 0-1 [%]
float32 	max_range 				    #maximal detectable input in [unit]
float32  	min_range 				    #minimal detectable input in [unit]


uint16      wavelengths 		    	#wavelength of emmitted light in [nm]
float32[]     ac_component              #pulsatile AC component (cardic synchronous variations in blood volume)
float32[]     dc_component              #superimposed DC component (shaped by sympathetic nervous system activity, respiration, thermoregulation)
string[] 	ppg_location 			    #Location of sensor that measures the ppg
string  	ppg_measuring_mode		    #Measuring method for ppg ("transmissive absorption", "reflection", ect.)

float32[] 	signal_gain 		    	#Gain refers to the amount by which the signals amplitude is increased or decreased. (Gain = 1 -> signal unchanged, gain <1 signal is attenuated, gain >1 signal is amplified)

