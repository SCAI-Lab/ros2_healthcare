std_msgs/Header header

string 	    device_serial_number 	    #Serialnumber of the device of signal origin
string	    unit                        #unit of the main output value 
uint16		sampling_frequency 		    #samplingfrequency of the device in [Hz] 	
float32	    resolution 				    #Resolution of the output in [unit]
float32 	accuracy 				    #Accuracy of the output signal tested by the manufacturer from 0-1 [%]
float32 	max_range 				    #maximal detectable input in [unit]
float32 	min_range 				    #minimal detectable input in [unit]
		


uint8		channel_number 			    #The channel number of the electrode 
string      electrode_name              #Electrode name according to the standard electrode position nomenclature 
string	    electrode_placement_methode #The standard according to which the electrodes are placed(e.g.20/10,10/10,10/5)
uint8       signal_gain                 #Gain refers to the amount by which the signals amplitude is increased or decreased[db] (Gain = 1 -> signal unchanged, gain <1 signal is attenuated, gain >1 signal is amplified)

