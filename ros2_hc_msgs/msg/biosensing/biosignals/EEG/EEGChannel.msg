EEGHeader            header 
	
float32[]	         eeg     	            #Amplitude of the output value in [mV] 
uint8[]              signal_quality         #signal quality from 0-100
float64[]            time_signal_recorded        #The moment in time the signal was recorded, whereas Stamp from std_header is the publishing time 