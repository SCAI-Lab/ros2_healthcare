ECGHeader            header 
	
float32[]	         ecg     	            #Amplitude of the output value in [unit] 
float32[]            signal_quality         #signal quality from 0-1 [%]
float64[]            time_signal_recorded   #The moment in time the signal was recorded, whereas Stamp from std_header is the publishing time 