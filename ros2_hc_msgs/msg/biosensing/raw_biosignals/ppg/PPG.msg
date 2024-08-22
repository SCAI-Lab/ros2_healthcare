PPGHeader		header

float32[]		ppg 				#PPG calculated by device in [a.u.]
float32[]       signal_quality         #signal quality from 0-1 [%]
float64[]       time_signal_recorded   #The moment in time the signal was recorded, whereas Stamp from std_header is the publishing time 