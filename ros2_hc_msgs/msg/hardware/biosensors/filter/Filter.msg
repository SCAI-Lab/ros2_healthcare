std_msgs/Header header

uint8       lowpass_filter_frequency            #Frequencies below are passed, higher frequencies are attenuated 
uint8       highpass_filter_frequency           #Higher frequencies are passed, lower frequencies are attenuated 
uint8[]     band_pass_filter_frequency          #Frequencies within the definded borders pass, frequencies outside of this range get attenuated 
uint8[]     band_stop_filter_frequency          #Frequencies within borders get attenuated, frequencies outside of boarders pass 
uint8       notch_filter_frequency              #Only this specific frequency gets attenuated 
uint8[]     comb_filter_frequency               #Multiple regularly spaced passbands. Values next to each other represent the boarders of the passband. Starting with the pair with index 0 and 1