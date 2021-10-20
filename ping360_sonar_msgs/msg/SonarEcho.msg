std_msgs/Header header            #header info
float32 angle               # the measurement angle [rad]
uint8 gain  # Sonar Gain
uint16 number_of_samples 
uint16 transmit_frequency # [kHz]
uint16 speed_of_sound # [m/s]
uint8 range      #  range value [m]
uint8[] intensities    # intensity data [0-255].  This is the actual data received from the sonar
