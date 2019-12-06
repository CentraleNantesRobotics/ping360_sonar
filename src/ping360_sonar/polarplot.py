#!/usr/bin/env python

import roslib
roslib.load_manifest('ping360_sonar')

from sensor import Ping360
import argparse
import cv2
import numpy as np
from math import pi, cos, sin
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy

def main():
     parser = argparse.ArgumentParser(description="Ping python library example.")
     parser.add_argument('--device', action="store", required=True, type=str, help="Ping device port.")
     parser.add_argument('--baudrate', action="store", type=int, default=2000000, help="Ping device baudrate.")
     parser.add_argument('--v', action="store", type=bool, default=False, help="Verbose")
     parser.add_argument('--size', action="store", type=int, default=200, help="Image Size")

     args = parser.parse_args()
     try:
          rospy.init_node('ping360_node')
          
          # Ping 360 Parameters
          device = "/dev/ttyUSB0"
          baudrate = 115200
          gain = 0
          numberOfSamples = 200 # Number of points
          transmitFrequency = 740 # Default frequency
          sonarRange = 1 # in m
          speedOfSound = 1500 # in m/s
          samplePeriod = calculateSamplePeriod(sonarRange, numberOfSamples, speedOfSound)
          transmitDuration = adjustTransmitDuration(sonarRange, samplePeriod, speedOfSound)

          # Output and ROS parameters
          step = 1
          topic = "ping360_sonar"
          imgSize = args.size
          queue_size= 1

          # Global Variables
          angle = 0
          p = Ping360(args.device, args.baudrate)
          imagePub = rospy.Publisher(topic, Image, queue_size=queue_size)
          bridge = CvBridge()

          # Initialize and configure the sonar
          print("Initialized: %s" % p.initialize())
          p.set_gain_setting(gain)
          p.set_transmit_frequency(transmitFrequency)
          p.set_transmit_duration(transmitDuration)
          p.set_sample_period(samplePeriod)
          p.set_number_of_samples(numberOfSamples)

          # Create a new mono-channel image
          image = np.zeros((imgSize, imgSize, 1), np.uint8)

          # Center point coordinates
          center = (float(imgSize/2),float(imgSize/2))
          while not rospy.is_shutdown():
               p.transmitAngle(angle)
               data = bytearray(getattr(p,'_data'))
               data_lst = [k for k in data]
               linear_factor = float(len(data_lst)) / float(center[0]) #TODO: this should probably be range/pixelsize
               try:
                    for i in range(int(center[0])): #TODO: check the update polar logic on ping-viewer
                         if(i < center[0]):
                              pointColor = data_lst[int(i*linear_factor-1)]
                         else:
                              pointColor = 0
                         for k in np.linspace(0,step,8*step):
                              theta = 2*pi*(angle+k)/400.0
                              x = float(i)*cos(theta)
                              y = float(i)*sin(theta)
                              image[int(center[0]+x)][int(center[1]+y)][0] = pointColor
               except IndexError:
                    continue
               
               angle = (angle + step) % 400
               if args.v:
                    cv2.imshow("PolarPlot",image)
                    cv2.waitKey(27)
               publish(image, imagePub, bridge)
               rospy.sleep(0.1)
     
     except KeyboardInterrupt:
          print("Shutting down")
          cv2.destroyAllWindows()

def publish(image, imagePub, bridge):
     try:
          imagePub.publish(bridge.cv2_to_imgmsg(image, "mono8"))
     except CvBridgeError as e:
          print(e)

# https://discuss.bluerobotics.com/t/please-provide-some-answer-regards-ping360/6393/3?u=stormix

def calculateSamplePeriod(distance, numberOfSamples, speedOfSound, _samplePeriodTickDuration = 25e-9):
     # type: (float, int, int, float) -> float
     """
      Calculate the sample period based in the new range
     """
     return 2*distance/(numberOfSamples*speedOfSound*_samplePeriodTickDuration)

def adjustTransmitDuration(distance, samplePeriod, speedOfSound, _firmwareMinTransmitDuration = 5):
     # type: (float, float, int, int) -> float
     """
     @brief Adjust the transmit duration for a specific range
     Per firmware engineer:
     1. Starting point is TxPulse in usec = ((one-way range in metres) * 8000) / (Velocity of sound in metres
     per second)
     2. Then check that TxPulse is wide enough for currently selected sample interval in usec, i.e.,
          if TxPulse < (2.5 * sample interval) then TxPulse = (2.5 * sample interval)
     3. Perform limit checking
     @return Transmit duration
     """
     # 1
     duration = 8000 * distance / speedOfSound
     # 2 (transmit duration is microseconds, samplePeriod() is nanoseconds)
     transmit_duration = max(2.5*getSamplePeriod(samplePeriod)/1000, duration)
     # 3
     return max(_firmwareMinTransmitDuration, min(transmitDurationMax(samplePeriod), transmit_duration))

def transmitDurationMax(samplePeriod, _firmwareMaxTransmitDuration = 500):
     # type: (float, int) -> float
     """
     @brief The maximum transmit duration that will be applied is limited internally by the
     firmware to prevent damage to the hardware
     The maximum transmit duration is equal to 64 * the sample period in microseconds
     @return The maximum transmit duration possible
     """
     return min(_firmwareMaxTransmitDuration, getSamplePeriod(samplePeriod) * 64e6)

def getSamplePeriod(samplePeriod, _samplePeriodTickDuration = 25e-9):
     # type: (float, float) -> float
     """  Sample period in ns """
     return samplePeriod*_samplePeriodTickDuration