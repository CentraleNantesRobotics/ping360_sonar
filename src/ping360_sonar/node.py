#!/usr/bin/env python

import argparse
import json
from math import cos, pi, sin

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ping360_sonar.cfg import sonarConfig
from ping360_sonar.msg import SonarEcho
from sensor import Ping360


def callback(config, level):
     global updated, gain, numberOfSamples, transmitFrequency, transmitDuration, sonarRange, speedOfSound, samplePeriod, debug, step, imgSize, queue_size, p
     # Update Ping 360 Parameters
     gain = config['gain']
     numberOfSamples = config['numberOfSamples']
     transmitFrequency = config['transmitFrequency']
     sonarRange = config['range']
     speedOfSound = config['speedOfSound']
     samplePeriod = calculateSamplePeriod(sonarRange, numberOfSamples, speedOfSound)
     transmitDuration = adjustTransmitDuration(sonarRange, samplePeriod, speedOfSound)
     debug = config['debug']
     step = config['step']
     queue_size = config['queueSize']
     rospy.loginfo("Reconfigure Request")
     updated = True
     return config

def main():
     rospy.init_node('ping360_node')
     srv = Server(sonarConfig, callback)

     # Global Variables
     angle = 0
     bridge = CvBridge()
     
     # Topic publishers
     imagePub = rospy.Publisher("/ping360_node/sonar/images", Image, queue_size=queue_size)
     rawPub = rospy.Publisher("/ping360_node/sonar/data", SonarEcho, queue_size=queue_size)

     # Initialize and configure the sonar
     updateSonarConfig(gain, transmitFrequency, transmitDuration, samplePeriod, numberOfSamples)

     # Create a new mono-channel image
     image = np.zeros((imgSize, imgSize, 1), np.uint8)
     
     # Center point coordinates
     center = (float(imgSize/2),float(imgSize/2))
     
     rate = rospy.Rate(100) # 10hz
     
     while not rospy.is_shutdown():
          
          # Update to the latest config data
          if updated:
               updateSonarConfig(gain, transmitFrequency, transmitDuration, samplePeriod, numberOfSamples)
          
          # Get sonar response
          data = getSonarData(angle)
          
          # Contruct and publish Sonar data msg
          rawDataMsg = generateRawMsg(angle, data) # TODO: check for empty responses
          rawPub.publish(rawDataMsg)
          
          # Contruct and publish Sonar image msg
          linear_factor = float(len(data)) / float(center[0]) #TODO: this should probably be range/pixelsize
          try:
               for i in range(int(center[0])): #TODO: check the update polar logic on the new ping-viewer
                    if(i < center[0]):
                         pointColor = data[int(i * linear_factor - 1)]
                    else:
                         pointColor = 0
                    for k in np.linspace(0,step,8*step):
                         theta = 2 * pi * (angle + k) / 400.0
                         x = float(i) * cos(theta)
                         y = float(i) * sin(theta)
                         image[int(center[0] + x)][int(center[1] + y)][0] = pointColor
          except IndexError:
               rospy.logwarn("IndexError: data response was empty, skipping this iteration..")
               continue
          
          angle = (angle + step) % 400
          if debug:
               cv2.imshow("PolarPlot",image)
               cv2.waitKey(1)
          else:
               cv2.destroyAllWindows()
          publishImage(image, imagePub, bridge)
          rate.sleep()

def getSonarData(angle):
     """
     Transmits the sonar angle and returns the sonar intensities
     Params:
          angle int (Gradian Angle)
     Return:
          data list Intensities from 0 - 255 
     """
     p.transmitAngle(angle)
     data = bytearray(getattr(p,'_data'))
     return [k for k in data]

def generateRawMsg(angle, data):
     """
     Generates the raw message for the data topic
     Params:
          angle int (Gradian Angle)
          data list List of intensities
     """
     msg = SonarEcho()
     msg.header.stamp = rospy.Time.now()
     msg.header.frame_id = 'sonar_frame'
     msg.angle = angle
     msg.gain = gain
     msg.number_of_samples = numberOfSamples
     msg.transmit_frequency = transmitFrequency
     msg.speed_of_sound = speedOfSound
     msg.range = sonarRange
     msg.intensities = data
     return msg

def publishImage(image, imagePub, bridge):
     try:
          imagePub.publish(bridge.cv2_to_imgmsg(image, "mono8"))
     except CvBridgeError as e:
          rospy.logwarn("Failed to publish sensor image")
          print(e)

def calculateSamplePeriod(distance, numberOfSamples, speedOfSound, _samplePeriodTickDuration = 25e-9):
     # type: (float, int, int, float) -> float
     """
      Calculate the sample period based in the new range
     """
     return 2 * distance / (numberOfSamples * speedOfSound * _samplePeriodTickDuration)

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
     transmit_duration = max(2.5 * getSamplePeriod(samplePeriod) / 1000, duration)
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
     return samplePeriod * _samplePeriodTickDuration

def updateSonarConfig(gain, transmitFrequency, transmitDuration, samplePeriod, numberOfSamples):
     global updated
     p.set_gain_setting(gain)
     p.set_transmit_frequency(transmitFrequency)
     p.set_transmit_duration(transmitDuration)
     p.set_sample_period(samplePeriod)
     p.set_number_of_samples(numberOfSamples)
     updated = False

# Ping 360 Parameters
device = rospy.get_param('~device',"/dev/ttyUSB0")
baudrate = rospy.get_param('~baudrate', 115200)
gain = rospy.get_param('~gain', 0)
numberOfSamples = rospy.get_param('~numberOfSamples', 200) # Number of points
transmitFrequency = rospy.get_param('~transmitFrequency', 740) # Default frequency
sonarRange = rospy.get_param('~sonarRange', 1) # in m
speedOfSound = rospy.get_param('~speedOfSound', 1500) # in m/s

samplePeriod = calculateSamplePeriod(sonarRange, numberOfSamples, speedOfSound)
transmitDuration = adjustTransmitDuration(sonarRange, samplePeriod, speedOfSound)
debug = rospy.get_param('~debug', True)

# Output and ROS parameters
step = rospy.get_param('~step', 1)
imgSize = rospy.get_param('~imgSize', 500)
queue_size= rospy.get_param('~queueSize', 1)

p = Ping360(device, baudrate)
updated = False
print("Initialized: %s" % p.initialize())
