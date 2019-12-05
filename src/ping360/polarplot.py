import roslib
roslib.load_manifest('ros-ping360-sonar')

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
     parser.add_argument('--size', action="store", type=int, default=200, help="Length")

     args = parser.parse_args()
     try:
          rospy.init_node('ping360_node')
          p = Ping360(args.device, args.baudrate)
          bridge = CvBridge()
          imagePub = rospy.Publisher('ping360_sonar', Image, queue_size=1)
          print("Initialized: %s" % p.initialize())
          if args.v:
               print('Verbose Mode')
          print(p.set_transmit_frequency(1000))
          print(p.set_sample_period(80))
          print(p.set_number_of_samples(200))

          max_range = 80*200*1450/2
          step = 1
          length = args.size
          image = np.zeros((length, length, 1), np.uint8)
          angle = 0
          center = (length/2,length/2)
          while not rospy.is_shutdown():
               p.transmitAngle(angle)
               data = bytearray(getattr(p,'_data'))
               data_lst = [k for k in data]
               linear_factor = len(data_lst)/center[0]
               for i in range(int(center[0])):
                    if(i < center[0]*max_range/max_range):
                         pointColor = data_lst[int(i*linear_factor-1)]
                    else:
                         pointColor = 0
                    for k in np.linspace(0,step,8*step):
                         image[int(center[0]+i*cos(2*pi*(angle+k)/400)), int(center[1]+i*sin(2*pi*(angle+k)/400)), 0] = pointColor
               angle = (angle + step) % 400
               color = cv2.applyColorMap(image,cv2.COLORMAP_JET)
               image = color
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
          imagePub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
     except CvBridgeError as e:
          print(e)