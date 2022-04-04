#!/usr/bin/env python

# ROS 2 version of Python node
from math import cos, pi, sin

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, IntegerRange
from rclpy.parameter import Parameter

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from ping360_sonar_msgs.msg import SonarEcho
from ping360_sonar.sonar_interface import SonarInterface, Sector

class Ping360_node(Node):
        
    def __init__(self):
        super().__init__('ping360')

        # parameters to be declared, will be parsed later
        # several values mean [default,lower, upper, [step]]
        parameters = {
            'gain': [0,0,2],
            'frequency': 740,
            'max_angle': [400,200,400],
            'min_angle': [0,0,200],
            'scan_threshold': [200,0,255],
            'samples': [200,10,1000],
            'angle_step': [1,1,15],
            'image_size': [500,200,1000],
            'image_rate': [100, 50, 2000],
            'speed_of_sound': [1500,1000,2000],
            'range': [2,1,50],
            'publish_image': True,
            'publish_scan': False,
            'publish_echo': False}
        
        for name, value in parameters.items():
            if type(value) not in (list, tuple):
                self.declare_parameter(name, value)
            else:
                if len(value) == 3:
                    default, low,up = value
                    step = 1
                else:
                    default, low,up,step = value
                descriptor = ParameterDescriptor(
                    name=name,
                    integer_range = [IntegerRange(from_value=low,
                                                  to_value=up,
                                                  step=step)])
                self.declare_parameter(name,default,descriptor)
                
        # init sonar interface
        self.sonar = SonarInterface(self.declare_parameter('device', '/dev/ttyUSB0').value,
                                    self.declare_parameter("baudrate", 115200).value,
                                    self.declare_parameter('real_sonar', False).value)
                
        self.image_pub = None
        self.scan_pub = None
        self.echo_pub = None
                
        # init messages
        self.sector = Sector()
        frame = self.declare_parameter("frame", "sonar").value
        self.image = Image()
        self.image.header.frame_id = frame
        self.image.encoding = 'mono8'
        self.image.is_bigendian = 0
        self.scan = LaserScan()
        self.scan.header.frame_id = frame
        self.scan.range_min = 0.75
        self.echo = SonarEcho()
        self.echo.header.frame_id = frame
        
        # configure from given params
        reason = self.configureFromParams()
        if len(reason):
            self.get_logger().info(reason)
            raise(RuntimeError(reason))        
        
        self.add_on_set_parameters_callback(self.cb_params)
        
        self.image_timer = self.create_timer(self.get_parameter('image_rate').value/1000, self.publishImage)


    def init_publishers(self, image, scan, echo):
        
        self.publish_image = image
        self.publish_scan = scan
        self.publish_echo = echo
        
        if image and self.image_pub is None:
            self.image_pub = self.create_publisher(Image, "scan_image", 1)
            
        if scan and self.scan_pub is None:
            self.scan_pub = self.create_publisher(LaserScan, "scan", 1)
            
        if echo and self.echo_pub is None:
            self.echo_pub = self.create_publisher(SonarEcho, "scan_echo", 1)
    
    def configureFromParams(self, changes = []):
        
        # get current params
        params = self.get_parameters(["gain","frequency","range",
                                   "min_angle","max_angle","angle_step",
                                   "speed_of_sound","samples","image_size", "scan_threshold",
                                    "publish_image","publish_scan","publish_echo"])
        params = dict((param.name, param.value) for param in params)        
        # override with requested changes, if any
        params.update(dict((param.name, param.value) for param in changes))
        
        # start with this as it may be invalid
        msg = self.sonar.configureAngles(params['min_angle'],params['max_angle'],params['angle_step'])
        if len(msg):
            return msg
        
        self.init_publishers(params['publish_image'],
                             params['publish_scan'],
                             params['publish_echo'])
        
        self.sonar.configureTransducer(params["gain"],
                                       params["samples"],
                                       params["frequency"],
                                       params["speed_of_sound"],
                                       params["range"])
        self.echo.gain = params["gain"]
        self.echo.range = params["range"]
        self.echo.speed_of_sound = params["speed_of_sound"]
        self.echo.number_of_samples = params["samples"]
        self.echo.transmit_frequency = params["frequency"]

        self.scan.angle_min = self.sonar.minAngle()
        self.scan.angle_increment = self.sonar.angleStep()
        self.scan.angle_max = self.sonar.maxAngle() - self.scan.angle_increment
        self.scan.range_max = float(params["range"])
        self.scan.time_increment = self.sonar.transmitDuration()

        size = params['image_size']
        if size != self.image.step:
            self.image.step = self.image.width = self.image.height = size
            self.image.data = [0 for _ in range(size*size)]

        self.sector.configure(params["samples"], size//2)
        self.scan_threshold = params["scan_threshold"]
        
        # no error
        return ''
    
    def now(self):
        return self.get_clock().now().to_msg()

    def cb_params(self, params):
        reason = self.configureFromParams(params)
        return SetParametersResult(successful=len(reason) == 0, reason=reason)
    
    def refresh(self):
        
        valid, end_turn = self.sonar.read()
        
        if not valid:
            return
        
        if self.publish_echo:
            self.publishEcho()

        if self.publish_image:            
            self.refreshImage()

        if self.publish_scan:
            self.publishScan(end_turn)                

    def publishEcho(self):
        # type: (Ping360_node) -> None
        """
        Publishes the last raw echo message
        """
        self.echo.angle = self.sonar.currentAngle()
        self.echo.intensities = self.sonar.data
        self.echo.header.stamp = self.now()
        self.echo_pub.publish(self.echo)

    def publishScan(self, end_turn):
        """
        Updates the laserScan message for the scan topic
        Actually publishes only after the end of each turn
        """
        count = self.sonar.angleCount()   
        cur = len(self.scan.ranges)
        for _ in range(count - cur):
            self.scan.ranges.append(0.)
            self.scan.intensities.append(0.)
        if cur > count:
            self.scan.ranges = self.scan.ranges[:count]
            self.scan.intensities = self.scan.intensities[:count]
            
        cur = self.sonar.angleIndex()
        for i in range(len(self.sonar.data)):
            if self.sonar.data[i] >= self.scan_threshold:
                dist = self.sonar.rangeFrom(i)
                if self.scan.range_min <= dist <= self.scan.range_max:
                    self.scan.ranges = dist
                    self.scan.intensities = self.sonar.data[i]/255.
                    break
        
        if end_turn:
            self.scan.header.stamp = self.now()
            self.scan_pub.publish(self.scan)
    
    def refreshImage(self):
        
        half_size = self.image.step//2
        self.sector.init(self.sonar.currentAngle(), self.sonar.angleStep())
        length = len(self.sonar.data)
        x = 0
        y = 0
        while True:
            more_points, x, y, index = self.sector.nextPoint(x, y)
            
            if index < length:
                self.image.data[half_size-y + self.image.step*(half_size-x)] = self.sonar.data[index]
            
            if not more_points:
                break

    def publishImage(self):
        if self.publish_image:
            self.image.header.stamp = self.now()
            self.image_pub.publish(self.image)

if __name__ == "__main__":
    
    rclpy.init()    
    node = Ping360_node()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    while rclpy.ok():
        node.refresh()
        executor.spin_once()
        
    
    rclpy.shutdown()
