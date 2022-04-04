#!/usr/bin/env python

# ROS 2 version of Python node
from math import cos, pi, sin

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from ping360_sonar_msgs.msg import SonarEcho
from ping360_sonar.sensor import Ping360


class Ping360_node(Node):

    def __init__(self):
        super().__init__('ping360_node')
        # declare parameters

        self.create_timer(0.01, self.timer_callback)  # 100Hz
        self.ParametersInteger = {
            'baudrate': 115200,
            'gain': 0,
            'transmitFrequency': 740,
            'queue_size': 1,
            'maxAngle': 400,
            'minAngle': 0,
            'threshold': 200,
            'numberOfSamples': 200,
            'step': 1,
            'imgSize': 500,
            'speedOfSound': 1500,
            'sonarRange': 1,
        }
        self.ParametersDouble = {
        }
        self.ParametersString = {
            'device': "/dev/ttyUSB0",
            'frameID': "somar_frame"
        }
        self.ParametersBool = {
            'debug': False,
            'enableImageTopic': True,
            'enableScanTopic': True,
            'enableDataTopic': True,
            'oscillate': True,
        }

        for param in self.ParametersInteger.keys():
            exec(
                f'self._{param} = self.declare_parameter(\"{param}\", {self.ParametersInteger[param]}).get_parameter_value().integer_value')

        for param in self.ParametersString.keys():
            exec(
                f'self._{param} = self.declare_parameter(\"{param}\", \"{self.ParametersString[param]}\").get_parameter_value().string_value')

        for param in self.ParametersDouble.keys():
            exec(
                f'self._{param} = self.declare_parameter(\"{param}\", {self.ParametersDouble[param]}).get_parameter_value().double_value')

        for param in self.ParametersBool.keys():
            exec(
                f'self._{param} = self.declare_parameter(\"{param}\", {self.ParametersBool[param]}).get_parameter_value().bool_value')

        # get parameters
        self._samplePeriod = self.calculateSamplePeriod()
        self._transmitDuration = self.adjustTransmitDuration()
        self._FOV = self._maxAngle - self._minAngle
        self._sign = 1
        self._angle = self._minAngle

        if self._FOV <= 0:
            self.get_logger().info(
                f"minAngle should be inferior to the maxAngle! Current settings: minAngle: {self._minAngle} - maxAngle: {self._maxAngle}")
            rclpy.shutdown()

        if self._step >= self._FOV:
            self.get_logger().info(
                f"The configured step is bigger then the set FOV (maxAngle - minAngle). Current settings: step: {self._step} - minAngle: {self._minAngle} - maxAngle: {self._maxAngle} - FOV: {self._FOV}")
            rclpy.shutdown()

        self._sensor = Ping360(self._device, self._baudrate)
        self.get_logger().info(f'Initialized Sensor: {self._sensor.initialize()}')
        self._updated = True
        self._ranges = [0]
        self._intensities = [0]

        # self.srv = Server(sonarConfig, callback) // never used in the original python file

        self._bridge = CvBridge()
        self._center = (float(self._imgSize / 2), float(self._imgSize / 2))

        self._imagePub = self.create_publisher(Image, "/ping360_images", self._queue_size)
        self._rawPub = self.create_publisher(SonarEcho, "/ping360_data", self._queue_size)
        self._laserPub = self.create_publisher(LaserScan, "/ping360_scan", self._queue_size)

        self.add_on_set_parameters_callback(self.cb_params)

    def cb_params(self, params):

        result = True
        reason = ""
        for param in params:
            self.get_logger().warn(f'Request: {param.name} -> {param.value}')
            if param.name in self.ParametersDouble:
                if param.type_ != Parameter.Type.DOUBLE:
                    self.get_logger().warn("\n"*(len(reason) != 0) + f'Type is not correct for parameter {param.name}, expected double')
                    reason += "\n"*(len(reason) != 0) + f"wrong data type for {param.name}, expected double"
                    result = False

            elif param.name in self.ParametersString:
                if param.type_ != Parameter.Type.STRING:
                    self.get_logger().warn("\n"*(len(reason) != 0) + f'Type is not correct for parameter {param.name}, expected string')
                    reason += "\n"*(len(reason) != 0) + f"wrong data type for {param.name}, expected string"
                    result = False

            elif param.name in self.ParametersBool:
                if param.type_ != Parameter.Type.BOOL:
                    self.get_logger().warn("\n"*(len(reason) != 0) + f'Type is not correct for parameter {param.name}, expected bool')
                    reason += "\n"*(len(reason) != 0) + f"wrong data type for {param.name}, expected bool"
                    result = False

            elif param.name in self.ParametersInteger:
                if param.type_ != Parameter.Type.INTEGER:
                    self.get_logger().warn("\n"*(len(reason) != 0) + f'Type is not correct for parameter {param.name}, expected int')
                    reason += "\n"*(len(reason) != 0) + f"wrong data type for {param.name}, expected int"
                    result = False

            else:
                result = False
                reason += "\n"*(len(reason) != 0) + f"parameter {param.name} unexpected"

        if result:
            for param in params:
                exec(f'self._{param.name} = param.value')

        return SetParametersResult(successful=result, reason=reason)

    def getSonarData(self):
        """
        Transmits the sonar angle and returns the sonar intensities
        Args:
            self: Ping360_node class
        Returns:
            list: Intensities from 0 to 255
        """
        self._sensor.transmitAngle(self._angle)
        data = bytearray(getattr(self._sensor, '_data'))
        return [k for k in data]

    def generateRawMsg(self, data):
        """
        Generates the raw message for the data topic
        Args:
            self: Ping360_node class
            data (list): List of intensities
        Returns:
            SonarEcho: message
        """
        msg = SonarEcho()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frameID
        msg.angle = float(self._angle)
        msg.gain = self._gain
        msg.number_of_samples = self._numberOfSamples
        msg.transmit_frequency = self._transmitFrequency
        msg.speed_of_sound = self._speedOfSound
        msg.range = self._sonarRange
        msg.intensities = data
        return msg

    def generateScanMsg(self):
        """
        Generates the laserScan message for the scan topic
        Args:
            self: Ping360_node class
        Returns:
            LaserScan: message
        """
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frameID
        msg.angle_min = 2 * pi * self._minAngle / 400
        msg.angle_max = 2 * pi * self._maxAngle / 400
        msg.angle_increment = 2 * pi * self._step / 400
        msg.time_increment = 0.
        msg.range_min = .75
        msg.range_max = float(self._sonarRange)
        msg.ranges = [float(k) for k in self._ranges]
        msg.intensities = [float(k) for k in self._intensities]

        return msg

    def publishImage(self, image):
        try:
            self._imagePub.publish(self._bridge.cv2_to_imgmsg(image, "mono8"))
        except CvBridgeError as e:
            self.get_logger().info("Failed to publish sensor image")
            print(e)

    def calculateRange(self, index, _samplePeriodTickDuration=25e-9):
        """
        Calculate the range based in the duration
        """
        return index * self._speedOfSound * _samplePeriodTickDuration * self._samplePeriod / 2

    def calculateSamplePeriod(self, _samplePeriodTickDuration=25e-9):
        """
        Calculate the sample period based in the new range
        """
        return int(2 * self._sonarRange / (self._numberOfSamples * self._speedOfSound * _samplePeriodTickDuration))

    def adjustTransmitDuration(self, _firmwareMinTransmitDuration=5):
        # type: (Ping360_node, float) -> float
        """
        @brief Adjust the transmit duration for a specific range
        Per firmware engineer:
        1. Starting point is TxPulse in usec = ((one-way range in metres) * 8000) / (Velocity of sound in metres
        per second)
        2. Then check that TxPulse is wide enough for currently selected sample interval in usec, i.e.,
            if TxPulse < (2.5 * sample interval) then TxPulse = (2.5 * sample interval)
            (transmit duration is microseconds, samplePeriod() is nanoseconds)
        3. Perform limit checking
        Returns:
            float: Transmit duration
        """
        duration = 8000 * self._sonarRange / self._speedOfSound
        transmit_duration = max(
            2.5 * self.getSamplePeriod() / 1000, duration)
        return int(max(_firmwareMinTransmitDuration, min(self.transmitDurationMax(), transmit_duration)))

    def transmitDurationMax(self, _firmwareMaxTransmitDuration=500):
        # type: (Ping360_node, float) -> float
        """
        @brief The maximum transmit duration that will be applied is limited internally by the
        firmware to prevent damage to the hardware
        The maximum transmit duration is equal to 64 * the sample period in microseconds

        Returns:
            float: The maximum transmit duration possible
        """
        return min(_firmwareMaxTransmitDuration, self.getSamplePeriod() * 64e6)

    def getSamplePeriod(self, _samplePeriodTickDuration=25e-9):
        # type: (Ping360_node, float) -> float
        """  Sample period in ns """
        return int(self._samplePeriod * _samplePeriodTickDuration)

    def updateSonarConfig(self):
        self._sensor.set_gain_setting(self._gain)
        self._sensor.set_transmit_frequency(self._transmitFrequency)
        self._sensor.set_transmit_duration(self._transmitDuration)
        self._sensor.set_sample_period(self._samplePeriod)
        self._sensor.set_number_of_samples(self._numberOfSamples)
        self._updated = False

    def dataSelection(self, data):
        for detectedIntensity in data:
            if detectedIntensity >= self._threshold:
                detectedIndex = data.index(detectedIntensity)
                # The index+1 represents the number of samples which then can be used to deduce the range
                distance = self.calculateRange(1 + detectedIndex)
                if 0.75 <= distance <= self._sonarRange:
                    self._ranges[0] = distance
                    self._intensities[0] = detectedIntensity
                    if self._debug:
                        print(f"Object at {self._angle} grad : {self._ranges[0]}m - {float(self._intensities[0] * 100 / 255)}%")
                    break

    def buildImage(self, data):
        image = np.zeros((self._imgSize, self._imgSize, 1), np.uint8)
        linear_factor = float(len(data)) / float(self._center[0])
        try:
            for i in range(int(self._center[0])):
                if i < self._center[0]:
                    pointColor = data[int(i * linear_factor - 1)]
                else:
                    pointColor = 0
                for k in np.linspace(0, self._step, 8 * self._step):
                    theta = 2 * pi * (self._angle + k) / 400.0
                    x = float(i) * cos(theta)
                    y = float(i) * sin(theta)
                    image[int(self._center[0] + x)][int(self._center[1] + y)][0] = pointColor
        except IndexError:
            self.get_logger().info(
                "IndexError: data response was empty, skipping this iteration..")
            pass
        return image

    def updateAngle(self):
        self._angle += self._sign * self._step
        if self._angle >= self._maxAngle:
            if not self._oscillate:
                self._angle = self._minAngle
            else:
                self._angle = self._maxAngle
                self._sign = -1

        if self._angle <= self._minAngle and self._oscillate:
            self._sign = 1
            self._angle = self._minAngle

    def timer_callback(self):

        if self._debug:
            self.get_logger().info('running')
            # self.get_logger().info(f'{self._samplePeriod}')

        if self._updated:
            self.updateSonarConfig()

        data = self.getSonarData()

        if self._enableDataTopic:
            rawDataMsg = self.generateRawMsg(data)
            self._rawPub.publish(rawDataMsg)

        if self._enableScanTopic:
            # Get the first high intensity value
            self.dataSelection(data)
            # Construct and publish Sonar scan msg
            scanDataMsg = self.generateScanMsg()
            self._laserPub.publish(scanDataMsg)

        if self._enableImageTopic:
            image = self.buildImage(data)
            self.publishImage(image)

        self.updateAngle()


def main():
    try:
        rclpy.init()
    except:
        pass
    node = Ping360_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
