#!/usr/bin/env python

from ping360_sonar.sensor import Ping360
from numpy import pi, sqrt, tan, cos, sign

class SonarInterface:
    
    samplePeriodTickDuration = 25e-9
    firmwareMinTransmitDuration = 5
    firmwareMaxTransmitDuration = 500
    maxDurationRatio = 64e6
    
    def __init__(self, port, baudrate, real_sonar):
        
        try:
            self.sonar = Ping360(port, baudrate)
            if real_sonar and not self.sonar.initialize():
                raise RuntimeError('Cannot instanciate sonar')
        except:
            if real_sonar:
                raise RuntimeError('Cannot initialize sonar')
            print('Using emulated sonar')
            self.sonar = None
            
        self.real_sonar = real_sonar
        
    def configureAngles(self, min_angle, max_angle, step):
        
        if max_angle <= min_angle or (max_angle-min_angle) % step != 0:
            return f'inconsistent angular settings: angular range is [{min_angle} - {max_angle}] while step is {step}'
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.step = step
        self.angle = min_angle
        return ''
    
    @staticmethod
    def grad2rad(grad):
        return grad*pi/200
    
    def minAngle(self):
        return self.grad2rad(self.min_angle)
    def maxAngle(self):
        return self.grad2rad(self.max_angle)
    def angleStep(self):
        return self.grad2rad(self.step)
    def currentAngle(self):
        return self.grad2rad(self.angle)
    def angleCount(self):
        return (self.max_angle-self.min_angle-1)//self.step
    def angleIndex(self):
        return (self.angle-self.min_angle)//self.step
    def rangeFrom(self, index):
        return (index+1)*self.max_range/self.samples
    
    def configureTransducer(self, gain, samples, frequency, speed_of_sound, max_range):
        
        self.gain = gain
        self.samples = samples
        self.frequency = frequency
        self.sample_period = int((2.*max_range)/
                                 (samples*speed_of_sound*SonarInterface.samplePeriodTickDuration));
        
        # TODO re-check this computation
        sample_period_ms = (2.*max_range)/(samples*speed_of_sound*1000)
        one_way_duration = (8000.*max_range)/speed_of_sound
        target_duration = max(2.5*sample_period_ms, one_way_duration)
        max_duration = min(SonarInterface.firmwareMaxTransmitDuration, sample_period_ms*SonarInterface.maxDurationRatio)
        self.transmit_duration = target_duration
        if target_duration > max_duration:
            self.transmit_duration = max_duration
        if target_duration < SonarInterface.firmwareMinTransmitDuration:
            self.transmit_duration = SonarInterface.firmwareMinTransmitDuration
            
    def transmitDuration(self):
        # microseconds to seconds
        return self.transmit_duration/1e6
            
    def read(self):
        # update angle before transmit
        self.angle += self.step
        end_turn = self.angle + self.step == self.max_angle
        if self.angle == self.max_angle:
            self.angle = self.min_angle
        
        if self.real_sonar:            
             self.sonar.control_transducer(
                    0,  # reserved
                    self.gain,
                    self.angle,
                    self.transmit_duration,
                    self.sample_period,
                    self.frequency,
                    self.samples,
                    1,
                    0)
             self.wait_message([definitions.PING360_DEVICE_DATA, definitions.COMMON_NACK], 4.0)
             self.data = bytearray(self.sonar._data)
             return (len(self.data) != 0, end_turn)
        
        # emulated sonar
        from random import randint
        from time import sleep        
        if self.angle % 100 < 100:
            self.data = [randint(0,255) for _ in range(self.samples)]
        else:
            self.data = [0 for _ in range(self.samples)]
        # emulate transmit duration in microseconds
        sleep(self.transmit_duration/1000000)
        return (True, end_turn)



# handles an angular sector of the image
class Bound:
    radius = 0
    def __init__(self, x, tm, tM):
        self.x = x
        if type(tM) == int:
            self.low = Bound.clamp(tm*x)
            self.up = int(tM*sqrt(Bound.radius**2-x**2-1))
        else:
            self.low = Bound.clamp(x*tm)
            self.up = Bound.clamp(x*tM)
            
            if self.up**2 + x**2 > Bound.radius**2:
                self.up = int(sign(self.up) * sqrt(Bound.radius**2-x**2-1))
                
        if self.up < self.low:
            self.low,self.up = self.up,self.low
            
    #staticmethod
    def clamp(coord):
        if coord < -Bound.radius+1:
            return -Bound.radius+1
        elif coord > Bound.radius-1:
            return Bound.radius-1
        return int(coord)
            
class Sector:
    def __init__(self):
        self.dr = None
        
    def configure(self, samples, radius):
        self.dr = radius/samples
        Bound.radius = radius
        
    def init(self, angle, step):
        angle_min = angle-step/2
        angle_max = angle+step/2
        xmin, xmax = self.xLimits(angle_min, angle_max)
        tm, tM = tan(angle_min), tan(angle_max)        
        self.bounds = []

        if xmin * xmax >= 0:
            # same side
            if abs(tm) > abs(tM):
                tm,tM = tM,tm
            for x in range(xmin, xmax+1):
                self.bounds.append(Bound(x,tm,tM))
        else:
            f = 1 if abs(angle-pi/2) < abs(angle-3*pi/2) else -1
            
            if f == -1:
                tm,tM = tM,tm
                
            for x in range(xmin, 0):
                self.bounds.append(Bound(x, tM,f))
            for x in range(0, xmax+1):
                self.bounds.append(Bound(x, tm,f))
                
        self.cur = -1
        
    def xLimits(self, angle_min, angle_max):
        cm = cos(angle_min)
        cM = cos(angle_max)
        if cM < cm:
            cm,cM = cM,cm
        if cm*cM > 0:
            if cM < 0:
                cM = 0
            else:
                cm = 0
        return Bound.clamp(round(Bound.radius*cm)), Bound.clamp(round(Bound.radius*cM))
    
    def nextPoint(self, x, y):
        if self.cur == -1:
            self.cur = 0
            x = self.bounds[0].x
            y = self.bounds[0].low
        elif y < self.bounds[self.cur].up:
            y += 1
        else:
            self.cur += 1
            if self.cur == len(self.bounds):
                return False, 0, 0, 0
            x = self.bounds[self.cur].x
            y = self.bounds[self.cur].low
        return True, x, y, int(round(sqrt(x*x+y*y)/self.dr))
