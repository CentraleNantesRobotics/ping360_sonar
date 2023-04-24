#!/usr/bin/env python

from brping import Ping360
from numpy import pi, sqrt, tan, cos, sign
from brping import definitions

class SonarInterface:
    
    samplePeriodTickDuration = 25e-9
    firmwareMinTransmitDuration = 5
    firmwareMaxTransmitDuration = 500
    firmwareMaxSamples = 1200
    firmwareMinSamplePeriod = 80
    maxDurationRatio = 64e6
    
    def __init__(self, port, baudrate, fallback_emulated, connection_type, udp_address, udp_port):
                
        self.angle = 0
        self.sonar = Ping360()
        self.max_range = 0.
        try:
            if connection_type == "serial":
                self.sonar.connect_serial(port, baudrate)
            elif connection_type == "udp":
                self.sonar.connect_udp(udp_address, udp_port)
                
            if self.sonar.initialize():
                return
        except:
            pass
        
        if not fallback_emulated:
            raise RuntimeError('Cannot initialize sonar')
        print('Using emulated sonar')
        self.sonar = None
        
    def configureAngles(self, aperture_deg, step_deg, ensure_divisor):
        # to gradians
        target_half_aperture = int(aperture_deg*200/360+0.5)
        best_half_aperture = target_half_aperture
        self.angle_step = int(round(step_deg*400/360))

        # ensure angle_step is a divisor of max-min in gradians, necessary for LaserScan messages
        if ensure_divisor:            
            # look around step, allow increased aperture
            target_step = self.angle_step
            
            # not too far from requested aperture, as close as possible to requested step (impacts turn duration)
            computeCost = lambda step,half_aperture: 1000 if half_aperture%step != 0 else abs(step-target_step) + abs(half_aperture-target_half_aperture)
            
            best_cost = computeCost(self.angle_step, target_half_aperture)
            if best_cost != 0:                
                for step in range(1, target_step*2):
                    for half_aperture in range(target_half_aperture, min(target_half_aperture+10, 200)+1):
                        cost = computeCost(step, half_aperture)
                        if cost < best_cost:
                            best_cost = cost
                            self.angle_step = step
                            best_half_aperture = half_aperture
                        
        self.angle_min = -best_half_aperture
        self.angle_max = best_half_aperture
        if self.angle_max == 200:                
            self.angle_max -= self.angle_step
        if self.angle < self.angle_min or self.angle > self.angle_max or (self.angle-self.angle_min) % self.angle_step != 0:
            self.angle = 0
    
    @staticmethod
    def grad2rad(grad):
        return grad*pi/200
    
    def angleMin(self):
        return self.grad2rad(self.angle_min)
    def angleMax(self):
        return self.grad2rad(self.angle_max)
    def angleStep(self):
        return self.grad2rad(self.angle_step)
    def currentAngle(self):
        return self.grad2rad(self.angle)
    def angleCount(self):
        return (self.angle_max-self.angle_min)//self.angle_step
    def angleIndex(self):
        if self.angle_step > 0:
            return (self.angle-self.angle_min)//self.angle_step
        return (self.angle-self.angle_max)//self.angle_step
    def rangeFrom(self, index):
        return (index+1)*self.max_range/self.samples
    def fullScan(self):
        return self.angle_min == -200
    
    def configureTransducer(self, gain, frequency, speed_of_sound, max_range):
        
        self.max_range = max_range
        self.gain = gain
        self.frequency = frequency
        
        self.samples = int(min(self.firmwareMaxSamples,2*max_range/(self.firmwareMinSamplePeriod*speed_of_sound*self.samplePeriodTickDuration)))
        
        self.sample_period = int((2.*max_range)/
                                 (self.samples*speed_of_sound*self.samplePeriodTickDuration));
        

        #* Per firmware engineer:
        #* 1. Starting point is TxPulse in usec = ((one-way range in metres) * 8000) / (Velocity of sound in metres
        #* per second)
        #* 2. Then check that TxPulse is wide enough for currently selected sample interval in usec, i.e.,
        #*    if TxPulse < (2.5 * sample interval) then TxPulse = (2.5 * sample interval)
        #* 3. Perform limit checking

        #1
        one_way_duration_us = (8000.*max_range)/speed_of_sound
        # 2 (transmit duration is microseconds, sample_period_ns is nanoseconds)        
        sample_period_ns = self.sample_period * self.samplePeriodTickDuration
        self.transmit_duration = max(2.5*sample_period_ns/1000, one_way_duration_us)
        # 3 ensure bounds        
        if self.transmit_duration < self.firmwareMinTransmitDuration:
            self.transmit_duration = self.firmwareMinTransmitDuration
        else:
            max_duration = min(self.firmwareMaxTransmitDuration, sample_period_ns*self.maxDurationRatio)
            if self.transmit_duration > max_duration:
                self.transmit_duration = max_duration
        self.transmit_duration = int(self.transmit_duration)
            
    def transmitDuration(self):
        # microseconds to seconds
        return self.transmit_duration/1e6
    
    def updateAngle(self):
        self.angle += self.angle_step
        
        if self.angle_min == -200:
            # full scan
            end_turn = self.angle + self.angle_step > self.angle_max
            if self.angle > self.angle_max:
                self.angle = self.angle_min
            return end_turn
        
        # sector scan, check near end of sector
        if self.angle + self.angle_step >= self.angle_max or self.angle + self.angle_step <= self.angle_min:
            self.angle_step *= -1
            return True
        return False
            
    def read(self):
        # update angle before transmit
        end_turn = self.updateAngle()
        
        if self.sonar is not None:
            print(f'transmit: {self.transmit_duration}')
            
            self.sonar.control_transducer(
                    0,  # reserved
                    self.gain,
                    self.angle if self.angle >= 0 else self.angle + 400,
                    self.transmit_duration,
                    self.sample_period,
                    self.frequency,
                    self.samples,
                    1,
                    0)
            self.sonar.wait_message([definitions.PING360_DEVICE_DATA, definitions.COMMON_NACK], 4.0)
            self.data = bytearray(self.sonar._data)
            return (len(self.data) != 0, end_turn)
        
        # emulated sonar
        from random import randint
        from time import sleep    
        self.data = [0 for _ in range(self.samples)]
        scale = 5*abs((self.angle+400) % 400 - 200)
        for i in range(self.samples):
            if randint(self.samples,2*self.samples) < 1.1*i + scale:
                self.data[i] = randint(220, 255)
        # emulate transmit duration in microseconds
        #sleep(self.transmit_duration/1000000)
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
        xmin, xmax,same_side = self.xLimits(angle_min, angle_max)
        tm, tM = tan(angle_min), tan(angle_max)        
        self.bounds = []

        if same_side:
            # same side
            if abs(tm) > abs(tM):
                tm,tM = tM,tm
            for x in range(xmin, xmax+1):
                self.bounds.append(Bound(x,tm,tM))
        else:
            f = 1 if abs(angle-pi/2) < abs(angle+pi/2) else -1
            
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
        return Bound.clamp(round(Bound.radius*cm)), Bound.clamp(round(Bound.radius*cM)), cm*cM >= 0
    
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
