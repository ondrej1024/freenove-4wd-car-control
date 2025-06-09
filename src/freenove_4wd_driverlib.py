###############################################################################
#
# Freenove 4WD car device driver library
#
# Author: Ondrej Wisniewski
#
# Description:
#   This library implements the drivers for the following devices:
#   - DC Motor
#   - Servo motor
#   - Battery monitor
#   - Neopixel LED strip
#
# Changelog:
#   07-06-2025: Initial version
#
# Copyright 2025
# 
# This file is part of the Freenove 4WD car control platform.
#
###############################################################################

from machine import Pin,PWM,ADC,I2C,Timer
from neopixel import NeoPixel
from time import sleep_ms

# Module version
MODVER = "0.1"

# LED sequences
LED_SEQ_HEARTBEAT = (1,1,1,9)
LED_SEQ_BLINKSLOW = (10,10)
LED_SEQ_BLINKFAST = (2,2)

MOTOR_PWM_FREQ = 500
SERVO_PWM_FREQ = 50
NEO_BASIC_COLORS = ["R","G","B"]


#######################
# MOTOR class
#######################
class MOTOR:
    def __init__(self, in1, in2):
        self._pwm1 = PWM(Pin(in1,Pin.OUT), freq=MOTOR_PWM_FREQ, duty_ns=0)
        self._pwm2 = PWM(Pin(in2,Pin.OUT), freq=MOTOR_PWM_FREQ, duty_ns=0)
        self._min_speed = 40
        self._min_duty_start = (self._min_speed*10000000)//MOTOR_PWM_FREQ

    def move(self,speed):
        # Restrict speed to value from -100% to +100%
        speed = max(min(speed,100),-100)
        # Calculate duty cycle (in ns)
        duty_ns = (abs(speed)*10000000)//MOTOR_PWM_FREQ
        boost_start = (speed != 0 and abs(speed) < self._min_speed)
        print("setting motor speed: %d (duty: %d)" % (speed,duty_ns))
        if speed<0:
            # Move backward
            self._pwm1.duty_ns(0)
            if self._pwm2.duty_ns() == 0 and boost_start:
                # Boost start needed:
                # Apply min speed for 50 ms
                print("apply boost start")
                self._pwm2.duty_ns(self._min_duty_start)
                sleep_ms(50)
            self._pwm2.duty_ns(duty_ns)
        else:
            # Move forward
            self._pwm2.duty_ns(0)
            if self._pwm1.duty_ns() == 0 and boost_start:
                # Boost start needed:
                # Apply min speed for 50 ms
                print("apply boost start")
                self._pwm1.duty_ns(self._min_duty_start)
                sleep_ms(50)
            self._pwm1.duty_ns(duty_ns)

    def stop(self):
        self._pwm1.duty_ns(0)
        self._pwm2.duty_ns(0)


#######################
# SERVO class
#######################
class SERVO:
    def __init__(self, pin):
        self._pwm = PWM(Pin(pin,Pin.OUT), freq=SERVO_PWM_FREQ, duty_ns=1500000)
        self._amin = 30
        self._amax = 150
        
    def position(self,angle):
        # Restrict angle to value from angle_min to angle_max
        angle = max(min(angle,self._amax), self._amin)
        duty_ns = int(((angle/90) + 0.5) * 1000000)
        print("setting servo angle: %d (duty: %d)" % (angle,duty_ns))
        self._pwm.duty_ns(duty_ns)
        
    def move(self,a1,a2):
        if a1>a2:
            step = -1
        else:
            step = 1
        for a in range(a1,a2+step,step):
            self.position(a)
            sleep_ms(10)

    def sweep(self):
        self.move(90,self._amin)
        self.move(self._amin,self._amax)
        self.move(self._amax,90)
        
        
#######################
# BATTERY class
#######################
class BATTERY:
    def __init__(self, pin):
        self._adc = ADC(pin)
        self._ref = 3.3   # PICO ADC reference voltage
        self._scale = 4   # cars voltage divider R5(3K)/R6(1K)
        self._vmax  = 8.4 # cars battery voltage (full)
        self._vmin  = 0   # FIXME: find real min battery voltage
        
    def voltage(self):
        v = (self._adc.read_u16()/0xFFFF) * self._ref * self._scale
        return v
        
    def perc(self):
        # FIXME: consider also vmin
        p = int(round(self.voltage()/self._vmax*100))
        return p
        

#######################
# LEDSTRIP class
#######################
class LEDSTRIP:
    def __init__(self, pin):
        self._neo = NeoPixel(Pin(pin,Pin.OUT),8)

    def allon(self,r,g,b):
        self._neo.fill((r,g,b))
        self._neo.write()
        
    def alloff(self):
        self._neo.fill((0,0,0))
        self._neo.write()
        
    def blink(self,color,brightness=255,num=4):
        if color in NEO_BASIC_COLORS:
            brightness = max(min(brightness,255), 0)
            for _ in range(num):
                if color == "R":
                    self.allon(brightness,0,0)
                elif color == "G":
                    self.allon(0,brightness,0)
                elif color == "B":
                    self.allon(0,0,brightness)
                sleep_ms(200)
                self.alloff()
                sleep_ms(200)
            
    def walk(self,color,brightness=255,clockwise=True):
        if color in NEO_BASIC_COLORS:
            brightness = max(min(brightness,255), 0)
            if clockwise == True:
                first = 0
                last = 8
                step = 1
            else:
                first = 7
                last = -1
                step = -1
            for i in range(first,last,step):
                self._neo.fill((0,0,0))
                if color == "R":
                    self._neo[i] = (brightness,0,0)
                elif color == "G":
                    self._neo[i] = (0,brightness,0)
                elif color == "B":
                    self._neo[i] = (0,0,brightness)
                self._neo.write()
                sleep_ms(200)
            self.alloff()
        
    def fade(self,color,inout):
        if color in NEO_BASIC_COLORS:
            if inout == True:
                first = 0
                last = 256
                step = 1
            else:
                first = 255
                last = -1
                step = -1
            for i in range(first,last,step):
                if color == "R":
                    self.allon(i,0,0)
                elif color == "G":
                    self.allon(0,i,0)
                elif color == "B":
                    self.allon(0,0,i)
                sleep_ms(5)
            
    def breathe(self,color):
        for _ in range(2):
            self.fade(color, True)
            self.fade(color, False)
            

#######################################################
# UPTIME counter class
# (weeks, days, hours, minutes, seconds)
#######################################################
class UPTIME:
    def __init__(self):
        self.cnt = 0
        self.sec = 0
        self.min = 0
        self.hrs = 0
        self.day = 0
        self.week = 0
        self._timer = Timer()
        self._timer.init(period=1000, mode=Timer.PERIODIC, callback=self.__tick)
        
    def __tick(self, t):
        self.cnt += 1
        self.sec  = self.cnt % 60
        self.min  = (self.cnt // 60) % 60
        self.hrs  = (self.cnt // 3600) % 24
        self.day  = (self.cnt // 86400) % 7
        self.week = (self.cnt // 604800)


########################
# LEDCTL class
########################
class LEDCTL:
    def __init__(self,pin):
        self.seq = None
        self.lon = True
        self.idx = 0
        self.cnt = 0
        self.led = Pin(pin, Pin.OUT)
        self.led.off()
        self._timer = Timer()
        self._timer.init(period=100, mode=Timer.PERIODIC, callback=self.__tick)
        
    def __tick(self, t):
        if self.seq:            
            if self.cnt == 0:
                self.led.toggle()
                self.cnt = self.seq[self.idx]
                self.idx = (self.idx+1)%len(self.seq)
            self.cnt-=1
        else:
            if self.lon:
                self.led.on()
            else:
                self.led.off()
        
    def get_seq(self):
        return self.seq
    
    def set_seq(self, seq, lon=False):
        if seq is None:
            self.seq = seq
            self.lon = lon
        elif len(seq) > 0 and len(seq)%2 == 0:
            self.seq = seq
            self.idx = 0
            self.cnt = 0
            self.led.off()
        else:
            print("ERROR: incorrect sequence")

    def heartbeat(self):
        self.set_seq(LED_SEQ_HEARTBEAT)
      
    def blinkslow(self):
        self.set_seq(LED_SEQ_BLINKSLOW)

    def blinkfast(self):
        self.set_seq(LED_SEQ_BLINKFAST)

    def on(self):
        self.set_seq(None,True)
      
    def off(self):
        self.set_seq(None)
