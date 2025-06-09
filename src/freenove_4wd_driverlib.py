###############################################################################
#
# Freenove 4WD car driver library
#
# Author: Ondrej Wisniewski
#
# Description:
#   This library implements the drivers for the following devices:
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
from time import sleep_ms

# Module version
MODVER = "0.1"

# LED sequences
LED_SEQ_HEARTBEAT = (1,1,1,9)
LED_SEQ_BLINKSLOW = (10,10)
LED_SEQ_BLINKFAST = (2,2)

MOTOR_PWM_FREQ = 500


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
        print("setting speed: %d (duty: %d)" % (speed,duty_ns))
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
