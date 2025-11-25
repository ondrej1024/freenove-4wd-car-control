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
#   - Ultrasonic sensor
#   - Track sensor
#   - Light sensor
#   - Buzzer
#   - Pico Led
#
# Changelog:
#   07-06-2025: Initial version
#
# Copyright 2025
# 
# This file is part of the Freenove 4WD car control platform.
#
###############################################################################

import machine
from machine import Pin,PWM,ADC,I2C,Timer
from neopixel import NeoPixel
from time import sleep_us,sleep_ms,ticks_us,ticks_diff

# Module version
MODVER = "0.1"

# LED sequences
LED_SEQ_HEARTBEAT = (1,1,1,9)
LED_SEQ_BLINKSLOW = (10,10)
LED_SEQ_BLINKFAST = (2,2)

MOTOR_PWM_FREQ   = 500  # in Hz
SERVO_PWM_FREQ   = 50   # in Hz
BUZZER_PWM_FREQ  = 2000 # in Hz
SPEED_OF_SOUND   = 343  # in m/s
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
                # Apply min speed for 100 ms
                print("apply boost start backward")
                self._pwm2.duty_ns(self._min_duty_start)
                sleep_ms(100)
            self._pwm2.duty_ns(duty_ns)
        else:
            # Move forward
            self._pwm2.duty_ns(0)
            if self._pwm1.duty_ns() == 0 and boost_start:
                # Boost start needed:
                # Apply min speed for 100 ms
                print("apply boost start forward")
                self._pwm1.duty_ns(self._min_duty_start)
                sleep_ms(100)
            self._pwm1.duty_ns(duty_ns)

    def stop(self):
        self._pwm1.duty_ns(0)
        self._pwm2.duty_ns(0)

#######################
# CAR class
#######################
class CAR:
    def __init__(self,m1,m2,m3,m4):
        self._m1 = m1
        self._m2 = m2
        self._m3 = m3
        self._m4 = m4
        
    def forward(self,speed):
        speed = abs(speed)
        # Front wheels
        self._m1.move(speed)
        self._m4.move(speed)
        # Back wheels
        self._m2.move(speed)
        self._m3.move(speed)

    def backward(self,speed):
        speed = -abs(speed)
        # Front wheels
        self._m1.move(speed)
        self._m4.move(speed)
        # Back wheels
        self._m2.move(speed)
        self._m3.move(speed)
    
    def spin_right(self):
        speed = 30
        # Left wheels
        self._m1.move(speed)
        self._m2.move(speed)
        # Right wheels
        self._m3.move(-speed)
        self._m4.move(-speed)
        
    def spin_left(self):
        speed = 30
        # Left wheels
        self._m1.move(-speed)
        self._m2.move(-speed)
        # Right wheels
        self._m3.move(speed)
        self._m4.move(speed)

    def turn_right(self,speed):
        # Left wheels
        self._m1.move(speed)
        self._m2.move(speed)
        # Right wheels
        self._m3.move(0)
        self._m4.move(0)
        
    def turn_left(self,speed):
        # Left wheels
        self._m1.move(0)
        self._m2.move(0)
        # Right wheels
        self._m3.move(speed)
        self._m4.move(speed)
        
    def stop(self):
        self._m1.stop()
        self._m2.stop()
        self._m3.stop()
        self._m4.stop()


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
        #print("setting servo angle: %d (duty: %d)" % (angle,duty_ns))
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
# ULTRASONIC sensor class
#######################################################
class ULTRASONIC:
    def __init__(self, pin_trg, pin_echo, car=None, buzzer=None):
        self._d = None
        self._is_alarm = False
        self._start_time = 0
        self._car = car
        self._buzzer = buzzer
        self._pin_trig = Pin(pin_trg, Pin.OUT)
        self._pin_trig.off()
        self._pin_echo = Pin(pin_echo, Pin.IN)
        self._pin_echo.irq(handler=self.__irq_pin, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self._timer = Timer()
        self._timer.init(period=200, mode=Timer.PERIODIC, callback=self.__irq_timer)
        self._d_scale = SPEED_OF_SOUND / 2 / 10000
        self._d_min = 10 # cm
        self._t_max = 70000 # us

    def __check_dist(self,d_cm):
        # Check for min distance
        if d_cm < self._d_min:
            if not self._is_alarm:
                print("distance is too short !!!")
                self._is_alarm = True
                if self._car is not None:
                    self._car.stop()
                if self._buzzer is not None:
                    self._buzzer.alarm()
        else:
            self._is_alarm = False

    def __irq_timer(self,t):
        # This is called periodically
        # Generate 10us high pulse on trigger output
        #print("   trigger pin pulse")
        self._pin_trig.on()
        sleep_us(10)
        self._pin_trig.off()
        
    def __irq_pin(self,pin):
        # This is called when echo input pin is triggered
        # Measure high pulse length in us on echo input
        if pin.value() == 1:
            # Start time measurement
            #print("   echo ping high")
            self._start_time = ticks_us()
        else:
            # Finish time measurement
            #print("   echo ping low")
            t_us = ticks_diff(ticks_us(),self._start_time)
            #print("   t=%d us" % t_us)
            self._start_time = 0
            if t_us >= 0 and t_us < self._t_max:
                # Calculate distance in cm
                d_cm = round(t_us * self._d_scale,1)
                self.__check_dist(d_cm)
            else:
                # Invalid measurement
                d_cm = None
            self._d = d_cm

    def distance(self):
        return (self._d)


#######################################################
# TRACK sensor class
#######################################################
class TRACK:
    def __init__(self, pin_trk1, pin_trk2, pin_trk3, car=None, buzzer=None):
        self._car = car
        self._buzzer = buzzer
        self._pin_trk1 = Pin(pin_trk1, Pin.IN)
        self._pin_trk2 = Pin(pin_trk2, Pin.IN)
        self._pin_trk3 = Pin(pin_trk3, Pin.IN)
        self._pin_trk1.irq(handler=self.__irq1, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self._pin_trk2.irq(handler=self.__irq2, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self._pin_trk3.irq(handler=self.__irq3, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self._val_left   = self._pin_trk1.value()
        self._val_center = self._pin_trk1.value()
        self._val_right  = self._pin_trk1.value()

    def __edge_detect(self):
        if (self._val_left == 1 and self._val_center == 1 and self._val_right == 1):
            print("edge detected !!!")
            if self._car is not None:
                self._car.stop()
            if self._buzzer is not None:
                self._buzzer.alarm()

    def __irq1(self, pin):
        print("track sensor left change")
        self._val_left = pin.value()
        self.__edge_detect()

    def __irq2(self, pin):
        print("track sensor center change")
        self._val_center = pin.value()
        self.__edge_detect()

    def __irq3(self, pin):
        print("track sensor right change")
        self._val_right = pin.value()
        self.__edge_detect()

    def status(self):
        return (self._val_left, self._val_center, self._val_right)


#######################################################
# LIGHT sensor class
#######################################################
class LIGHT:
    def __init__(self, pin_photo1, pin_photo2):
        self._ain1 = ADC(pin_photo1)
        self._ain2 = ADC(pin_photo2)
        self._light_level_left = None
        self._light_level_right = None
        self._timer = Timer()
        self._timer.init(period=1000, mode=Timer.PERIODIC, callback=self.__tick)

    def __measure(self, adc):
        v = adc.read_u16()*100/0xFFFF # in %
        return int(round(v))

    def __tick(self, t):
        #print("check light")
        self._light_level_left = self.__measure(self._ain1)
        self._light_level_right = self.__measure(self._ain2)

    def level(self):
        return (self._light_level_left, self._light_level_right)


#######################
# BUZZER class
#######################
class BUZZER:
    def __init__(self, pin):
        # We cannot use HW PWM for the buzzer as its pin (2) overlaps with
        # the PWM channel of a motor pin (18)
        self._pin = Pin(pin,Pin.OUT)

    def _softpwm(self,freq,duty=50,duration=100):
        pulse_len = 1000000//freq # in us
        pulse_len_hi = (pulse_len*duty)//100
        pulse_len_lo = pulse_len - pulse_len_hi
        cycles = (duration*1000)//pulse_len
        for _ in range(cycles):
            # Generate pulse
            self._pin.on()
            sleep_us(pulse_len_hi)
            self._pin.off()
            sleep_us(pulse_len_lo)

    def beep(self, duration=100):
        self._softpwm(BUZZER_PWM_FREQ,10,duration)
        
    def alarm(self, repeat=3):
        for _ in range(repeat):
            self.beep()
            sleep_ms(100)


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
