###############################################################################
#
# Freenove 4WD car hardware related definitions and functions
#
# Author: Ondrej Wisniewski
#
# Description:
#
# Changelog:
#   07-06-2025: Initial version
#
# Copyright 2025
# 
# This file is part of the Freenove 4WD car control platform.
# 
###############################################################################

from machine import RTC,Pin,I2C,PWM,UART,ADC
from freenove_4wd_driverlib import MOTOR,CAR,SERVO,BATTERY,LEDSTRIP,ULTRASONIC,BUZZER,TRACK,LIGHT,UPTIME,LEDCTL

# Module version
MODVER = "0.1"

# HW Constants
PIN_M1_IN1   = const(18)    # Motor 1 (front left)  IN1
PIN_M1_IN2   = const(19)    # Motor 1 (front left)  IN2
PIN_M2_IN1   = const(21)    # Motor 2 (rear left)   IN1
PIN_M2_IN2   = const(20)    # Motor 2 (rear left)   IN2
PIN_M3_IN1   = const(7)     # Motor 3 (rear right)  IN1
PIN_M3_IN2   = const(6)     # Motor 3 (rear right)  IN2
PIN_M4_IN1   = const(9)     # Motor 4 (front right) IN1
PIN_M4_IN2   = const(8)     # Motor 4 (front right) IN2

PIN_TRK1     = const(10)    # Track detector 1
PIN_TRK2     = const(11)    # Track detector 2
PIN_TRK3     = const(12)    # Track detector 3

PIN_SRV1     = const(13)    # Servo 1
PIN_SRV2     = const(14)    # Servo 2
PIN_SRV3     = const(15)    # Servo 3

PIN_UART0_TX = const( 0)    # Serial TX
PIN_UART0_RX = const( 1)    # Serial RX

PIN_BUZZER   = const( 2)    # Buzzer
PIN_IR_RX    = const( 3)    # IR receiver
PIN_NEOPIX   = const(16)    # Neopixel LED strip
PIN_LED_PICO = const("LED") # Pico LED

PIN_I2C0_SDA = const( 4)    # I2C SDA
PIN_I2C0_SCL = const( 5)    # I2C CLK

PIN_US_TRIG  = const( 4)    # Ultasonic trigger
PIN_US_ECHO  = const( 5)    # Ultasonic echo

PIN_ADC_BATT = const(26)    # Battery voltage
PIN_ADC_PR1  = const(27)    # Photoresistor 1
PIN_ADC_PR2  = const(28)    # Photoresistor 2



__i2c = None


#######################
# Init I2C bus
#######################
def __init_i2c():
    global __i2c
    if __i2c == None:
        print("Init I2C bus")
        __i2c = I2C(0, scl=Pin(GP_I2C0_SCL), sda=Pin(GP_I2C0_SDA))
    return __i2c


#######################
# Init DC motors
#######################
def init_motors():
    motors = []
    print("Init Motors")
    motors.append(MOTOR(PIN_M1_IN1,PIN_M1_IN2))
    motors.append(MOTOR(PIN_M2_IN1,PIN_M2_IN2))
    motors.append(MOTOR(PIN_M3_IN1,PIN_M3_IN2))
    motors.append(MOTOR(PIN_M4_IN1,PIN_M4_IN2))
    return motors


#######################
# Init car
#######################
def init_car():
    print("Init Car")
    m1 = MOTOR(PIN_M1_IN1,PIN_M1_IN2)
    m2 = MOTOR(PIN_M2_IN1,PIN_M2_IN2)
    m3 = MOTOR(PIN_M3_IN1,PIN_M3_IN2)
    m4 = MOTOR(PIN_M4_IN1,PIN_M4_IN2)
    car = CAR(m1,m2,m3,m4)
    return car


#######################
# Init ADC inputs
#######################
def init_batt_mon():

    print("Init battery monitor")
    bm = BATTERY(PIN_ADC_BATT)
    return bm


#######################
# Init Servo motor
#######################
def init_servo():

    print("Init Servo")
    s = SERVO(PIN_SRV1)
    return s


#######################
# Init LED strip
#######################
def init_ledstrip():

    print("Init LED strip")
    l = LEDSTRIP(PIN_NEOPIX)
    return l


########################
# Init Ultrasonic sensor
########################
def init_ultrasonic(car=None, buzzer=None):

    print("Init Ultrasonic sensor")
    u = ULTRASONIC(PIN_US_TRIG,PIN_US_ECHO,car,buzzer)
    return u


########################
# Init Track sensor
########################
def init_track(car=None, buzzer=None):

    print("Init Track sensor")
    t = TRACK(PIN_TRK1,PIN_TRK2,PIN_TRK3,car,buzzer)
    return t


########################
# Init Light sensor
########################
def init_light():

    print("Init Light sensor")
    light = LIGHT(PIN_ADC_PR2,PIN_ADC_PR1)
    return light


########################
# Init Buzzer
########################
def init_buzzer():

    print("Init Buzzer")
    b = BUZZER(PIN_BUZZER)
    return b


#######################
# Init LEDs
#######################
def init_led():
    
    print("Init LEDs")
    lp = LEDCTL(PIN_LED_PICO)
    return lp


#######################
# Init serial port
#######################
def init_serial():
    print("Init Debug port")
    return (UART(1, baudrate=115200, tx=Pin(PIN_UART0_TX), rx=Pin(PIN_UART0_RX)))


#######################
# Init Pico RTC
#######################
def init_rtc_pico():
    print("Init RTC Pico")
    return (RTC())


#######################
# Init uptime counter
#######################
def init_uptime():
    print("Init Uptime counter")
    return (UPTIME())
