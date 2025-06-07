###############################################################################
#
# Freenove 4WD car controller
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

import freenove_4wd_hardware
from time import sleep

DELAY_ACTION = 2
DELAY_WAIT   = 2


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
        self._m1.move(speed)
        self._m2.move(speed)
        self._m3.move(speed)
        self._m4.move(speed)

    def backward(self,speed):
        speed = -abs(speed)
        self._m1.move(speed)
        self._m2.move(speed)
        self._m3.move(speed)
        self._m4.move(speed)
    
    def turn_right(self):
        speed = 30
        self._m1.move(speed)
        self._m2.move(speed)
        self._m3.move(-speed)
        self._m4.move(-speed)
        
    def turn_left(self):
        speed = 30
        self._m1.move(-speed)
        self._m2.move(-speed)
        self._m3.move(speed)
        self._m4.move(speed)

    def curve_right(self,speed):
        self._m1.move(speed)
        self._m2.move(speed)
        self._m3.move(speed//3)
        self._m4.move(speed//3)
        
    def curve_left(self,speed):
        self._m1.move(speed//3)
        self._m2.move(speed//3)
        self._m3.move(speed)
        self._m4.move(speed)
        
    def stop(self):
        self._m1.stop()
        self._m2.stop()
        self._m3.stop()
        self._m4.stop()
        
        
###########################################################
#
#        MAIN PROGRAM
#
###########################################################

# Init led
l = freenove_4wd_hardware.init_led()

# Init motors
(m1,m2,m3,m4) = freenove_4wd_hardware.init_motors()

# Init car
car = CAR(m1,m2,m3,m4)

sleep(DELAY_WAIT)

# Move forward
car.forward(30)
sleep(DELAY_ACTION)
car.stop()
sleep(DELAY_WAIT)

# Move backward
car.backward(30)
sleep(DELAY_ACTION)
car.stop()
sleep(DELAY_WAIT)

# Turn left
car.turn_left()
sleep(DELAY_ACTION)
car.stop()
sleep(DELAY_WAIT)

# Turn right
car.turn_right()
sleep(DELAY_ACTION)
car.stop()
sleep(DELAY_WAIT)

# Curve left
car.curve_left(60)
sleep(DELAY_ACTION)
car.stop()
sleep(DELAY_WAIT)

# Curve right
car.curve_right(60)
sleep(DELAY_ACTION)
car.stop()
sleep(DELAY_WAIT)

