 
import freenove_4wd_hardware
from time import sleep

DELAY_ACTION = 2
DELAY_WAIT   = 2

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
        self._m3.move(15)
        self._m4.move(15)
        
    def turn_left(self,speed):
        # Left wheels
        self._m1.move(15)
        self._m2.move(15)
        # Right wheels
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

# Init battery monitor
b = freenove_4wd_hardware.init_batt_mon()
print("battery voltage is %.2fV (%d%%)" % (b.voltage(),b.perc()))

# Init motors
(m1,m2,m3,m4) = freenove_4wd_hardware.init_motors()

# Init car
car = CAR(m1,m2,m3,m4)

sleep(DELAY_WAIT)

# Move forward
car.forward(20)
sleep(DELAY_ACTION)
car.stop()
sleep(DELAY_WAIT)

# Move forward
car.backward(20)
sleep(DELAY_ACTION)
car.stop()
sleep(DELAY_WAIT)

# Spin left
car.spin_left()
sleep(DELAY_ACTION)
car.stop()
sleep(DELAY_WAIT)

# Spin right
car.spin_right()
sleep(DELAY_ACTION)
car.stop()
sleep(DELAY_WAIT)

# Turn left
car.turn_left(40)
sleep(DELAY_ACTION)
car.stop()
sleep(DELAY_WAIT)

# Turn right
car.turn_right(40)
sleep(DELAY_ACTION)
car.stop()
sleep(DELAY_WAIT)
