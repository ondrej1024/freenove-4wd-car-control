 
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
        
        
###########################################################
#
#        MAIN PROGRAM
#
###########################################################

# Init motors
(m1,m2,m3,m4) = freenove_4wd_hardware.init_motors()

# Init led
l = freenove_4wd_hardware.init_led()
l.on()

# Init buzzer
bu = freenove_4wd_hardware.init_buzzer()
bu.beep()

# Init battery monitor
bm = freenove_4wd_hardware.init_batt_mon()
print("battery voltage is %.2fV (%d%%)" % (bm.voltage(),bm.perc()))
sleep(DELAY_WAIT)

# Init Neopixel LED strip
n = freenove_4wd_hardware.init_ledstrip()
n.walk("R")
n.walk("G")
n.walk("B")
sleep(DELAY_WAIT)

# Init servo
s = freenove_4wd_hardware.init_servo()
s.sweep()

# Init Ultrasonic sensor
u = freenove_4wd_hardware.init_ultrasonic()
print("obstacle %.2f cm ahead" % u.distance())
sleep(DELAY_WAIT)

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
car.turn_left(30)
sleep(DELAY_ACTION)
car.stop()
sleep(DELAY_WAIT)

# Turn right
car.turn_right(30)
sleep(DELAY_ACTION)
car.stop()
sleep(DELAY_WAIT)

n.blink("G")
