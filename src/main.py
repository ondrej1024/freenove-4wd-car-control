 
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
led = freenove_4wd_hardware.init_led()
led.on()

# Init Neopixel LED strip
neo = freenove_4wd_hardware.init_ledstrip()
neo.walk("R")
neo.walk("G")
neo.walk("B")
sleep(DELAY_WAIT)

# Init servo
srv = freenove_4wd_hardware.init_servo()
srv.sweep()

# Init battery monitor
bat = freenove_4wd_hardware.init_batt_mon()
sleep(DELAY_WAIT)

# Init buzzer
buz = freenove_4wd_hardware.init_buzzer()
buz.beep()

# Init Ultrasonic sensor
ult = freenove_4wd_hardware.init_ultrasonic(buzzer=buz)
sleep(DELAY_WAIT)

# Init Track sensor
trk = freenove_4wd_hardware.init_track(buzzer=buz)
sleep(DELAY_WAIT)

# Init Light sensor
lit = freenove_4wd_hardware.init_light()
sleep(DELAY_WAIT)

# Init car
car = CAR(m1,m2,m3,m4)
sleep(DELAY_WAIT)

'''
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
'''

while True:
    sleep(2)
    print("battery voltage is %.2fV (%d%%)" % (bat.voltage(),bat.perc()))
    print("obstacle %.2f cm ahead" % ult.distance())
    print("track status is %d-%d-%d" % trk.status())
    print("light level left %.1f%% / right %.1f%%" % lit.level())
    print()
    