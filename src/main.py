 
import freenove_4wd_hardware
import net
from time import sleep
import asyncio
from globals import Globals

DELAY_ACTION = 2
DELAY_WAIT   = 2
       

async def sensormon():
    print("Start sensor monitor")
    while True:
        await asyncio.sleep(2)
        try:
            print("battery voltage is %.2fV (%d%%)" % (Globals.bat.voltage(),Globals.bat.perc()))
            print("obstacle %.2f cm ahead" % Globals.ult.distance())
            print("track status is %d-%d-%d" % Globals.trk.status())
            print("light level left %.1f%% / right %.1f%%" % Globals.lit.level())
        except TypeError:
            pass
        print()


###################################################
# MAIN FUNCTION ON CORE 0
###################################################
async def main_core0():
    # Start all modules as concurrent tasks
    # Each module executes an infinite loop
    await asyncio.gather(net.webserver(),
                         sensormon()
                        )


def run_core0():
    print("Start CORE 0 loop")
    asyncio.run(main_core0())


###########################################################
#
#        MAIN PROGRAM
#
###########################################################

# Init motors
(m1,m2,m3,m4) = freenove_4wd_hardware.init_motors()

# Init car
Globals.car = freenove_4wd_hardware.init_car(m1,m2,m3,m4)

# Init led
Globals.led = freenove_4wd_hardware.init_led()
Globals.led.heartbeat()

# Init Neopixel LED strip
Globals.neo = freenove_4wd_hardware.init_ledstrip()
#neo.walk("R")
#neo.walk("G")
#neo.walk("B")
#sleep(DELAY_WAIT)

# Init servo
Globals.srv = freenove_4wd_hardware.init_servo()
#srv.sweep()
#sleep(DELAY_WAIT)

# Init battery monitor
Globals.bat = freenove_4wd_hardware.init_batt_mon()
#sleep(DELAY_WAIT)

# Init buzzer
Globals.buz = freenove_4wd_hardware.init_buzzer()
#buz.beep()

# Init Ultrasonic sensor
Globals.ult = freenove_4wd_hardware.init_ultrasonic(buzzer=Globals.buz)
#sleep(DELAY_WAIT)

# Init Track sensor
Globals.trk = freenove_4wd_hardware.init_track(buzzer=Globals.buz)
#sleep(DELAY_WAIT)

# Init Light sensor
Globals.lit = freenove_4wd_hardware.init_light()
#sleep(DELAY_WAIT)

# Init network
net.connect_network()
Globals.led.on()
Globals.neo.blink("G")


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

# Start services
run_core0()
