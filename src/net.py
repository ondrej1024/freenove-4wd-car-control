###############################################################################
#
# Freenove 4WD car networking
#
# Author: Ondrej Wisniewski
#
# Description:
#   These functionsa handle the cars networking
#   - Wifi network connection
#   - Websocket server
#
# Changelog:
#   15-11-2025: Initial version
#
# Copyright 2025
#
# This file is part of the Freenove 4WD car control platform.
#
###############################################################################
import network
import time
import asyncio
import json
from microdot import Microdot
from microdot import send_file
from microdot.websocket import with_websocket
from globals import Globals

SSID = "ssid"
PASS = "pass"
HOST = "robocar"

nic = None


#####################################
# Network connection
#####################################
def connect_network():
    global nic
    print ("Init Network")
    network.hostname(HOST)
    nic = network.WLAN(network.WLAN.IF_STA)
    nic.active(True)
    nic.connect(ssid=SSID, key=PASS)

    while True:
        time.sleep(1)
        status = nic.status()
        print("   status: %d" % status)
        if nic.isconnected() and status == network.STAT_GOT_IP:
            break
    print("   connected with hostname %s, IP %s, gateway %s" % (network.hostname(),nic.ipconfig("addr4")[0],nic.ipconfig("gw4")))
    print("   signal strength: %d" % nic.status("rssi"))


#####################################
# Network status
#####################################
def network_status():
    global nic
    netstat = {"ip":nic.ipconfig("addr4")[0],
               "gw":nic.ipconfig("gw4"),
               "ss":nic.status("rssi"),
               "id":SSID}
    return netstat    
    

#####################################
# Websockets server
#####################################

webapp = Microdot()

@webapp.route("/")
async def index(request):
    print("webserver: send index.html")
    return send_file("index.html")

@webapp.route('/ws')
@with_websocket
async def handle_ws(request, ws):
    try:
        print("websocket: start ws handler")
        while True:
            
            # Wait for message
            message = await ws.receive()
            print("WS message received: %s" % (message))
            if message == "forward":
                Globals.car.forward(20)
            elif message == "backward":
                Globals.car.backward(20)
            elif message == "turnleft":
                Globals.car.turn_left(30)
            elif message == "turnright":
                Globals.car.turn_right(30)
            elif message == "spinleft":
                Globals.car.spin_left()
            elif message == "spinright":
                Globals.car.spin_right()
            elif message == "stop":
                Globals.car.stop()
                
            #try:
            #    message = asyncio.wait_for(ws.receive(),1)
            #    print("received message: %s" % message)
            #except asyncio.TimeoutError:
            #    pass
            
            # Get network status
            net = network_status()
            
            # Send sensor data
            payload = {"data":[Globals.bat.perc(),
                               Globals.ult.distance(),
                               Globals.trk.status(),
                               Globals.lit.level()
                              ],
                       "net": [net["ip"],
                               net["gw"],
                               net["ss"],
                               net["id"]
                              ]
                      }
            await ws.send(json.dumps(payload))
            #await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("Client disconnected!")

async def webserver():
    print("Start Websocket server")
    server = asyncio.create_task(webapp.start_server(port=80))
    await server

def start_webserver():
    print("Init Websocket server")
    #webapp.run()
    asyncio.run(webserver())
