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

SSID = "my SSID"
PASS = "my password"


#####################################
# Network connection
#####################################
def connect_network():
    print ("Init Network")
    nic = network.WLAN(network.WLAN.IF_STA)
    nic.active(True)
    nic.connect(ssid=SSID, key=PASS)

    while True:
        time.sleep(1)
        status = nic.status()
        print("   status: %d" % status)
        if nic.isconnected() and status == network.STAT_GOT_IP:
            break
    print("   connected with IP %s, gateway %s" % (nic.ipconfig("addr4")[0],nic.ipconfig("gw4")))


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
    cnt = 1
    try:
        print("websocket: start ws handler")
        while True:
            #message = await ws.receive()
            #try:
            #    message = asyncio.wait_for(ws.receive(),1)
            #    print("received message: %s" % message)
            #except asyncio.TimeoutError:
            #    pass
            
            payload = {"data":[Globals.bat.perc(),
                               Globals.ult.distance(),
                               Globals.trk.status(),
                               Globals.lit.level()
                              ]
                      }
            
            await ws.send(json.dumps(payload))
            #await ws.send(str(cnt))
            cnt = cnt + 1
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("Client disconnected!")

async def webserver():
    print("Start Websocket server")
    server = asyncio.create_task(webapp.start_server())
    await server

def start_webserver():
    print("Init Websocket server")
    #webapp.run()
    asyncio.run(webserver())
