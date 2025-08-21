#!/usr/bin/env python
import curses
import math
import time

# Dependency installed with `pip install paho-mqtt`.
# https://pypi.org/project/paho-mqtt/
import paho.mqtt.client as mqtt

MQTT_SERVER_ADDR = '192.168.1.110'
MQTT_CMD_TOPIC = '/mint_bot/cmd'
MQTT_STATE_TOPIC = '/mint_bot/state'

state = {
    'moving': '0'
}

# Define MQTT callbacks
def on_connect(client, userdata, connect_flags, reason_code, properties):
    print("Connected with result code "+str(reason_code))
    client.subscribe(MQTT_STATE_TOPIC)

def on_message(client, userdata, msg):
    state['moving'] = msg.payload.decode('ascii')

# Create an MQTT client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

# Set MQTT callbacks
client.on_connect = on_connect
client.on_message = on_message

# Connect to the MQTT broker
client.connect(MQTT_SERVER_ADDR, 1883, 60)


stdscr = curses.initscr()
curses.cbreak()
curses.noecho()
stdscr.nodelay(True)
stdscr.keypad(1)

new_command = "STOP"
started = False
try:
    while client.loop(timeout=1.0) == mqtt.MQTT_ERR_SUCCESS:
        time.sleep(.1)
        key = stdscr.getch()
        if key != curses.ERR:
            if key == curses.KEY_UP:
                new_command = "MOVE,0.1"
            elif key == curses.KEY_DOWN:
                new_command = "MOVE,-0.1"
            elif key == curses.KEY_RIGHT:
                new_command = f"ROTATE,-{math.pi/2}"
            elif key == curses.KEY_LEFT:
                new_command = f"ROTATE,{math.pi/2}"
            else:
                new_command = "STOP"
                stdscr.move(2, 0)
                stdscr.clrtoeol()
            started = False
            client.publish(MQTT_CMD_TOPIC, new_command)

        if new_command != "STOP":
            if started:
                if state['moving'] == "1":
                    stdscr.addstr(2, 0, f"Waiting to finish: {new_command}")
                    stdscr.clrtoeol()
                    stdscr.refresh()
                else:
                    stdscr.addstr(2, 0, f"Finished: {new_command}")
                    stdscr.clrtoeol()
                    stdscr.refresh()

            else:
                if state['moving'] != "1":
                    stdscr.addstr(2, 0, f"Waiting to start: {new_command}")
                    stdscr.clrtoeol()
                    stdscr.refresh()
                else:
                    started = True

    print('Connection Failure')
except KeyboardInterrupt:
    pass

curses.endwin()
