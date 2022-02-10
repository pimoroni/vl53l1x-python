#!/usr/bin/env python

import argparse
import json
import signal
import sys
import time

import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

import VL53L1X

RANGING = {0: "Unchanged", 1: "Short Range", 2: "Medium Range", 3: "Long Range"}
DEFAULT_RANGING = 3

DEFAULT_MQTT_BROKER_IP = "localhost"
DEFAULT_MQTT_BROKER_PORT = 1883
DEFAULT_MQTT_TOPIC = "VL53L1X"

DEFAULT_SLEEP = 0.1

# mqtt callbacks
def on_connect(client, userdata, flags, rc):
    print(f"CONNACK received with code {rc}")
    if rc == 0:
        print("connected OK")
    else:
        print("Bad connection Returned code=", rc)


def on_publish(client, userdata, mid):
    print("mid: " + str(mid))


def main():
    global running

    parser = argparse.ArgumentParser(description="Publish VL53L1X distances over mqtt")
    parser.add_argument(
        "--broker", default=DEFAULT_MQTT_BROKER_IP, type=str, help="mqtt broker IP",
    )
    parser.add_argument(
        "--port", default=DEFAULT_MQTT_BROKER_PORT, type=int, help="mqtt broker port",
    )
    parser.add_argument(
        "--topic", default=DEFAULT_MQTT_TOPIC, type=str, help="mqtt topic"
    )
    parser.add_argument(
        "--ranging",
        default=DEFAULT_RANGING,
        type=int,
        choices=[0, 1, 2, 3],
        help="ranging mode",
    )
    parser.add_argument(
        "--sleep",
        default=DEFAULT_SLEEP,
        type=float,
        help="sleep time between measurements",
    )
    args = parser.parse_args()

    print(
        """distance-mqtt.py

    Display the distance read from the sensor, and publishes over mqtt.

    ranging: {}, {}
    sleep: {}
    broker: {}
    port: {}
    topic: {}

    Press Ctrl+C to exit.

    """.format(
            args.ranging,
            RANGING[args.ranging],
            args.sleep,
            args.broker,
            args.port,
            args.topic,
        )
    )

    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_publish = on_publish
    mqtt_client.connect(args.broker, port=args.port)
    mqtt_client.loop_start()

    tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
    tof.open()
    tof.start_ranging(args.ranging)  # Start ranging


    def exit_handler(signal, frame):
        global running
        running = False


    running = True
    # Attach a signal handler to catch SIGINT (Ctrl+C) and exit gracefully
    signal.signal(signal.SIGINT, exit_handler)

    while running:
        distance_in_mm = tof.get_distance()
        print("Distance: {}mm".format(distance_in_mm))
        mqtt_client.publish(args.topic, json.dumps({"distance_mm": distance_in_mm}))
        time.sleep(args.sleep)

    tof.stop_ranging()
    mqtt_client.disconnect()
    mqtt_client.loop_stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
