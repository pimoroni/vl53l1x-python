#!/usr/bin/env python

import json
import signal
import sys
import time

import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

import VL53L1X

DEFAULT_MQTT_BROKER_IP = "localhost"
DEFAULT_MQTT_BROKER_PORT = 1883
DEFAULT_MQTT_TOPIC = "VL53L1X"


def exit_handler(signal, frame):
    global running
    running = False
    tof.stop_ranging()
    print()
    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description="Publish enviroplus values over mqtt")
    parser.add_argument(
        "--broker", default=DEFAULT_MQTT_BROKER_IP, type=str, help="mqtt broker IP",
    )
    parser.add_argument(
        "--port", default=DEFAULT_MQTT_BROKER_PORT, type=int, help="mqtt broker port",
    )
    parser.add_argument(
        "--topic", default=DEFAULT_MQTT_TOPIC, type=str, help="mqtt topic"
    )
    args = parser.parse_args()

    print(
        """distance.py

    Display the distance read from the sensor, and publishes over mqtt.

    Uses the "Short Range" timing budget by default.


    broker: {}
    port: {}
    topic: {}

    Press Ctrl+C to exit.

    """.format(
            args.broker, args.port, args.topic
        )
    )

    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_publish = on_publish
    mqtt_client.connect(args.broker, port=args.port)
    mqtt_client.loop_start()

    tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
    tof.open()
    tof.start_ranging(1)  # Start ranging
    # 0 = Unchanged
    # 1 = Short Range
    # 2 = Medium Range
    # 3 = Long Range

    running = True
    # Attach a signal handler to catch SIGINT (Ctrl+C) and exit gracefully
    signal.signal(signal.SIGINT, exit_handler)

    while running:
        distance_in_mm = tof.get_distance()
        print("Distance: {}mm".format(distance_in_mm))
        mqtt_client.publish(args.topic, json.dumps({"distance_mm": distance_in_mm}))
        time.sleep(0.2)


if __name__ == "__main__":
    main()
