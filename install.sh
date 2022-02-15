#!/bin/bash

PYTHON_VERSIONS=(
	"3.9"
	"3.8"
	"3.7"
	"3.5"
	"3.4"
	"2.7"
)

printf "VL53L1X Python Library: Installer\n\n"

if [ $(id -u) -ne 0 ]; then
	printf "Script must be run as root. Try 'sudo ./install.sh'\n"
	exit 1
fi

for ((i = 0; i < ${#PYTHON_VERSIONS[@]}; i++)); do
	PYTHON_VERSION="${PYTHON_VERSIONS[$i]}"
	PYTHON_PATH="/usr/bin/python$PYTHON_VERSION"
	if [[ -f "$PYTHON_PATH" ]]; then
		printf "Installing for Python $PYTHON_VERSION\n"
		$PYTHON_PATH -m pip install vl53l1x
		$PYTHON_PATH -m pip install paho-mqtt
	else
		printf "Skipping Python $PYTHON_VERSION\n"
	fi
done

