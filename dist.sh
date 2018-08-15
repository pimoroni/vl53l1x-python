#!/bin/bash

PYTHON_VERSIONS=(
	"3.5"
	"3.4"
	"2.7"
)

for ((i = 0; i < ${#PYTHON_VERSIONS[@]}; i++)); do
	PYTHON_VERSION="${PYTHON_VERSIONS[$i]}"
	PYTHON_PATH="/usr/bin/python$PYTHON_VERSION"
	printf "Building for Python $PYTHON_VERSION\n"
	$PYTHON_PATH setup.py bdist_wheel
done

