#!/bin/bash

# Remove build directory
function rebuild_ai() {
	rm -r /home/cobot/Documents/repos/sepb/ai/build
}

# Install dependencies
function install_ai() {
	python3 -m pip install .
}

# Start Artificial Intelligence
function start_ai() {
	rebuild_ai
	install_ai
}
