#!/bin/bash

source $HOME/Documents/repos/sepb/bash/variables.sh

# Remove build directory
function rebuild_ai() {
	# Check if the directory exists
	if [ -d $AI_BUILD_DIR ]; then
		echo "Deleting existing build directory"
		rm -r $AI_BUILD_DIR
	fi
}

# Install dependencies
function install_ai() {
	cd $AI_PATH
	python3 -m pip install .
}

# Start Artificial Intelligence
function start_ai() {
	rebuild_ai
	install_ai
}
