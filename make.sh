#!/bin/bash

# Function to clean up child processes
cleanup() {
    echo "Terminating all child processes..."
    kill $(jobs -p) &>/dev/null
    wait
}

# Set the trap to catch SIGINT (Ctrl+C) and call cleanup
trap cleanup SIGINT

# Loop through all arguments and evaluate each as a shell command
for cmd in "$@"; do
    eval "$cmd" &
done

# Wait for all background processes to finish
wait

# Cleanup remaining processes
cleanup