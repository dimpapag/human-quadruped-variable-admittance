#!/bin/bash

# Define a list of potential Arduino ports
ports=(/dev/ttyACM*)

# Flag to indicate if a port was found
port_found=false

# Iterate over potential ports
for port in "${ports[@]}"; do
    # Check if the port exists
    if [ -e "$port" ]; then
        echo "Arduino found on $port"
        sudo chmod 777 "$port"
        port_found=true
        break
    fi
done

# If no port was found, print a message
if [ "$port_found" = false ]; then
    echo "No Arduino found."
fi
