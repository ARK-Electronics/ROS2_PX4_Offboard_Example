#!/usr/bin/env python3

import subprocess
import time

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Run the PX4 SITL simulation
    "cd ~/PX4-Autopilot && make px4_sitl gz_x500",

    # Run QGroundControl
    # "cd ~/QGroundControl && ./QGroundControl.AppImage"
]

# Default terminal command on Ubuntu
terminal_cmd = 'x-terminal-emulator'

# Loop through each command in the list
for command in commands:
    # Build the full command
    full_cmd = [terminal_cmd, '-e', f"bash -c '{command}; exec bash'"]
    # Run the command
    subprocess.Popen(full_cmd)
    # Pause between each command
    time.sleep(1)
