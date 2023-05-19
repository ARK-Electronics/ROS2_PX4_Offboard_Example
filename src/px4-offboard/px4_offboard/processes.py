#!/usr/bin/env python3

import subprocess
import time

# List of commands to run
commands = [
    "MicroXRCEAgent udp4 -p 8888",
    "cd ~/PX4-Autopilot && make px4_sitl gz_x500",
    "cd ~/QGroundControl && ./QGroundControl.AppImage"
]

for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause for 5 seconds
    time.sleep(3)
