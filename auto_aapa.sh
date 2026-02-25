#!/bin/bash
# Wrapper script to safely launch AAPA automation in Ekos Post-Startup
# This disconnects the script from Ekos so Ekos doesn't hang waiting for it to finish.

nohup /home/pi/.gemini/antigravity/scratch/indi-aapa/aapa_closed_loop.sh > /tmp/aapa_automation.log 2>&1 < /dev/null &
disown
exit 0
