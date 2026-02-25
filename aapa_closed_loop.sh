#!/bin/bash
# AAPA Closed-Loop Polar Alignment Automation Script
# This script monitors Ekos Alignment Module for Altitude and Azimuth errors
# and automatically commands the AAPA driver motor to close the loop.

INDI_PORT=7624
AAPA_DEVICE="AAPA Polar Alignment"
ALIGN_DEVICE="Align" 

# Thresholds (in degrees). Stop correcting if error is below this.
THRESHOLD_DEG=0.01

# The number of motor steps/units per degree of error.
# Users will need to calibrate this multiplier based on their specific mount threading.
# Example: 1 degree of error requires 50 AAPA motor jog units.
STEPS_PER_DEG_AZ=50
STEPS_PER_DEG_ALT=50

echo "Starting AAPA Closed-Loop Automation..."
echo "Waiting 10 seconds for Ekos to finish establishing its connections..."
sleep 10
echo "Waiting for Ekos Alignment solution..."

while true; do
    # 1. Grab the latest Azimuth and Altitude error from Ekos Align Module
    # We look for the properties the Align module exposes when it computes a solution.
    # Note: Property names might vary slightly depending on Ekos version.
    
    # Try reading ALIGN_ERROR property (if published), with a 1-second timeout
    ERR_AZ=$(indi_getprop -1 -t 1 -p $INDI_PORT "$ALIGN_DEVICE.ALIGN_SOLUTION.AZ_ERROR" 2>/dev/null)
    ERR_ALT=$(indi_getprop -1 -t 1 -p $INDI_PORT "$ALIGN_DEVICE.ALIGN_SOLUTION.ALT_ERROR" 2>/dev/null)
    
    if [[ -z "$ERR_AZ" || -z "$ERR_ALT" ]]; then
        # Just wait if Ekos hasn't solved yet
        sleep 2
        continue
    fi
    
    # Parse numbers out of string if needed (assuming format is a pure float)
    AZ_VAL=$(echo $ERR_AZ | tr -cd '0-9.-')
    ALT_VAL=$(echo $ERR_ALT | tr -cd '0-9.-')
    
    # Use awk to check if we are below threshold
    AZ_DONE=$(awk -v az=$AZ_VAL -v t=$THRESHOLD_DEG 'BEGIN{if(az < t && az > -t) print 1; else print 0}')
    ALT_DONE=$(awk -v alt=$ALT_VAL -v t=$THRESHOLD_DEG 'BEGIN{if(alt < t && alt > -t) print 1; else print 0}')
    
    if [ "$AZ_DONE" -eq 1 ] && [ "$ALT_DONE" -eq 1 ]; then
        echo "Successfully Aligned! Error (Az:$AZ_VAL, Alt:$ALT_VAL) is below threshold $THRESHOLD_DEG"
        break
    fi
    
    echo "Current Error -> Azimuth: $AZ_VAL degrees | Altitude: $ALT_VAL degrees"
    
    # Calculate Jog Units needed to correct the error
    # E.g., Error of 0.5 degrees * 50 steps/deg = 25 Jog Units
    JOG_AZ=$(awk -v az=$AZ_VAL -v sm=$STEPS_PER_DEG_AZ 'BEGIN{printf "%.2f", -az * sm}')
    JOG_ALT=$(awk -v alt=$ALT_VAL -v sm=$STEPS_PER_DEG_ALT 'BEGIN{printf "%.2f", -alt * sm}')
    
    echo "Sending Jogs -> X_JOG: $JOG_AZ | Y_JOG: $JOG_ALT"
    
    # Send Jog command to the AAPA Driver
    # Note: In INDI, setting multiple elements requires formatting them with semicolons
    indi_setprop -p $INDI_PORT "$AAPA_DEVICE.AAPA_JOG.X_JOG=$JOG_AZ;Y_JOG=$JOG_ALT"
    
    # Wait for the physical movement to finish (approximate based on speed/distance)
    sleep 3
    
    # Wait for the next solution from Ekos
    echo "Movement complete. Waiting 10 seconds for Ekos to refresh its image..."
    sleep 10
done
