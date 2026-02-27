#!/bin/bash
# AAPA Closed-Loop Polar Alignment Automation Script
# Supports two modes:
#   1. Auto mode: monitors Ekos Align INDI device for PAA solution (if published)
#   2. Manual mode: enter az/alt error in arcseconds directly

INDI_PORT=7624
AAPA_DEVICE="AAPA Polar Alignment"
ALIGN_DEVICE="Align"

# Threshold in degrees. Stop correcting if error is below this.
THRESHOLD_DEG=0.01

# ─────────────────────────────────────────────
# Helper: send correction in degrees directly to AAPA_PAA_ERROR property
# Usage: send_paa_correction <az_deg> <alt_deg>
# ─────────────────────────────────────────────
send_paa_correction() {
    local az_deg=$1
    local alt_deg=$2
    echo "  → Sending PAA correction: Az=${az_deg}°  Alt=${alt_deg}°"
    indi_setprop -p $INDI_PORT "${AAPA_DEVICE}.AAPA_PAA_ERROR.AZ_ERR=${az_deg};ALT_ERR=${alt_deg}"
}

# ─────────────────────────────────────────────
# Mode: --correct <az_arcsec> <alt_arcsec>
# Quick one-shot correction from Ekos PAA screen values.
# Example: aapa_closed_loop.sh --correct 14 62
# (Negative values will move in the opposite direction)
# ─────────────────────────────────────────────
if [[ "$1" == "--correct" ]]; then
    AZ_ARCSEC=${2:-0}
    ALT_ARCSEC=${3:-0}
    AZ_DEG=$(awk -v a="$AZ_ARCSEC"  'BEGIN{printf "%.6f", a/3600}')
    ALT_DEG=$(awk -v a="$ALT_ARCSEC" 'BEGIN{printf "%.6f", a/3600}')
    echo "AAPA PAA One-Shot Correction"
    echo "  Input: Az=${AZ_ARCSEC}\"  Alt=${ALT_ARCSEC}\""
    send_paa_correction "$AZ_DEG" "$ALT_DEG"
    echo "Done."
    exit 0
fi

# ─────────────────────────────────────────────
# Mode: --interactive
# Prompt the user to enter the error from the Ekos PAA screen.
# ─────────────────────────────────────────────
if [[ "$1" == "--interactive" ]]; then
    echo "AAPA Interactive PAA Correction Loop"
    echo "After each Ekos solve, read the Az/Alt arcsecond values and type them here."
    echo "Prefix with '-' for negative (e.g. -14 for 14\" in the opposite direction)."
    echo "Press Ctrl+C to stop."
    while true; do
        echo ""
        read -r -p "Azimuth error in arcseconds [0 = done]: " AZ_ARCSEC
        [[ "$AZ_ARCSEC" == "0" || -z "$AZ_ARCSEC" ]] && break
        read -r -p "Altitude error in arcseconds:           " ALT_ARCSEC
        AZ_DEG=$(awk -v a="$AZ_ARCSEC"  'BEGIN{printf "%.6f", a/3600}')
        ALT_DEG=$(awk -v a="$ALT_ARCSEC" 'BEGIN{printf "%.6f", a/3600}')
        send_paa_correction "$AZ_DEG" "$ALT_DEG"
        echo "Motors moving... waiting 10 seconds for Ekos to refresh image."
        sleep 10
    done
    echo "Correction loop ended."
    exit 0
fi

# ─────────────────────────────────────────────
# Mode: --no-daemon (auto / background loop)
# Polls the INDI Align device (if Ekos publishes it).
# ─────────────────────────────────────────────
if [[ "$1" != "--no-daemon" ]]; then
    echo "AAPA Automation: Backgrounding loop..."
    nohup $0 --no-daemon </dev/null >/tmp/aapa_automation.log 2>&1 &
    exit 0
fi

echo "Starting AAPA Auto Closed-Loop Automation..."
echo "Waiting 10 seconds for Ekos to finish establishing its connections..."
sleep 10
echo "Monitoring for Ekos Alignment solution on INDI port $INDI_PORT..."

while true; do
    ERR_AZ=$(indi_getprop  -1 -t 1 -p $INDI_PORT "${ALIGN_DEVICE}.ALIGN_SOLUTION.AZ_ERROR"  2>/dev/null)
    ERR_ALT=$(indi_getprop -1 -t 1 -p $INDI_PORT "${ALIGN_DEVICE}.ALIGN_SOLUTION.ALT_ERROR" 2>/dev/null)

    if [[ -z "$ERR_AZ" || -z "$ERR_ALT" ]]; then
        sleep 2
        continue
    fi

    AZ_VAL=$(echo "$ERR_AZ"  | tr -cd '0-9.-')
    ALT_VAL=$(echo "$ERR_ALT" | tr -cd '0-9.-')

    AZ_DONE=$(awk  -v v="$AZ_VAL"  -v t=$THRESHOLD_DEG 'BEGIN{print (v<t && v>-t) ? 1 : 0}')
    ALT_DONE=$(awk -v v="$ALT_VAL" -v t=$THRESHOLD_DEG 'BEGIN{print (v<t && v>-t) ? 1 : 0}')

    if [[ "$AZ_DONE" -eq 1 && "$ALT_DONE" -eq 1 ]]; then
        echo "Successfully Aligned! Error (Az:$AZ_VAL°, Alt:$ALT_VAL°) is below threshold ${THRESHOLD_DEG}°"
        break
    fi

    echo "Current Error → Azimuth: ${AZ_VAL}°  Altitude: ${ALT_VAL}°"
    send_paa_correction "$AZ_VAL" "$ALT_VAL"

    echo "Waiting 10 seconds for Ekos to refresh image..."
    sleep 10
done
