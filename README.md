# INDI AAPA — Automated Astronomical Polar Alignment

An [INDI](https://www.indilib.org/) driver for the **AAPA** (Automated Astronomical Polar Alignment) device — a motorised altitude/azimuth adjustment system built with Arduino + Grbl that lets you polar-align an equatorial mount without touching the knobs.

> **⚠️ Beta Software** — This driver is under active development. Please report any issues on the [GitHub Issues](https://github.com/michelebergo/indi-aapa/issues) page.

---

## Features

- **Manual Jog Control** — Command azimuth (X) and altitude (Y) motor axes from Ekos / any INDI client
- **Adjustable Speed** — Set the feed rate to match your mount's adjustment sensitivity
- **Abort Button** — Emergency stop for all motion
- **Live Position Readout** — Real-time motor position reported back to the client
- **Closed-Loop Automation** *(experimental)* — Script that reads Ekos polar alignment error and automatically corrects it

---

## Hardware Requirements

| Component | Details |
|-----------|---------|
| **AAPA Device** | Motorised alt/az adjustment platform |
| **Controller** | Arduino (Uno/Nano) running Grbl 1.1 firmware |
| **Connection** | USB serial (typically `/dev/ttyUSB0`) |
| **Computer** | Raspberry Pi or Linux PC running INDI / KStars+Ekos |

---

## Installation

### Prerequisites

A Debian/Ubuntu-based Linux system (including Raspberry Pi OS) with INDI already installed. If you don't have INDI yet:

```bash
sudo apt-add-repository ppa:mutlaqja/ppa
sudo apt update
sudo apt install indi-bin libindi-dev
```

### Install the AAPA Driver

```bash
git clone https://github.com/michelebergo/indi-aapa.git
cd indi-aapa
chmod +x install.sh
sudo ./install.sh
```

That's it! The installer will compile the driver and place everything in the correct system paths.

### Uninstall

```bash
sudo ./uninstall.sh
```

---

## Configuration in Ekos / KStars

1. Open **KStars → Ekos → Profile Editor**
2. Click **"Auxiliary"** and select **"AAPA Polar Alignment"** from the driver list
3. Set the serial port (usually `/dev/ttyUSB0`) in the driver's **Port** field
4. Click **Connect**

Once connected you will see:

| Control | Description |
|---------|-------------|
| **Position** | Current X (azimuth) and Y (altitude) in motor steps |
| **Jog** | Enter a relative movement value and press Set |
| **Speed** | Feed rate in mm/min (default: 500) |
| **Abort** | Emergency stop |

---

## Closed-Loop Automation *(Experimental)*

The included `aapa_closed_loop.sh` script can automatically correct polar alignment errors detected by Ekos.

### How It Works

1. Ekos captures a plate-solve and computes alt/az polar alignment error
2. The script reads the error values via INDI
3. It converts degrees of error into motor jog units
4. It sends correction commands to the AAPA driver
5. It waits for a new solve and repeats until the error is below threshold

### Setup

1. Edit the calibration constants in `aapa_closed_loop.sh`:

```bash
# Motor units per degree of error — calibrate for YOUR mount
STEPS_PER_DEG_AZ=50
STEPS_PER_DEG_ALT=50

# Stop correcting below this threshold (degrees)
THRESHOLD_DEG=0.01
```

2. In Profile Editor → Scripts → Post-Startup, point to `auto_aapa.sh` (installed in `/usr/local/bin/`).

### Calibrating STEPS_PER_DEG

1. Use Ekos Polar Alignment to measure the current error (e.g. 0.5° in azimuth)
2. Manually jog the AAPA by a known amount (e.g. 25 units)
3. Re-solve and measure the new error
4. Calculate: `STEPS_PER_DEG = jog_units / degrees_corrected`
5. Update the value in the script

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| **Driver not listed in Ekos** | Verify the XML is installed: `ls /usr/share/indi/indi_aapa_polaralignment.xml` |
| **Connection fails** | Check the serial port: `ls /dev/ttyUSB*`. Try `sudo chmod 666 /dev/ttyUSB0` |
| **Handshake timeout** | The Arduino may need a longer reset time. Reconnect and wait a few seconds |
| **Motor doesn't move** | Verify Grbl is responding: `screen /dev/ttyUSB0 115200` and type `?` |
| **Wrong direction** | Swap the sign in `STEPS_PER_DEG` or invert motor wiring |

---

## Reporting Bugs

Please open an issue at [github.com/michelebergo/indi-aapa/issues](https://github.com/michelebergo/indi-aapa/issues) with:

- A description of what went wrong
- Your system info (OS, INDI version: `indiserver --version`)
- Relevant log output from `/tmp/aapa_automation.log` (if using closed-loop)
- INDI log output (enable logging in Ekos → INDI Control Panel → Logs)

---

## License

This project is licensed under the [GNU General Public License v2.0](LICENSE) — the standard license for INDI ecosystem drivers.
