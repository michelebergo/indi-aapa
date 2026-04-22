#include "oapa_driver.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <termios.h>

#include <libindi/indicom.h>
#include <libindi/inditimer.h>
#include <libindi/indilogger.h>

// 300ms polling interval (matches NINA plugin)
#define POLL_MS 300

// Move timeout in seconds (matches NINA plugin)
#define MOVE_TIMEOUT_SEC 30

// Number of stuck iterations before declaring motor stuck
#define STUCK_THRESHOLD 5

// We declare an auto pointer to the device.
static std::unique_ptr<OAPA> oapaDevice(new OAPA());

OAPA::OAPA()
#ifdef HAVE_ALIGNMENT_CORRECTION_INTERFACE
    : ACI(this)
#endif
{
    PortFD = -1;
    memset(m_GrblStatus, 0, sizeof(m_GrblStatus));
    setVersion(2, 0);
}

const char *OAPA::getDefaultName()
{
    return "OAPA Polar Alignment";
}

bool OAPA::initProperties()
{
    // Initialize standard properties
    INDI::DefaultDevice::initProperties();

#ifdef HAVE_ALIGNMENT_CORRECTION_INTERFACE
    // Initialize PAC interface properties
    ACI::initProperties(MAIN_CONTROL_TAB);

    // Register as Auxiliary + Alignment Correction device
    setDriverInterface(AUX_INTERFACE | ALIGNMENT_CORRECTION_INTERFACE);
#endif

    // ─── Main Control Tab ──────────────────────────────────────

    // Position (Read Only)
    IUFillNumber(&PositionN[0], "X_POS", "Azimuth", "%6.2f", -100000, 100000, 0, 0);
    IUFillNumber(&PositionN[1], "Y_POS", "Altitude", "%6.2f", -100000, 100000, 0, 0);
    IUFillNumberVector(&PositionNP, PositionN, 2, getDeviceName(), "OAPA_POSITION", "Position", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // Relative Jog (Write Only)
    IUFillNumber(&JogN[0], "X_JOG", "Azimuth Relative", "%6.2f", -10000, 10000, 0, 0);
    IUFillNumber(&JogN[1], "Y_JOG", "Altitude Relative", "%6.2f", -10000, 10000, 0, 0);
    IUFillNumberVector(&JogNP, JogN, 2, getDeviceName(), "OAPA_JOG", "Jog", MAIN_CONTROL_TAB, IP_WO, 0, IPS_IDLE);

    // Absolute Move (Write Only)
    IUFillNumber(&AbsMoveN[0], "X_TARGET", "Azimuth Target", "%6.2f", -100000, 100000, 0, 0);
    IUFillNumber(&AbsMoveN[1], "Y_TARGET", "Altitude Target", "%6.2f", -100000, 100000, 0, 0);
    IUFillNumberVector(&AbsMoveNP, AbsMoveN, 2, getDeviceName(), "OAPA_ABS_MOVE", "Move To", MAIN_CONTROL_TAB, IP_WO, 0, IPS_IDLE);

    // Abort button
    IUFillSwitch(&AbortS[0], "ABORT", "Abort", ISS_OFF);
    IUFillSwitchVector(&AbortSP, AbortS, 1, getDeviceName(), "OAPA_ABORT", "Abort Motion", MAIN_CONTROL_TAB, IP_WO, ISR_ATMOST1, 0, IPS_IDLE);

    // PAA Error Input: write Ekos PAA result here to trigger auto-correction
    IUFillNumber(&PAAErrorN[0], "AZ_ERR", "Azimuth Error (deg)", "%.6f", -180, 180, 0, 0);
    IUFillNumber(&PAAErrorN[1], "ALT_ERR", "Altitude Error (deg)", "%.6f", -90, 90, 0, 0);
    IUFillNumberVector(&PAAErrorNP, PAAErrorN, 2, getDeviceName(), "OAPA_PAA_ERROR",
                       "PAA Error Input", MAIN_CONTROL_TAB, IP_WO, 0, IPS_IDLE);

    // ─── Configuration Tab ─────────────────────────────────────
    const char *CONFIG_TAB = "Configuration";

    // Per-axis speed (matching NINA's XSpeed / YSpeed)
    IUFillNumber(&SpeedN[0], "X_SPEED", "Azimuth Speed (F)", "%6.0f", 1, 10000, 10, 500);
    IUFillNumber(&SpeedN[1], "Y_SPEED", "Altitude Speed (F)", "%6.0f", 1, 10000, 10, 500);
    IUFillNumberVector(&SpeedNP, SpeedN, 2, getDeviceName(), "OAPA_SPEED", "Speed", CONFIG_TAB, IP_RW, 0, IPS_IDLE);

    // Gear ratio (matching NINA's XGearRatio / YGearRatio)
    IUFillNumber(&GearRatioN[0], "X_RATIO", "Azimuth Gear Ratio", "%6.2f", 0.01, 1000, 0.1, 1.0);
    IUFillNumber(&GearRatioN[1], "Y_RATIO", "Altitude Gear Ratio", "%6.2f", 0.01, 1000, 0.1, 1.0);
    IUFillNumberVector(&GearRatioNP, GearRatioN, 2, getDeviceName(), "OAPA_GEAR_RATIO", "Gear Ratio", CONFIG_TAB, IP_RW, 0, IPS_IDLE);

    // Steps-per-degree calibration (for PAA error conversion)
    IUFillNumber(&StepsPerDegN[0], "AZ_STEPS", "Azimuth Steps/Deg", "%6.1f", 0.1, 10000, 1, 50);
    IUFillNumber(&StepsPerDegN[1], "ALT_STEPS", "Altitude Steps/Deg", "%6.1f", 0.1, 10000, 1, 50);
    IUFillNumberVector(&StepsPerDegNP, StepsPerDegN, 2, getDeviceName(), "OAPA_STEPS_PER_DEG",
                       "Calibration", CONFIG_TAB, IP_RW, 0, IPS_IDLE);

    // Reverse Azimuth (matching NINA's ReverseAzimuth)
    IUFillSwitch(&ReverseAzS[0], "REV_AZ_ON", "Enabled", ISS_OFF);
    IUFillSwitch(&ReverseAzS[1], "REV_AZ_OFF", "Disabled", ISS_ON);
    IUFillSwitchVector(&ReverseAzSP, ReverseAzS, 2, getDeviceName(), "OAPA_REVERSE_AZ",
                       "Reverse Azimuth", CONFIG_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Reverse Altitude (matching NINA's ReverseAltitude)
    IUFillSwitch(&ReverseAltS[0], "REV_ALT_ON", "Enabled", ISS_OFF);
    IUFillSwitch(&ReverseAltS[1], "REV_ALT_OFF", "Disabled", ISS_ON);
    IUFillSwitchVector(&ReverseAltSP, ReverseAltS, 2, getDeviceName(), "OAPA_REVERSE_ALT",
                       "Reverse Altitude", CONFIG_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Backlash compensation (matching NINA's XBacklashCompensation)
    IUFillNumber(&BacklashN[0], "X_BACKLASH", "Azimuth Backlash", "%6.2f", 0, 100, 0.1, 0);
    IUFillNumberVector(&BacklashNP, BacklashN, 1, getDeviceName(), "OAPA_BACKLASH",
                       "Backlash Compensation", CONFIG_TAB, IP_RW, 0, IPS_IDLE);

    // ─── Motor Tab ─────────────────────────────────────────────
    const char *MOTOR_TAB = "Motor";

    // Motor run current in mA (matching NINA's XC/YC commands)
    IUFillNumber(&MotorCurrentN[0], "X_CURRENT", "Azimuth Current (mA)", "%6.0f", 100, 3000, 50, 800);
    IUFillNumber(&MotorCurrentN[1], "Y_CURRENT", "Altitude Current (mA)", "%6.0f", 100, 3000, 50, 800);
    IUFillNumberVector(&MotorCurrentNP, MotorCurrentN, 2, getDeviceName(), "OAPA_MOTOR_CURRENT",
                       "Run Current", MOTOR_TAB, IP_RW, 0, IPS_IDLE);

    // Motor hold percent (matching NINA's XH/YH commands)
    IUFillNumber(&MotorHoldN[0], "X_HOLD", "Azimuth Hold %", "%3.0f", 0, 100, 5, 50);
    IUFillNumber(&MotorHoldN[1], "Y_HOLD", "Altitude Hold %", "%3.0f", 0, 100, 5, 50);
    IUFillNumberVector(&MotorHoldNP, MotorHoldN, 2, getDeviceName(), "OAPA_MOTOR_HOLD",
                       "Hold Current", MOTOR_TAB, IP_RW, 0, IPS_IDLE);

    // Port definition
    IUFillText(&PortT[0], "PORT", "Port", "/dev/ttyUSB0");
    IUFillTextVector(&PortTP, PortT, 1, getDeviceName(), "DEVICE_PORT", "Ports", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    // Add standard connection properties
    addAuxControls();

    return true;
}

void OAPA::ISGetProperties(const char *dev)
{
    INDI::DefaultDevice::ISGetProperties(dev);

    // If connected, pass these properties to the client
    if (isConnected()) {
        defineProperty(&PositionNP);
        defineProperty(&JogNP);
        defineProperty(&AbsMoveNP);
        defineProperty(&SpeedNP);
        defineProperty(&GearRatioNP);
        defineProperty(&StepsPerDegNP);
        defineProperty(&ReverseAzSP);
        defineProperty(&ReverseAltSP);
        defineProperty(&BacklashNP);
        defineProperty(&MotorCurrentNP);
        defineProperty(&MotorHoldNP);
        defineProperty(&PAAErrorNP);
        defineProperty(&AbortSP);
    }
    defineProperty(&PortTP);
}

bool OAPA::updateProperties()
{
    INDI::DefaultDevice::updateProperties();
    
    if (isConnected()) {
        defineProperty(&PositionNP);
        defineProperty(&JogNP);
        defineProperty(&AbsMoveNP);
        defineProperty(&SpeedNP);
        defineProperty(&GearRatioNP);
        defineProperty(&StepsPerDegNP);
        defineProperty(&ReverseAzSP);
        defineProperty(&ReverseAltSP);
        defineProperty(&BacklashNP);
        defineProperty(&MotorCurrentNP);
        defineProperty(&MotorHoldNP);
        defineProperty(&PAAErrorNP);
        defineProperty(&AbortSP);
    } else {
        deleteProperty(PositionNP.name);
        deleteProperty(JogNP.name);
        deleteProperty(AbsMoveNP.name);
        deleteProperty(SpeedNP.name);
        deleteProperty(GearRatioNP.name);
        deleteProperty(StepsPerDegNP.name);
        deleteProperty(ReverseAzSP.name);
        deleteProperty(ReverseAltSP.name);
        deleteProperty(BacklashNP.name);
        deleteProperty(MotorCurrentNP.name);
        deleteProperty(MotorHoldNP.name);
        deleteProperty(PAAErrorNP.name);
        deleteProperty(AbortSP.name);
    }

#ifdef HAVE_ALIGNMENT_CORRECTION_INTERFACE
    ACI::updateProperties();
#endif
    
    return true;
}

bool OAPA::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);
    IUSaveConfigNumber(fp, &SpeedNP);
    IUSaveConfigNumber(fp, &GearRatioNP);
    IUSaveConfigNumber(fp, &StepsPerDegNP);
    IUSaveConfigSwitch(fp, &ReverseAzSP);
    IUSaveConfigSwitch(fp, &ReverseAltSP);
    IUSaveConfigNumber(fp, &BacklashNP);
    IUSaveConfigNumber(fp, &MotorCurrentNP);
    IUSaveConfigNumber(fp, &MotorHoldNP);
    IUSaveConfigText(fp, &PortTP);
    return true;
}

bool OAPA::Connect()
{
    const char *port = PortT[0].text;
    LOGF_INFO("Attempting to connect to OAPA on %s", port);
    
    // Connect to serial port at 115200 baud, 8N1
    if (tty_connect(port, 115200, 8, 0, 1, &PortFD) != TTY_OK) {
        LOGF_ERROR("Failed to connect to port %s", port);
        return false;
    }
    LOG_INFO("Serial connection opened, waiting for Arduino reset...");
    
    // Wait for 3.0s after connection to allow Arduino Grbl firmware/ESP32 to reset
    usleep(3000000);
    
    // Confirm connection
    if (!Handshake()) {
        LOG_ERROR("Failed to handshake with OAPA");
        tty_disconnect(PortFD);
        PortFD = -1;
        return false;
    }
    
    LOGF_INFO("Connected to OAPA on %s", port);
    
    // Set periodic timer to capture status
    SetTimer(POLL_MS);
    return true;
}

bool OAPA::Disconnect()
{
    m_CorrectionInProgress = false;
    m_MoveInProgress = false;
    m_StuckCount = 0;

    if (PortFD > 0) {
        tty_disconnect(PortFD);
        PortFD = -1;
    }
    
    LOG_INFO("Disconnected from OAPA");
    return true;
}

bool OAPA::Handshake()
{
    char buf[512];
    int nbytes = 0;
    
    // Purge any stale data from serial buffer instead of blocking read loop
    tcflush(PortFD, TCIOFLUSH);

    // Send standard GRBL reset/status command
    LOG_INFO("Sending ? to initiate handshake");
    sendCommand("?");
    
    // Wait for response, reading chunks since Grbl prints a welcome message sometimes
    int totalBytes = 0;
    int retries = 50; // Max 50 chunks to prevent infinite loop
    while(totalBytes < (int)sizeof(buf) - 2 && retries-- > 0) {
        int bytes_read = 0;
        if (tty_read(PortFD, buf + totalBytes, 1, 1, &bytes_read) == TTY_OK && bytes_read > 0) {
             totalBytes += bytes_read;
             buf[totalBytes] = '\0';
             if (strstr(buf, "<") != nullptr) {
                 LOGF_INFO("Handshake success, received: %s", buf);
                 // clear remaining buffer
                 tcflush(PortFD, TCIOFLUSH);
                 return true;
             }
        } else {
             break; // Timeout or error
        }
    }
    
    LOGF_ERROR("Handshake read timeout or error. Buffer contents: %s", buf);
    return false;
}

void OAPA::sendCommand(const char *cmd)
{
    if (PortFD < 0) return;
    
    char sendBuf[128];
    snprintf(sendBuf, sizeof(sendBuf), "%s\n", cmd);
    
    int nbytes;
    tty_write(PortFD, sendBuf, strlen(sendBuf), &nbytes);
}

void OAPA::jogAxis(const char *axis, double units, double speed)
{
    // Apply gear ratio
    double gearRatio = 1.0;
    bool reversed = false;

    if (strcmp(axis, "X") == 0) {
        gearRatio = GearRatioN[0].value;
        reversed = (ReverseAzS[0].s == ISS_ON);
    } else if (strcmp(axis, "Y") == 0) {
        gearRatio = GearRatioN[1].value;
        reversed = (ReverseAltS[0].s == ISS_ON);
    }

    double scaledUnits = units * gearRatio;
    if (reversed)
        scaledUnits = -scaledUnits;

    // Track direction for backlash
    Direction newDir = (scaledUnits >= 0) ? DIR_POSITIVE : DIR_NEGATIVE;
    if (strcmp(axis, "X") == 0) {
        Direction oldDir = m_LastXDirection;
        m_LastXDirection = newDir;
        // Check if direction changed and backlash is configured
        if (oldDir != DIR_NONE && oldDir != newDir && fabs(BacklashN[0].value) > 0) {
            LOGF_INFO("X direction changed, applying backlash compensation: %.2f", BacklashN[0].value);
            applyBacklashCompensation(axis, speed);
        }
    } else if (strcmp(axis, "Y") == 0) {
        m_LastYDirection = newDir;
    }

    char cmd[128];
    snprintf(cmd, sizeof(cmd), "$J=G91G21%s%.2fF%.0f", axis, scaledUnits, speed);
    sendCommand(cmd);
    LOGF_INFO("Jogging %s: %.2f (raw=%.2f, ratio=%.2f, rev=%d) at F%.0f",
              axis, scaledUnits, units, gearRatio, reversed, speed);
}

void OAPA::jogAxisAbsolute(const char *axis, double position, double speed)
{
    // Apply gear ratio
    double gearRatio = 1.0;
    bool reversed = false;

    if (strcmp(axis, "X") == 0) {
        gearRatio = GearRatioN[0].value;
        reversed = (ReverseAzS[0].s == ISS_ON);
    } else if (strcmp(axis, "Y") == 0) {
        gearRatio = GearRatioN[1].value;
        reversed = (ReverseAltS[0].s == ISS_ON);
    }

    double target = position * gearRatio;
    if (reversed)
        target = -target;

    // Track direction based on current position
    double currentPos = (strcmp(axis, "X") == 0) ? PositionN[0].value : PositionN[1].value;
    Direction newDir = (target >= currentPos) ? DIR_POSITIVE : DIR_NEGATIVE;
    if (strcmp(axis, "X") == 0) {
        Direction oldDir = m_LastXDirection;
        m_LastXDirection = newDir;
        if (oldDir != DIR_NONE && oldDir != newDir && fabs(BacklashN[0].value) > 0) {
            LOGF_INFO("X direction changed on abs move, applying backlash compensation: %.2f", BacklashN[0].value);
            applyBacklashCompensation(axis, speed);
        }
    } else if (strcmp(axis, "Y") == 0) {
        m_LastYDirection = newDir;
    }

    char cmd[128];
    snprintf(cmd, sizeof(cmd), "$J=G53%s%.2fF%.0f", axis, target, speed);
    sendCommand(cmd);
    LOGF_INFO("Absolute move %s to %.2f (gear-scaled, rev=%d) at F%.0f", axis, target, reversed, speed);
}

void OAPA::applyBacklashCompensation(const char *axis, double speed)
{
    // Matching NINA's ClearBacklash: overshoot then return
    double bl = BacklashN[0].value;
    char cmd1[128], cmd2[128];
    snprintf(cmd1, sizeof(cmd1), "$J=G91G21%s%.2fF%.0f", axis, -bl, speed);
    snprintf(cmd2, sizeof(cmd2), "$J=G91G21%s%.2fF%.0f", axis, bl, speed);
    sendCommand(cmd1);
    usleep(200000);  // Brief pause between backlash moves
    sendCommand(cmd2);
}

bool OAPA::updateDeviceStatus()
{
    if (PortFD < 0) return false;
    
    // Send status query
    sendCommand("?");
    
    char buf[256];
    int nbytes;
    
    // Read up to newline
    if (tty_read_section(PortFD, buf, '\n', 1, &nbytes) == TTY_OK) {
        buf[nbytes] = '\0';
        
        // Typical GRBL status: <Idle|MPos:0.000,0.000,0.000|...>
        // Extended: <Idle|MPos:x,y,z|T:target,R:running,E:endstop,S:speed>
        if (buf[0] == '<') {
            // Extract status string (e.g., "Idle", "Run", "Jog")
            char *statusEnd = strchr(buf + 1, '|');
            if (statusEnd) {
                int len = statusEnd - (buf + 1);
                if (len > 0 && len < (int)sizeof(m_GrblStatus)) {
                    strncpy(m_GrblStatus, buf + 1, len);
                    m_GrblStatus[len] = '\0';
                }
            }

            bool isIdle = (strstr(buf, "Idle") != nullptr);

            char *mpos = strstr(buf, "MPos:");
            if (mpos) {
                float x = 0, y = 0, z = 0;
                if (sscanf(mpos, "MPos:%f,%f,%f", &x, &y, &z) >= 2) {
                    PositionN[0].value = x;
                    PositionN[1].value = y;
                    
                    IDSetNumber(&PositionNP, nullptr);
                }
            }

#ifdef HAVE_ALIGNMENT_CORRECTION_INTERFACE
            // If a correction was in progress and Grbl is now idle, report completion
            if (m_CorrectionInProgress && isIdle) {
                m_CorrectionInProgress = false;
                LOG_INFO("Alignment correction completed successfully.");
                CorrectionSP.setState(IPS_OK);
                CorrectionSP.reset();
                CorrectionSP.apply();
                CorrectionStatusLP[0].setState(IPS_OK);
                CorrectionStatusLP.apply();
            }
#endif
            return true;
        }
    }
    
    return false;
}

void OAPA::TimerHit()
{
    if (!isConnected()) return;
    
    updateDeviceStatus();

    // ─── Move completion tracking ──────────────────────────────
    if (m_MoveInProgress) {
        double dx = fabs(PositionN[0].value - m_TargetX);
        double dy = fabs(PositionN[1].value - m_TargetY);

        // Check if we've reached target (within 0.01 tolerance, matching NINA)
        if (dx <= 0.01 && dy <= 0.01) {
            m_MoveInProgress = false;
            m_StuckCount = 0;
            LOG_INFO("Move completed: target reached.");
            AbsMoveNP.s = IPS_OK;
            IDSetNumber(&AbsMoveNP, nullptr);
        } else {
            // Check for stuck motor
            static double lastX = 0, lastY = 0;
            if (fabs(PositionN[0].value - lastX) < 0.01 && fabs(PositionN[1].value - lastY) < 0.01) {
                m_StuckCount++;
                if (m_StuckCount > STUCK_THRESHOLD) {
                    m_MoveInProgress = false;
                    m_StuckCount = 0;
                    LOGF_ERROR("Motor appears stuck at X=%.2f Y=%.2f (target X=%.2f Y=%.2f)",
                               PositionN[0].value, PositionN[1].value, m_TargetX, m_TargetY);
                    AbsMoveNP.s = IPS_ALERT;
                    IDSetNumber(&AbsMoveNP, nullptr);
                }
            } else {
                m_StuckCount = 0;
            }
            lastX = PositionN[0].value;
            lastY = PositionN[1].value;

            // Check timeout
            time_t now = time(nullptr);
            if (difftime(now, m_MoveStartTime) > MOVE_TIMEOUT_SEC) {
                m_MoveInProgress = false;
                m_StuckCount = 0;
                LOGF_ERROR("Move timeout after %ds. Current X=%.2f Y=%.2f, target X=%.2f Y=%.2f",
                           MOVE_TIMEOUT_SEC, PositionN[0].value, PositionN[1].value, m_TargetX, m_TargetY);
                AbsMoveNP.s = IPS_ALERT;
                IDSetNumber(&AbsMoveNP, nullptr);
            }
        }
    }
    
    // Re-arm timer
    SetTimer(POLL_MS);
}

bool OAPA::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (strcmp(dev, getDeviceName()) != 0)
        return false;

#ifdef HAVE_ALIGNMENT_CORRECTION_INTERFACE
    // Delegate to AlignmentCorrectionInterface first
    if (ACI::processNumber(dev, name, values, names, n))
        return true;
#endif

    if (strcmp(name, JogNP.name) == 0) {
        JogNP.s = IPS_BUSY;
        IDSetNumber(&JogNP, nullptr);
        
        double x_jog = 0, y_jog = 0;
        
        for (int i = 0; i < n; i++) {
            if (strcmp(names[i], "X_JOG") == 0) {
                x_jog = values[i];
                JogN[0].value = 0; // Reset UI
            } else if (strcmp(names[i], "Y_JOG") == 0) {
                y_jog = values[i];
                JogN[1].value = 0; // Reset UI
            }
        }
        
        // Use per-axis speeds
        if (x_jog != 0)
            jogAxis("X", x_jog, SpeedN[0].value);
        
        if (y_jog != 0)
            jogAxis("Y", y_jog, SpeedN[1].value);
        
        JogNP.s = IPS_OK;
        IDSetNumber(&JogNP, nullptr);
        
        return true;
    }

    // Absolute move
    if (strcmp(name, AbsMoveNP.name) == 0) {
        double x_target = 0, y_target = 0;
        bool doX = false, doY = false;
        for (int i = 0; i < n; i++) {
            if (strcmp(names[i], "X_TARGET") == 0) { x_target = values[i]; doX = true; }
            else if (strcmp(names[i], "Y_TARGET") == 0) { y_target = values[i]; doY = true; }
        }

        AbsMoveNP.s = IPS_BUSY;
        IDSetNumber(&AbsMoveNP, nullptr);

        if (doX)
            jogAxisAbsolute("X", x_target, SpeedN[0].value);
        if (doY)
            jogAxisAbsolute("Y", y_target, SpeedN[1].value);

        // Start move tracking
        m_TargetX = doX ? (x_target * GearRatioN[0].value) : PositionN[0].value;
        m_TargetY = doY ? (y_target * GearRatioN[1].value) : PositionN[1].value;
        if (ReverseAzS[0].s == ISS_ON && doX) m_TargetX = -m_TargetX;
        if (ReverseAltS[0].s == ISS_ON && doY) m_TargetY = -m_TargetY;
        m_MoveInProgress = true;
        m_StuckCount = 0;
        m_MoveStartTime = time(nullptr);

        LOGF_INFO("Absolute move started: target X=%.2f Y=%.2f", m_TargetX, m_TargetY);
        return true;
    }
    
    if (strcmp(name, SpeedNP.name) == 0) {
        IUUpdateNumber(&SpeedNP, values, names, n);
        SpeedNP.s = IPS_OK;
        IDSetNumber(&SpeedNP, nullptr);
        LOGF_INFO("Speed updated: Az=%.0f Alt=%.0f", SpeedN[0].value, SpeedN[1].value);
        return true;
    }

    if (strcmp(name, GearRatioNP.name) == 0) {
        IUUpdateNumber(&GearRatioNP, values, names, n);
        // Enforce minimum of 0.01
        if (GearRatioN[0].value < 0.01) GearRatioN[0].value = 0.01;
        if (GearRatioN[1].value < 0.01) GearRatioN[1].value = 0.01;
        GearRatioNP.s = IPS_OK;
        IDSetNumber(&GearRatioNP, nullptr);
        LOGF_INFO("Gear ratio updated: X=%.2f Y=%.2f", GearRatioN[0].value, GearRatioN[1].value);
        return true;
    }

    if (strcmp(name, StepsPerDegNP.name) == 0) {
        IUUpdateNumber(&StepsPerDegNP, values, names, n);
        StepsPerDegNP.s = IPS_OK;
        IDSetNumber(&StepsPerDegNP, nullptr);
        LOGF_INFO("Calibration updated: Az=%.1f Alt=%.1f steps/deg",
                  StepsPerDegN[0].value, StepsPerDegN[1].value);
        return true;
    }

    if (strcmp(name, BacklashNP.name) == 0) {
        IUUpdateNumber(&BacklashNP, values, names, n);
        BacklashNP.s = IPS_OK;
        IDSetNumber(&BacklashNP, nullptr);
        LOGF_INFO("Backlash compensation updated: %.2f", BacklashN[0].value);
        return true;
    }

    // Motor run current — send XC/YC commands to firmware
    if (strcmp(name, MotorCurrentNP.name) == 0) {
        IUUpdateNumber(&MotorCurrentNP, values, names, n);
        MotorCurrentNP.s = IPS_OK;
        IDSetNumber(&MotorCurrentNP, nullptr);
        if (isConnected()) {
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "XC%d", (int)MotorCurrentN[0].value);
            sendCommand(cmd);
            snprintf(cmd, sizeof(cmd), "YC%d", (int)MotorCurrentN[1].value);
            sendCommand(cmd);
            LOGF_INFO("Motor run current set: X=%dmA Y=%dmA",
                      (int)MotorCurrentN[0].value, (int)MotorCurrentN[1].value);
        }
        return true;
    }

    // Motor hold percent — send XH/YH commands to firmware
    if (strcmp(name, MotorHoldNP.name) == 0) {
        IUUpdateNumber(&MotorHoldNP, values, names, n);
        MotorHoldNP.s = IPS_OK;
        IDSetNumber(&MotorHoldNP, nullptr);
        if (isConnected()) {
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "XH%d", (int)MotorHoldN[0].value);
            sendCommand(cmd);
            snprintf(cmd, sizeof(cmd), "YH%d", (int)MotorHoldN[1].value);
            sendCommand(cmd);
            LOGF_INFO("Motor hold percent set: X=%d%% Y=%d%%",
                      (int)MotorHoldN[0].value, (int)MotorHoldN[1].value);
        }
        return true;
    }

    // PAA Error: convert degrees to steps and auto-correct
    if (strcmp(name, PAAErrorNP.name) == 0) {
        double az_err = 0, alt_err = 0;
        for (int i = 0; i < n; i++) {
            if (strcmp(names[i], "AZ_ERR") == 0) az_err = values[i];
            else if (strcmp(names[i], "ALT_ERR") == 0) alt_err = values[i];
        }
        double az_steps  = -az_err  * StepsPerDegN[0].value;
        double alt_steps = -alt_err * StepsPerDegN[1].value;
        LOGF_INFO("PAA correction: az_err=%.4f deg -> %.0f steps | alt_err=%.4f deg -> %.0f steps",
                  az_err, az_steps, alt_err, alt_steps);
        PAAErrorNP.s = IPS_BUSY;
        IDSetNumber(&PAAErrorNP, nullptr);
        if (az_steps != 0)  jogAxis("X", az_steps, SpeedN[0].value);
        if (alt_steps != 0) jogAxis("Y", alt_steps, SpeedN[1].value);
        PAAErrorNP.s = IPS_OK;
        IDSetNumber(&PAAErrorNP, nullptr);
        return true;
    }

    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool OAPA::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (strcmp(dev, getDeviceName()) != 0)
        return false;

#ifdef HAVE_ALIGNMENT_CORRECTION_INTERFACE
    // Delegate to AlignmentCorrectionInterface first
    if (ACI::processSwitch(dev, name, states, names, n))
        return true;
#endif

    if (strcmp(name, AbortSP.name) == 0) {
        IUUpdateSwitch(&AbortSP, states, names, n);
        
        if (AbortS[0].s == ISS_ON) {
            // GRBL abort command: Feed Hold (!) followed by Reset (\x18)
            sendCommand("!");
            // Wait a little, then issue soft-reset
            usleep(50000); 
            char resetCmd[2] = {0x18, 0};
            sendCommand(resetCmd);
            
            m_CorrectionInProgress = false;
            m_MoveInProgress = false;
            m_StuckCount = 0;
            LOG_INFO("Motion aborted.");
            
            AbortS[0].s = ISS_OFF;
            AbortSP.s = IPS_OK;
            IDSetSwitch(&AbortSP, nullptr);
        }
        
        return true;
    }

    // Reverse Azimuth
    if (strcmp(name, ReverseAzSP.name) == 0) {
        IUUpdateSwitch(&ReverseAzSP, states, names, n);
        ReverseAzSP.s = IPS_OK;
        IDSetSwitch(&ReverseAzSP, nullptr);
        LOGF_INFO("Reverse Azimuth: %s", (ReverseAzS[0].s == ISS_ON) ? "Enabled" : "Disabled");
        return true;
    }

    // Reverse Altitude
    if (strcmp(name, ReverseAltSP.name) == 0) {
        IUUpdateSwitch(&ReverseAltSP, states, names, n);
        ReverseAltSP.s = IPS_OK;
        IDSetSwitch(&ReverseAltSP, nullptr);
        LOGF_INFO("Reverse Altitude: %s", (ReverseAltS[0].s == ISS_ON) ? "Enabled" : "Disabled");
        return true;
    }

    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool OAPA::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (strcmp(dev, getDeviceName()) != 0)
        return false;

    if (strcmp(name, PortTP.name) == 0) {
        IUUpdateText(&PortTP, texts, names, n);
        PortTP.s = IPS_OK;
        IDSetText(&PortTP, nullptr);
        return true;
    }

    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

// ═══════════════════════════════════════════════════════════════
//  AlignmentCorrectionInterface Implementation
// ═══════════════════════════════════════════════════════════════

#ifdef HAVE_ALIGNMENT_CORRECTION_INTERFACE

IPState OAPA::StartCorrection(double azError, double altError)
{
    if (!isConnected()) {
        LOG_ERROR("Cannot start correction: not connected.");
        return IPS_ALERT;
    }

    if (m_CorrectionInProgress) {
        LOG_WARN("Correction already in progress.");
        return IPS_BUSY;
    }

    double stepsAz  = StepsPerDegN[0].value;
    double stepsAlt = StepsPerDegN[1].value;
    double speed    = SpeedN[0].value;

    // Convert error in degrees to motor jog units
    // Negative sign: we want to CORRECT the error, not add to it
    double jogAz  = -azError  * stepsAz;
    double jogAlt = -altError * stepsAlt;

    LOGF_INFO("Starting alignment correction: Az error=%.4f° (jog %.2f), Alt error=%.4f° (jog %.2f)",
              azError, jogAz, altError, jogAlt);

    if (fabs(jogAz) > 0.01)
        jogAxis("X", jogAz, speed);

    if (fabs(jogAlt) > 0.01)
        jogAxis("Y", jogAlt, speed);

    m_CorrectionInProgress = true;

    // TimerHit/updateDeviceStatus will detect Grbl Idle state
    // and set CorrectionSP/CorrectionStatusLP to OK automatically
    return IPS_BUSY;
}

IPState OAPA::AbortCorrection()
{
    if (!isConnected()) {
        LOG_ERROR("Cannot abort: not connected.");
        return IPS_ALERT;
    }

    // GRBL feed-hold + soft-reset
    sendCommand("!");
    usleep(50000);
    char resetCmd[2] = {0x18, 0};
    sendCommand(resetCmd);

    m_CorrectionInProgress = false;
    LOG_INFO("Alignment correction aborted.");

    return IPS_OK;
}

#endif // HAVE_ALIGNMENT_CORRECTION_INTERFACE
