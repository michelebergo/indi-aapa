#ifndef OAPA_DRIVER_H
#define OAPA_DRIVER_H

#include <libindi/defaultdevice.h>
#include <libindi/inditimer.h>

// Detect new AlignmentCorrectionInterface (INDI >= 2.1.0, PR #2342)
#if __has_include(<libindi/indialignmentcorrectioninterface.h>)
#include <libindi/indialignmentcorrectioninterface.h>
#define HAVE_ALIGNMENT_CORRECTION_INTERFACE 1
#endif

class OAPA : public INDI::DefaultDevice
#ifdef HAVE_ALIGNMENT_CORRECTION_INTERFACE
    , public INDI::AlignmentCorrectionInterface
#endif
{
    // Alias for shorter notation in .cpp
#ifdef HAVE_ALIGNMENT_CORRECTION_INTERFACE
    using ACI = INDI::AlignmentCorrectionInterface;
#endif

public:
    OAPA();
    virtual ~OAPA() = default;

    virtual const char *getDefaultName() override;
    virtual bool initProperties() override;
    virtual void ISGetProperties(const char *dev) override;
    virtual bool updateProperties() override;
    virtual bool saveConfigItems(FILE *fp) override;

    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;

protected:
    virtual bool Connect() override;
    virtual bool Disconnect() override;
    virtual void TimerHit() override;
    virtual bool Handshake();

#ifdef HAVE_ALIGNMENT_CORRECTION_INTERFACE
    // From AlignmentCorrectionInterface
    virtual IPState StartCorrection(double azError, double altError) override;
    virtual IPState AbortCorrection() override;
#endif

private:
    bool updateDeviceStatus();
    void sendCommand(const char *cmd);
    void jogAxis(const char *axis, double units, double speed);
    void jogAxisAbsolute(const char *axis, double position, double speed);
    void applyBacklashCompensation(const char *axis, double speed);

    // ─── Position (Read Only) ──────────────────────────────────
    INumberVectorProperty PositionNP;
    INumber PositionN[2];

    // ─── Relative Jog (Write Only) ─────────────────────────────
    INumberVectorProperty JogNP;
    INumber JogN[2];

    // ─── Absolute Move (Write Only) ────────────────────────────
    INumberVectorProperty AbsMoveNP;
    INumber AbsMoveN[2];

    // ─── Per-axis Speed ────────────────────────────────────────
    INumberVectorProperty SpeedNP;
    INumber SpeedN[2];  // [0] = X/Azimuth speed, [1] = Y/Altitude speed

    // ─── Gear Ratio ────────────────────────────────────────────
    INumberVectorProperty GearRatioNP;
    INumber GearRatioN[2];  // [0] = X ratio, [1] = Y ratio

    // ─── Steps-Per-Degree calibration ──────────────────────────
    INumberVectorProperty StepsPerDegNP;
    INumber StepsPerDegN[2];

    // ─── Reverse Axis ──────────────────────────────────────────
    // Two independent on/off switches
    ISwitchVectorProperty ReverseAzSP;
    ISwitch ReverseAzS[2];   // ON / OFF

    ISwitchVectorProperty ReverseAltSP;
    ISwitch ReverseAltS[2];  // ON / OFF

    // ─── Backlash Compensation ─────────────────────────────────
    INumberVectorProperty BacklashNP;
    INumber BacklashN[1];  // X-axis backlash (matching NINA)

    // ─── Motor Run Current (mA) ────────────────────────────────
    INumberVectorProperty MotorCurrentNP;
    INumber MotorCurrentN[2];  // [0] = X, [1] = Y

    // ─── Motor Hold Percent ────────────────────────────────────
    INumberVectorProperty MotorHoldNP;
    INumber MotorHoldN[2];  // [0] = X, [1] = Y

    // ─── PAA Error Input ───────────────────────────────────────
    INumberVectorProperty PAAErrorNP;
    INumber PAAErrorN[2];

    // ─── Abort Button ──────────────────────────────────────────
    ISwitchVectorProperty AbortSP;
    ISwitch AbortS[1];

    // ─── Serial Port ───────────────────────────────────────────
    ITextVectorProperty PortTP;
    IText PortT[1];

    int PortFD;

    // ─── Movement tracking (for completion + backlash) ─────────
    bool m_CorrectionInProgress{false};
    bool m_MoveInProgress{false};
    double m_TargetX{0};
    double m_TargetY{0};
    int m_StuckCount{0};
    time_t m_MoveStartTime{0};

    // Direction tracking for backlash compensation
    enum Direction { DIR_NONE, DIR_POSITIVE, DIR_NEGATIVE };
    Direction m_LastXDirection{DIR_NONE};
    Direction m_LastYDirection{DIR_NONE};

    // Grbl status string
    char m_GrblStatus[32];
};

#endif // OAPA_DRIVER_H
