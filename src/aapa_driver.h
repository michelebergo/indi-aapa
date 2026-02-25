#ifndef AAPA_DRIVER_H
#define AAPA_DRIVER_H

#include <libindi/defaultdevice.h>
#include <libindi/inditimer.h>

class AAPA : public INDI::DefaultDevice
{
public:
    AAPA();
    virtual ~AAPA() = default;

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

private:
    bool updateDeviceStatus();
    void sendCommand(const char *cmd);

    // Number properties for actual positions
    INumberVectorProperty PositionNP;
    INumber PositionN[2];

    // Number properties for relative jogging
    INumberVectorProperty JogNP;
    INumber JogN[2];
    
    // Number property for Feed Rate/Speed
    INumberVectorProperty SpeedNP;
    INumber SpeedN[1];

    // Switch property to stop motion
    ISwitchVectorProperty AbortSP;
    ISwitch AbortS[1];

    ITextVectorProperty PortTP;
    IText PortT[1];

    int PortFD;
};

#endif // AAPA_DRIVER_H
