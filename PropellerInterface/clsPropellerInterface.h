
#ifndef MBED_H
#include "mbed.h"
#endif

#define PROPELLER_DEBUG 0
#define PROPELLER_DEBUG_VALIDATE_PACKET 0
#define PROPELLER_DEBUG_HIGHLEVEL 0
#define PROPELLER_LOG_REPLY_FROM_PROPELLER 0

#define TELNETBUFFERSIZE 80

class clsPropellerInterface {
    private:
        BusInOut    *m_bus_PropellerDataBUS;
        DigitalIn   *m_in_PropellerControlRX;
        DigitalOut  *m_out_PropellerControlTX;
        PwmOut      *_statusLed;
        
        void        TX(char* strPacket, int intPacketLength);
        int         RX();
        int         intValidatePacket(char *strData);
        int         FindCharPosition(char *data, int length, int searchValue, int startPosition);
        
    public:
        char        m_strReply[255];
        int         m_intLastPacketRXLength;
                
        void        SetupPropellerInterface(BusInOut *bus_PropellerDataBUS, DigitalIn *in_PropellerControlRX, DigitalOut *out_PropellerControlTX, PwmOut *statusLed);
        int         intTX(char* strPacket, int intPacketLength);
        long        lngSendCommand(int intCommand, int intAxis, long lngParameterValue);
        long        lngDecodeBase128ValueInReply();
        
};
