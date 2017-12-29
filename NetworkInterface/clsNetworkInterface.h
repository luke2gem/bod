#ifndef MBED_H
#include "mbed.h"
#endif

#ifndef NETWORK_H
#define NETWORK_H 1

#define TELNET_DEBUG 0
#define TELNETBUFFERSIZE 50 // Keeping this at 254 or below because you don't want it larger than the serial buffer
#define NETWORK_DEBUG_VALIDATE_PACKET 0

// vvvvvvvvvvv ETHERNET vvvvvvvvvvv
// Import library from: 
// http://mbed.org/projects/cookbook/svn/EMAC/lwip/trunk/Core
#include "lwip/opt.h"
#include "lwip/stats.h"
#include "lwip/sys.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"
#include "lwip/dhcp.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "netif/etharp.h"
#include "netif/loopif.h"
#include "device.h"
// ^^^^^^^^^^^ ETHERNET ^^^^^^^^^^^

err_t recv_callback(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
err_t recv_callbackSerialPort1(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
err_t recv_callbackSerialPort2(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
err_t recv_callbackSerialPort3(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);

err_t accept_callback(void *arg, struct tcp_pcb *npcb, err_t err);
err_t accept_callbackSerialPort1(void *arg, struct tcp_pcb *npcb, err_t err);
err_t accept_callbackSerialPort2(void *arg, struct tcp_pcb *npcb, err_t err);
err_t accept_callbackSerialPort3(void *arg, struct tcp_pcb *npcb, err_t err);
        
class clsNetworkInterface {
    private:
        // ETHERNET OBJETS
        struct netif    netif_data;
        
        // TELNET VARIABLES
        char            m_strCommsBuffer[TELNETBUFFERSIZE];        // Buffer array used for incoming data on the telnet port
        int             m_intCommsBufferIndex;                 // Current position within the telnet RX buffer
        void            (* PacketReceived)(char *strReceivedData, int intNodeAddress, int intPacketLength);
        
        int             m_intLastPacketRXLength;
        int             FindCharPosition(char *data, int length, int searchValue, int startPosition);
        
    public:
        struct tcp_pcb  *m_objClientConnection;
        struct tcp_pcb  *_ethernetSerialPort1;
        struct tcp_pcb  *_ethernetSerialPort2;
        struct tcp_pcb  *_ethernetSerialPort3;
        
        void            (* EthernetSerialPortDataReceived)(int portnum, char *data, int length);
        void            SendSerialData(int port, char *data, int length);
        int             m_arrIPAddress[4];
        char            m_strCommsInputTemp[TELNETBUFFERSIZE]; // Temp buffer array used for telnet data manipulation
        
        // Constructor
        clsNetworkInterface(
                                void (* fncFunctionToCallWhenPacketReceived)(char *strReceivedData, int intNodeAddress, int intPacketLength),
                                void (* fncFunctionToCallWhenEthernetSerialDataRX)(int portnum, char *data, int length)
                           ) 
        {
            PacketReceived = fncFunctionToCallWhenPacketReceived;
            EthernetSerialPortDataReceived = fncFunctionToCallWhenEthernetSerialDataRX;
            _ethernetSerialPort1 = NULL;
            _ethernetSerialPort2 = NULL;
            _ethernetSerialPort3 = NULL;
        }
        
        // Destructor
        virtual ~clsNetworkInterface() { 
        
        }
        
        void SetupTCP(int intUseDHCP);
        void SendReplyValue(long lngValue);
        long lngDecodeBase128ValueInReply(int intStartChar);
        int intParseTelnetData();
        int intValidatePacket(char *strData);
        void SendReply(char *strData, int intDataLength);
};

// ETHERNET OBJETS
extern clsNetworkInterface *m_objNetworkInterface;
#endif
