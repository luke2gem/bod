#ifndef MBED_H
#include "mbed.h"
#endif

#include "clsNetworkInterface.h"

// Class definition for the ethernet to serial code
class EthernetToSerial {
    private:
        // Variables & Objects
        Serial              *_serialPort;
        clsNetworkInterface *_networkInterface;
        
        // Circular buffers for serial TX and RX data - used by interrupt routines
        static const int    BUFFER_SIZE = 1024;
        char                _rxBuffer[BUFFER_SIZE];
        
        // Circular buffer pointers volatile makes read-modify-write atomic 
        volatile int        _inPointer;
        volatile int        _outPointer;

        // Methods
        
        
    public:
        // Constructor
        EthernetToSerial(clsNetworkInterface* networkInterface, Serial* serialPort, int ethernetPort) 
        {
            _networkInterface = networkInterface;
            _serialPort = serialPort;
            _inPointer = 0;
            _outPointer = 0;
            
        }
        
        // Destructor
        virtual ~EthernetToSerial()
        {
        
        }
        
        // Methods
        void                RxInterrupt();
        void                CheckPortReceiveBuffer();
        void                AttachInterrupts(void (* serialRxHandler)());
};
