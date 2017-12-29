
#include "mbed.h"
#include "ConfigFile.h"
#include "EthernetToSerial.h"
#include "clsNetworkInterface.h"
#include "clsPropellerInterface.h"

/* Propeller commands */
#define HomeAxis = 4
#define MoveABS = 8
#define MoveINC = 12
#define StopAxis = 16
#define IsAxisBusy = 20
#define GetEncoderPosition = 24
#define GetLogicalPosition = 28
#define SetStartSpeed = 32
#define SetDriveSpeed = 36
#define SetHomeSpeed = 40
#define SetAccelerationRate = 44
#define SetMotorDirection = 48
#define SetEncoderDirection = 52
#define GetInitialSpeed = 56
#define GetDriveSpeed = 60
#define GetHomeSpeed = 64
#define GetAccelerationRate = 68
#define GetMotorDirection = 72
#define GetEncoderDirection = 76
#define SetEncoderPosition = 80
#define SetLogicalPosition = 84
#define LineMoveABS = 88
#define LineMoveINC = 92
#define QueryHomeInput = 96
#define QueryHomeStatus = 100
#define SwitchAuxOutput = 104
#define SetHomeTimeout = 108
#define SetHomeOffsetSpeed = 112
#define SetHomeChangeDirectionDelay = 116
#define ToggleEnableLine = 120
#define GetESTOPState = 200
#define ClearESTOPState = 204
#define QueryResetFlag = 208
#define ClearResetFlag = 212
#define GetVersionInfo = 216
#define MoveContinuous = 220
#define SlowStop = 224
/* End of Propeller commands */

// I/O CONFIG
PwmOut                  _led1(LED1);
PwmOut                  _led2(LED2);
PwmOut                  _led3(LED3);
PwmOut                  _led4(LED4);
        
// COMMS OBJECTS
Serial                  pc(USBTX, USBRX);

// I/O VARIABLES
char                    m_bytOutputs[1];               // Array of output state bytes
DigitalOut              out_OutputLatch(p6);
DigitalOut              out_InputLatch(p7);
DigitalOut              out_InputCS1(p5);
DigitalOut              out_InputCS2(p8);

// SERIAL VARIABLES
Serial                  serial_COM1(p9, p10); // tx, rx
Serial                  serial_COM2(p28, p27); // tx, rx
Serial                  serial_COM3(p13, p14); // tx, rx
DigitalOut              out_COM3_485CS(p17);
EthernetToSerial        *_ethernetToSerial[5];

// PROPELLER INTERFACE
BusInOut                bus_DataBUS(p21, p22, p23, p24, p25, p26, p16, p15);
DigitalIn               in_PropellerControlRX(p11);
DigitalOut              out_PropellerControlTX(p12);

// NETWORK INTERFACE
clsNetworkInterface     *m_objNetworkInterface; // Note: this should be named the same as in the clsNetworkInterface library
clsPropellerInterface   *m_objPropellerInterface;

// CONFIG FILE
LocalFileSystem         m_objFileSystem("local");
ConfigFile              m_objConfigFile;

// SERIAL PORT BUFFERS
// Circular buffers for serial TX and RX data - used by interrupt routines
const int buffer_size = 1024;
char serial_rx_buffer[4][buffer_size];

// Circular buffer pointers volatile makes read-modify-write atomic 
volatile int serial_in_pointer[4];
volatile int serial_out_pointer[4];

// FUNCTION PROTOTYPES
void ReadConfigFile();
void SetupTCP(int intUseDHCP);
void SetupIO();
void TCPPacketReceived(char *strCommsBuffer, int intNodeAddress, int intPacketLength);
void EthernetSerialPortDataReceived(int portnum, char *data, int length);
void ProcessLoop_CheckSerialPorts();
void ProcessLoop_SetOutputStates();
int intReadInputState(int bank);
void serial_COM1_Rx_interrupt();
void serial_COM2_Rx_interrupt();
void serial_COM3_Rx_interrupt();

int _vibrateAxis1 = 0;
int _vibrateAxis2 = 0;
int _vibrateAxis1State = 0;
int _vibrateAxis2State = 0;
long _vibrateAxis1Distance = 0;
long _vibrateAxis2Distance = 0;

Timer _inputStateCheckTimer;
Timer _inputStateChangedTimer;
bool _lastInputState;


long lngSendCommand(int intCommand, int intAxis, long lngParameterValue);

// ===========================================================================================================================================================================================
// MAIN PROCESS LOOP
// ===========================================================================================================================================================================================
int main() {
    _led1.period_us(20);
    _led2.period_us(20);
    _led3.period_us(20);
    _led4.period_us(20);
    _led1 = 1;

    // Setup I/O devices
    SetupIO();
    
    // Set initial variable values
    for (int i=0; i<1; i++) {
        m_bytOutputs[i] = 0x00;
    }

    // Set output states
    ProcessLoop_SetOutputStates();

    // Initialise pointer values
    for (int i=0; i<4; i++) {
        serial_in_pointer[i] = 0;
        serial_out_pointer[i] = 0;        
    }

    // Setup a serial interrupt function to receive data for each serial port
    serial_COM1.attach(&serial_COM1_Rx_interrupt, Serial::RxIrq);
    serial_COM2.attach(&serial_COM2_Rx_interrupt, Serial::RxIrq);
    serial_COM3.attach(&serial_COM3_Rx_interrupt, Serial::RxIrq);

    // Set the PC USB serial baud rate.
    pc.baud(115200);
    
    // Create network interface class (note this is before reading config as some settings are written directly into this class instance)
    m_objNetworkInterface = new clsNetworkInterface(&TCPPacketReceived, &EthernetSerialPortDataReceived);
    
    // Create a propeller interface object
    m_objPropellerInterface = new clsPropellerInterface();

    // Create the Ethernet - Serial classes
    //_ethernetToSerial[1] = new EthernetToSerial(m_objNetworkInterface, &serial_COM1, 10001);
    //_ethernetToSerial[2] = new EthernetToSerial(&serial_COM2, 10002);
    //_ethernetToSerial[3] = new EthernetToSerial(&serial_COM3, 10003);
    
    //_ethernetToSerial[1]->AttachInterrupts(&_ethernetToSerial[1].RxInterrupt);
    
    // Read configuration file from flash memory
    ReadConfigFile();

    _led2 = 1;

    // Setup TCP/IP - pass in 0 for static IP, or 1 for DHCP
    m_objNetworkInterface->SetupTCP(0);

    _led3 = 1;

    // Setup propeller interface
    m_objPropellerInterface->SetupPropellerInterface(&bus_DataBUS, &in_PropellerControlRX, &out_PropellerControlTX, &_led3);
    //out_PropellerControlTX.write(1);
    
    _led4 = 1;

    // Talk to the propeller at the start so we know if it is talking ok
    wait(0.1);
    if (m_objPropellerInterface->lngSendCommand(216, 1, NULL) == 0) { // GetVersionInfo
        _led1 = 0;
        _led2 = 0;
            
        while (true) {
            _led3 = 1;
            _led4 = 0;
            wait(0.1);
            _led3 = 0;
            _led4 = 1;
            wait(0.1);
        }
    }
    
    printf("Propeller OK\r\n");

    /*
    m_objPropellerInterface->lngSendCommand(40, 1, 500000); // Home speed
    m_objPropellerInterface->lngSendCommand(108, 1, 3600); // Home timeout
    m_objPropellerInterface->lngSendCommand(112, 1, 100); // Home offset speed
    m_objPropellerInterface->lngSendCommand(116, 1, 1); // Home change direction delay
    m_objPropellerInterface->lngSendCommand(16, 1, NULL); // Stop
    m_objPropellerInterface->lngSendCommand(212, 1, NULL); // Clear reset flag
    m_objPropellerInterface->lngSendCommand(4, 1, NULL); // Home command
    */
    printf("==================================================\n");
    printf("Running...\n");
    printf("--------------------------------------------------\n");
    
    // Fade LEDs off
    double ledState = 1.0;
    double ledFadeIncrement = 0.05;
    while (ledState > 0) {
        wait_ms(20);
        ledState -= ledFadeIncrement;
        _led1 = ledState; _led2 = ledState; _led3 = ledState; _led4 = ledState;
    }
    
    // Setup and start a timer used for activity monitoring
    Timer tmrStatus;
    tmrStatus.start();
    
    // Start the timer looking for input state changes
    _inputStateChangedTimer.start();
    _inputStateCheckTimer.start();
    
    // Main program loop
    while (1) {
        // Set output states
        ProcessLoop_SetOutputStates();
        
        // Poll serial ports
        ProcessLoop_CheckSerialPorts();
        //_ethernetToSerial[1]->CheckPortReceiveBuffer();
        //serial_COM1_Rx_interrupt();
        //serial_COM2_Rx_interrupt();
        //serial_COM3_Rx_interrupt();
        
        /*
        if (_inputStateCheckTimer.read_us() > 10000)
        {
            // Read input state
            int inputState = intReadInputState(0) & (1 << 5);
            if (inputState != _lastInputState) {
                _lastInputState = inputState;
                _inputStateChangedTimer.reset();
            }
            
            // Reset the checker timer
            _inputStateCheckTimer.reset();
        }
        */
                
        // Poll network interface
        device_poll();
         
         /*
        if (_vibrateAxis1 > 0)
        {
            int busyState = m_objPropellerInterface->lngSendCommand(20, 1, NULL);
            if (busyState == 0)
            {
                if (_vibrateAxis1State == 0)
                {
                    m_objPropellerInterface->lngSendCommand(12, 1, _vibrateAxis1Distance);
                } else {
                    m_objPropellerInterface->lngSendCommand(12, 1, -_vibrateAxis1Distance);
                }
                
                _vibrateAxis1State++;
                if (_vibrateAxis1State > 1) { _vibrateAxis1State = 0; }    
            }       
        }
        
        if (_vibrateAxis2 > 0)
        {
            int busyState = m_objPropellerInterface->lngSendCommand(20, 2, NULL);
            if (busyState == 0)
            {
                if (_vibrateAxis2State == 0)
                {
                    m_objPropellerInterface->lngSendCommand(12, 2, _vibrateAxis2Distance);
                } else {
                    m_objPropellerInterface->lngSendCommand(12, 2, -_vibrateAxis2Distance);
                }
                
                _vibrateAxis2State++;
                if (_vibrateAxis2State > 1) { _vibrateAxis2State = 0; }
            }
        }
         */
         
        // Flash status lights
        if(tmrStatus.read_ms() > 20) {
            ledState += ledFadeIncrement;
            if (ledFadeIncrement > 0 && ledState >= 1) { ledFadeIncrement *= -1;  }
            if (ledFadeIncrement < 0 && ledState <= 0) { ledFadeIncrement *= -1;  }
            
            _led1 = ledState;
            tmrStatus.reset();
        }
    }
}

// ===========================================================================================================================================================================================

// This function is called when we received a complete & validated packet from the host controller via TCP
void TCPPacketReceived(char *strCommsBuffer, int intNodeAddress, int intPacketLength) {
    int     intCMD=0;
    long    lngValue;
    int     bitPosition;
    int     intPortState;
    
    _led2 = 1;
    
    // Parse out the command
    intCMD = (int)((char*)&strCommsBuffer[2])[0];
    if (TELNET_DEBUG) { printf("Received Command: %d, Node Address: %d\n", intCMD, intNodeAddress); }
    //printf("Received Command: %d, Node Address: %d\n", intCMD, intNodeAddress);

    // Check the node address of the packet
    if (intNodeAddress == 1) {
        // Send packet to the propeller
        if (PROPELLER_DEBUG_HIGHLEVEL) { printf("Command received for the propeller: (%d) %s\n", intPacketLength, strCommsBuffer); }
        // Send the received command to the propeller
        int intReplyLength = m_objPropellerInterface->intTX(strCommsBuffer, intPacketLength);
        
        // If the packet sent and a reply received
        if (intReplyLength > 0) {
            // Send the reply back to the TCP client
            m_objNetworkInterface->SendReply(m_objPropellerInterface->m_strReply, intReplyLength);
            _led2 = 0;
            return;
        }
        
    } else if (intNodeAddress == 3) {
        if (PROPELLER_DEBUG_HIGHLEVEL) { printf("Command for node the mbed!\n"); }
        //printf("Command for node the mbed!\n");
        switch (intCMD) {
            case 1:
                // Change IP address
                /*
                // Set a configuration value.
                if (!cfg.setValue("MyKey", "TestValue")) {
                    error("Failure to set a value.\n");
                }    
                // Set a configuration value.
                if (!cfg.write("/local/config.cfg")) {
                    error("Failure to write a configuration file.\n");
                }
                */
                break;
                
            case 32:
                // Extract the required output state from the packet
                lngValue = m_objNetworkInterface->lngDecodeBase128ValueInReply(3);
                
                // Set the output bits
                m_bytOutputs[0] = (char)lngValue; 
                //m_bytOutputs[1] = (char)(lngValue>>8);
                //m_bytOutputs[2] = (char)(lngValue>>16);
                //m_bytOutputs[3] = (char)(lngValue>>24);
                
                // Reply with an OK
                m_objNetworkInterface->SendReplyValue(1);
                break;
                
            case 228:
                // Parse input state
                //lngValue = m_objNetworkInterface->lngDecodeBase128ValueInReply(3);
                
                // Read input states
                int inputs0;
                inputs0 = intReadInputState(0);
                int inputs1;
                inputs1 = intReadInputState(1);
                
                // Set port state value
                intPortState = inputs0 + (inputs1 << 8);
                //printf("Input State: %d\n", intPortState);
                
                // Reply with an OK
                m_objNetworkInterface->SendReplyValue(intPortState);
                break;
                
            case 229:
                // Parse required output
                lngValue = m_objNetworkInterface->lngDecodeBase128ValueInReply(3);
                bitPosition = 8 - (int)lngValue;
                
                // Switch output on
                m_bytOutputs[0] |= (1 << bitPosition);
                
                // Reply with an OK
                m_objNetworkInterface->SendReplyValue(1);
                break;
                
            case 230:
                // Parse required output
                lngValue = m_objNetworkInterface->lngDecodeBase128ValueInReply(3);
                bitPosition = 8 - (int)lngValue;
                
                // Switch output off
                m_bytOutputs[0] &= ~(1 << bitPosition);
                
                // Reply with an OK
                m_objNetworkInterface->SendReplyValue(1);
                break;

            case 231: // START VIBRATE AXIS 1
                // Parse required position
                lngValue = m_objNetworkInterface->lngDecodeBase128ValueInReply(3);
                
                _vibrateAxis1Distance = lngValue;
                _vibrateAxis1State = 0;
                _vibrateAxis1 = 1;
                
                // Reply with an OK
                m_objNetworkInterface->SendReplyValue(1);
                break;

            case 232: // START VIBRATE AXIS 2
                // Parse required position
                lngValue = m_objNetworkInterface->lngDecodeBase128ValueInReply(3);
                
                _vibrateAxis2Distance = lngValue;
                _vibrateAxis2State = 0;
                _vibrateAxis2 = 1;
                                
                // Reply with an OK
                m_objNetworkInterface->SendReplyValue(1);
                break;

            case 233: // STOP VIBRATE AXIS 1
                _vibrateAxis1 = 0;                
                
                // Reply with an OK
                m_objNetworkInterface->SendReplyValue(1);
                break;

            case 234: // STOP VIBRATE AXIS 2
                _vibrateAxis2 = 0;
                
                // Reply with an OK
                m_objNetworkInterface->SendReplyValue(1);
                break;

            case 235: // TIME SINCE INPUT STATE LAST CHANGED
                long time;
                time = _inputStateChangedTimer.read_ms();
                
                // Reply with the time since the input state last changed
                m_objNetworkInterface->SendReplyValue(time);
                break;

            default:
                printf("Parameter not found\n");
                m_objNetworkInterface->SendReplyValue(-1);
                break;
        }
    }
    
    _led2 = 0;        
}

// Read input state
int intReadInputState(int bank) {
    // Set input select pin for the required input bank
    if (bank == 1) {
        out_InputCS1 = 0;
    } else {
        out_InputCS2 = 0;
    }
    
    // Toggle latch pin
    out_InputLatch = 1;
    wait_us(25);
    
    // Read databus
    int data = bus_DataBUS;

    // Reset input select pins and latch line
    out_InputLatch = 0;
    out_InputCS1 = 1;
    out_InputCS2 = 1;

    // Return the result
    return (~data) & 0xFF;
}

// Process loop function that sets the I/O expander output states to the value stored in the output array
void ProcessLoop_SetOutputStates() {
    bus_DataBUS.output();
    bus_DataBUS.write(m_bytOutputs[0]);
    out_OutputLatch.write(1);
    wait_us(25);
    out_OutputLatch.write(0);
    bus_DataBUS.input();
    wait_us(25);
}

// ===========================================================================================================================================================================================

// Interupt routine to read in data from serial port one when it arrives
void serial_COM1_Rx_interrupt() {
    int portnum = 1;
    // Loop just in case more than one character is in UART's receive FIFO buffer. Stop if buffer full
    while ((serial_COM1.readable()) && (((serial_in_pointer[portnum] + 1) % buffer_size) != serial_out_pointer[portnum])) {
        serial_rx_buffer[portnum][serial_in_pointer[portnum]] = serial_COM1.getc();
        serial_in_pointer[portnum] = (serial_in_pointer[portnum] + 1) % buffer_size;
    }
}

// Interupt routine to read in data from serial port two when it arrives
void serial_COM2_Rx_interrupt() {
    int portnum = 2;
    // Loop just in case more than one character is in UART's receive FIFO buffer. Stop if buffer full
    while ((serial_COM2.readable()) && (((serial_in_pointer[portnum] + 1) % buffer_size) != serial_out_pointer[portnum])) {
        serial_rx_buffer[portnum][serial_in_pointer[portnum]] = serial_COM2.getc();
        serial_in_pointer[portnum] = (serial_in_pointer[portnum] + 1) % buffer_size;
    }
}

// Interupt routine to read in data from serial port three when it arrives
void serial_COM3_Rx_interrupt() {
    int portnum = 3;
    // Loop just in case more than one character is in UART's receive FIFO buffer. Stop if buffer full
    while ((serial_COM3.readable()) && (((serial_in_pointer[portnum] + 1) % buffer_size) != serial_out_pointer[portnum])) {
        serial_rx_buffer[portnum][serial_in_pointer[portnum]] = serial_COM3.getc();
        serial_in_pointer[portnum] = (serial_in_pointer[portnum] + 1) % buffer_size;
    }
}

// Callback from the ethernet serial ports when data has been received via ethernet
void EthernetSerialPortDataReceived(int portnum, char *data, int length) {
    //printf("Ethernet Serial Data Received, port %d, '%s', length: %d\n", portnum, data, length);
    
    data[length] = 0;
    for (int i=0; i<length; i++) {
        //SerialPort(portnum).putc(data[i]);
        switch (portnum) {
            case 1: serial_COM1.putc(data[i]); break;
            case 2: serial_COM2.putc(data[i]); break;
            default: serial_COM3.putc(data[i]); break;
        }
    }
}

// Process loop function that checks the serial ports and performs serial to TCP bridging
void ProcessLoop_CheckSerialPorts() {
    // Buffer to store data in
    char data[1024];
    int intIndex;
        
    // Loop through each com port
    for (int portnum=1; portnum<=3; portnum++) {    
        // Reset buffer index
        intIndex = 0;
       
        // Only read if there is data in the buffer
        if(serial_in_pointer[portnum] != serial_out_pointer[portnum])
        {
            // Start Critical Section - don't interrupt while changing global buffer variables
            switch (portnum)
            {
                case 1: NVIC_DisableIRQ(UART1_IRQn); break;
                case 2: NVIC_DisableIRQ(UART2_IRQn); break;
                case 3: NVIC_DisableIRQ(UART3_IRQn); break;                
            }
            
            // Read from the buffer until we reach the end of the data indicated when the out pointer is equal to the in pointer
            while (serial_out_pointer[portnum] != serial_in_pointer[portnum])
            {
                data[intIndex] = serial_rx_buffer[portnum][serial_out_pointer[portnum]];
                serial_out_pointer[portnum] = (serial_out_pointer[portnum] + 1) % buffer_size;
                intIndex++;
                
                // Break out of the loop once the buffer is full!
                if (intIndex >= 1024)
                {
                    break;
                }
            }
            
            // End Critical Section
            switch (portnum)
            {
                case 1: NVIC_EnableIRQ(UART1_IRQn); break;
                case 2: NVIC_EnableIRQ(UART2_IRQn); break;
                case 3: NVIC_EnableIRQ(UART3_IRQn); break;                
            }
            
            // Terminate the received data
            data[intIndex] = 0;
            
            //printf("Serial data received on port %d: '%s'\n", portnum, data);
        }
        
        // If we have some data
        if (intIndex > 0) {
            printf("Sending Serial Data to TCP (%d): %s\r\n", intIndex, data);
            m_objNetworkInterface->SendSerialData(portnum, data, intIndex);
        }
    }
}

// ===========================================================================================================================================================================================

void SetSerialPortSettings() {
    char key[32];
    char value[BUFSIZ];
    char value2[BUFSIZ];
    char value3[BUFSIZ];
    
    for (int i=1;i<=3;i++) {
        sprintf(key, "Serial%dBaud", i); 
        if (m_objConfigFile.getValue(key, &value[0], sizeof(value))) {
            printf("    Serial Port %d Baud Rate: %d\n", i, atoi(value));
            if (i == 1) { serial_COM1.baud(atoi(value)); }
            if (i == 2) { serial_COM2.baud(atoi(value)); }
            if (i == 3) { serial_COM3.baud(atoi(value)); }
        }
        
        sprintf(key, "Serial%dDataBits", i); if (!m_objConfigFile.getValue(key, &value[0], sizeof(value))) { continue; }
        sprintf(key, "Serial%dParity", i); if (!m_objConfigFile.getValue(key, &value2[0], sizeof(value2))) { continue; }
        sprintf(key, "Serial%dStopBits", i); if (!m_objConfigFile.getValue(key, &value3[0], sizeof(value3))) { continue; }
        
        printf("    Serial Port %d DataBits: %d\n", i, atoi(value));
        printf("    Serial Port %d Parity: %s\n", i, value2);
        printf("    Serial Port %d StopBits: %d\n", i, atoi(value3));
        
        Serial::Parity     parity;
        if (value2 == "Even") {
            parity = Serial::Even;
        } else if (value2 == "Odd") {
            parity = Serial::Odd;
        } else {
            parity = Serial::None;
        }
        
        // bits    The number of bits in a word (5-8; default = 8)
        // parity    The parity used (Serial::None, Serial::Odd, Serial::Even, Serial::Forced1, Serial::Forced0; default = Serial::None)
        // stop    The number of stop bits (1 or 2; default = 1)
        if (i == 1) { serial_COM1.format(atoi(value),parity,atoi(value3)); }
        if (i == 2) { serial_COM2.format(atoi(value),parity,atoi(value3)); }
        if (i == 3) { serial_COM3.format(atoi(value),parity,atoi(value3)); }
    }
}

// Function which reads the device config file from flash memory and sets relevant variables
void ReadConfigFile() {
    printf("==================================================\n");
    printf("Reading Device Configuration Settings From File\n");
    
    char value[BUFSIZ];
    
    // Read a configuration file from a mbed.
    if (!m_objConfigFile.read("/local/config.cfg")) {
        error("     Failure to read the configuration file.\n");
    }
    // Get a configuration value.
    if (m_objConfigFile.getValue("IP1", &value[0], sizeof(value))) { m_objNetworkInterface->m_arrIPAddress[0] = atoi(value); }
    if (m_objConfigFile.getValue("IP2", &value[0], sizeof(value))) { m_objNetworkInterface->m_arrIPAddress[1] = atoi(value); }
    if (m_objConfigFile.getValue("IP3", &value[0], sizeof(value))) { m_objNetworkInterface->m_arrIPAddress[2] = atoi(value); }
    if (m_objConfigFile.getValue("IP4", &value[0], sizeof(value))) { m_objNetworkInterface->m_arrIPAddress[3] = atoi(value); }
    
    printf("    Device IP Address: %d.%d.%d.%d\n", m_objNetworkInterface->m_arrIPAddress[0], m_objNetworkInterface->m_arrIPAddress[1], m_objNetworkInterface->m_arrIPAddress[2], m_objNetworkInterface->m_arrIPAddress[3]);
    printf("    Telnet Debug Mode: %d\n", TELNET_DEBUG);
    
    // Set the serial port settings
    SetSerialPortSettings();
}

// Function that sets up the I/O expander devices ready for operation
void SetupIO() {
    printf("==================================================\n");
    printf("Setting up I/O Devices\n");

    out_InputCS1 = 1;
    out_InputCS2 = 1;
    out_InputLatch = 0;
    
    m_bytOutputs[0] = 0;
    out_OutputLatch = 0;
}
