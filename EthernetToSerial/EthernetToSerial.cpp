#include "EthernetToSerial.h"

// Attach serial interupt routine
void EthernetToSerial::AttachInterrupts(void (* serialRxHandler)()) 
{
    _serialPort->attach(serialRxHandler, Serial::RxIrq);
}

// Interupt routine to read in data from serial port when it arrives
void EthernetToSerial::RxInterrupt() 
{
    // Loop just in case more than one character is in UART's receive FIFO buffer. Stop if buffer full.
    while ((_serialPort->readable()) && (((_inPointer + 1) % BUFFER_SIZE) != _outPointer)) {
        _rxBuffer[_inPointer] = _serialPort->getc();
        _inPointer = (_inPointer + 1) % BUFFER_SIZE;
    }
}

// Check the serial port buffer
void EthernetToSerial::CheckPortReceiveBuffer()
{
    // Buffer to store data in
    char data[BUFFER_SIZE];
    int index = 0;
        
    // Only read if there is data in the buffer
    if(_inPointer != _outPointer)
    {
        // Start Critical Section - don't interrupt while changing global buffer variables
        //NVIC_DisableIRQ(UART1_IRQn);
        
        // Read from the buffer until we reach the end of the data indicated when the out pointer is equal to the in pointer
        while (_outPointer != _inPointer)
        {
            data[index++] = _rxBuffer[_outPointer];
            _outPointer = (_outPointer + 1) % BUFFER_SIZE;
        }
        
        // End Critical Section
        //NVIC_EnableIRQ(UART1_IRQn);
        
        // Terminate the received data
        data[index] = 0;
        
        //printf("Serial data received on port %d: '%s'\n", portnum, data);
        //m_objNetworkInterface->SendSerialData(portnum, data, intIndex);
    }
}