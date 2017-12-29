#include "clsPropellerInterface.h"

// Setup the propeller interface, ensuring pin designations are set and that the port is ready to receive data
void clsPropellerInterface::SetupPropellerInterface(BusInOut *bus_PropellerDataBUS, DigitalIn *in_PropellerControlRX, DigitalOut *out_PropellerControlTX, PwmOut *statusLed) {
    // Assign pins
    m_bus_PropellerDataBUS = bus_PropellerDataBUS;
    m_in_PropellerControlRX = in_PropellerControlRX;
    m_out_PropellerControlTX = out_PropellerControlTX;
    _statusLed = statusLed;
    
    // Set bus as input pins ready for receiving our first data nibble
    m_bus_PropellerDataBUS->input();
    // Set TX line high as ready to receive
    m_out_PropellerControlTX->write(1);
}

// Send a command to the propeller and obtain a response
long clsPropellerInterface::lngSendCommand(int intCommand, int intAxis, long lngParameterValue) {
    int n;
    char intChecksum;
    char strPacket[12]; // Maximum packet size with a value

    // Command is offset by the axis number
    char bytCommand = intCommand + (intAxis - 1);

    // Clear the comms buffer
    strcpy(strPacket,"");
    n = 0;

    // Build up a packet to send to propeller
    strPacket[n++] = 2;  // STX
    strPacket[n++] = 1;  // Node address
    strPacket[n++] = bytCommand; // Command
    
    // If a value parameter has been specified
    if (lngParameterValue != NULL) {
        // Fixed 5 character encoded value in base 128 format, values offset by 32 (so they are away from control characters)
        strPacket[n++] = (int)(((lngParameterValue>>28) & 0x7F) + 32);
        strPacket[n++] = (int)(((lngParameterValue>>21) & 0x7F) + 32);
        strPacket[n++] = (int)(((lngParameterValue>>14) & 0x7F) + 32);
        strPacket[n++] = (int)(((lngParameterValue>>7) & 0x7F) + 32);
        strPacket[n++] = (int)(((lngParameterValue) & 0x7F) + 32);
    }

    // Calculate the checksum of the packet data - from AFTER STX
    intChecksum = 0;
    for (int i=1; i<n; i++) {
        intChecksum ^= (int)(strPacket[i]);
        if (PROPELLER_DEBUG) { printf("Char: %d, Checksum: %d\n", (int)(strPacket[i]), intChecksum); }
    }
    // Checksum always has bit 7 set so that it isnt in the readable char range.
    intChecksum |= 0x80;
    if (PROPELLER_DEBUG) { printf("Calculated checksum for packet to transmit: %d\n", intChecksum); }

    // Add checksum and ETX to the packet, zero terminate
    strPacket[n++] = intChecksum; // Checksum
    strPacket[n++] = 3; // ETX
    strPacket[n] = 0;
 
    // Send the command to the propeller and receive the reply
    int intPacketLength = intTX(strPacket, n);
    if (intPacketLength > 5) {
    	if (PROPELLER_DEBUG) { printf("Decoding Base128 Value...\r\n"); }

        // Parse and return the reply value
        return lngDecodeBase128ValueInReply();
    } else if (intPacketLength > 0) {
    	if (PROPELLER_DEBUG) { printf("Decoding Value...\r\n"); }

    	// Value is a single number response (boolean)
        int intValue = (int)m_strReply[1];
        if (intValue == 49) { return 1; }
        if (intValue == 48) { return 0; }
        return intValue;
    }
    
    if (PROPELLER_DEBUG) { printf("No Reply...\r\n"); }

    // Failed to receive response from propeller!
    return NULL;
}

// Decode the reply value from a propeller reply
long clsPropellerInterface::lngDecodeBase128ValueInReply() {
    int intBase128[5];
    long lngValue = 0;
    
    if (PROPELLER_DEBUG) { printf("Decoding Value from %s\n", m_strReply); }

    for (int i=0; i<5; i++) {
    	if (PROPELLER_DEBUG) { printf("%c\r\n", m_strReply[i + 1]); }
    	if (PROPELLER_DEBUG) { printf("%d\r\n", (int)m_strReply[i + 1]); }

        intBase128[i] = (int)m_strReply[i + 1] - 32; // Encoded byte values are offset by 32 so they are away from control characters
        lngValue *= 128; // Shift value along by 7 bits (multiply by 128 does this)
        lngValue += intBase128[i]; // Append the byte value
    }
            
    // If number received is a negative one
    if ((intBase128[0] && 0x08) > 0) {
        lngValue = -(4294967296 - lngValue);
    }

    if (PROPELLER_DEBUG) { printf("Decoded Value: %ld\n", lngValue); }
    return lngValue;
}

// Transmits a packet to the propller and obtains a response.
// Function returns the length of the reply data received back from the propller
int clsPropellerInterface::intTX(char* strPacket, int intPacketLength) {
    // Attempt to send packets to the propeller multiple times to give us the best chance of good comms
    for (int intRetry=0; intRetry<3; intRetry++) {
        _statusLed->write(1);
        
        // Send the received command to the propeller
        TX(strPacket, intPacketLength);

        // Get a reply from the prop
        int intReplyLength = RX();
        
        _statusLed->write(0);
        
        // Debug messages
        if (PROPELLER_DEBUG_HIGHLEVEL) { printf("Propeller Reply: %s, length: %d\n", m_strReply, intReplyLength); }
        //printf("Propeller Reply: %s, length: %d\n", m_strReply, intReplyLength);
        
        if (((char*)&strPacket[2])[0] == 28) {
            // DEBUG
            //printf("******************************\n");
            //printf("%ld ", lngDecodeBase128ValueInReply());
            
            //for (int i=0;i<intReplyLength;i++) {
            //    printf("%d ", m_strReply[i]);
            //}
            
            //printf("\n");
            //printf("******************************\n");
        }
    
        // If a reply was received
        if (intReplyLength > 0) {
            // Check that the reply was valid
            if (intValidatePacket(m_strReply) > 0) {
                return intReplyLength;
            } else {
                printf("Checksum from propeller is invalid\n");
            }
        }
    }
    
    // Unable to get a response from the propller
    return 0;
}

// Transmit a packet to the propeller
void clsPropellerInterface::TX(char* strPacket, int intPacketLength) {
    // Check the slave is ready to receive data
    if (m_in_PropellerControlRX->read() == 0) {
    	if (PROPELLER_DEBUG_HIGHLEVEL) { printf("Slave is not ready to accept commands\n"); }
        // Slave is not ready
        return;
    }
    
    // Set data pins to outputs
    m_bus_PropellerDataBUS->output();

    int intByteCharacter;
    if (PROPELLER_DEBUG_HIGHLEVEL) { printf("Sending packet to propeller: "); }
    
    // Loop through each data byte
    for (int intByte=0; intByte<intPacketLength; intByte++) {
        // Get the byte that needs sending
        intByteCharacter = strPacket[intByte];
        if (PROPELLER_DEBUG_HIGHLEVEL) { printf("%d ", intByteCharacter); }
        
        // Loop through low and high nibbles in the byte
        //for (int intNibble=0; intNibble<2; intNibble++) {        
            // Set data nibble
            //if (intNibble == 0) {
            //    m_bus_PropellerDataBUS->write(intByteCharacter & 0x0F);
            //} else {
            //    m_bus_PropellerDataBUS->write((intByteCharacter & 0xF0) >> 4);
            //}
            
            // Set data
            m_bus_PropellerDataBUS->write(intByteCharacter);

            // Set TCLK low
            m_out_PropellerControlTX->write(0);

            Timer tmrTimeout;
            tmrTimeout.start();
            
            // Wait for RCLK to be pulled low (slave received data)
            while (m_in_PropellerControlRX->read() != 0) {
                // Check for a timeout waiting for a reply from the propeller
                if(tmrTimeout.read_ms() > 1000) {
                    printf("Timed out waiting for propeller to acknowledge data\n");
                    return;
                }
            }
            
            // Set TCLK high
            m_out_PropellerControlTX->write(1);

            tmrTimeout.reset();
            // Wait for RCLK to be pulled high again (slave ready for next nibble)
            while (m_in_PropellerControlRX->read() == 0) {
                // Check for a timeout waiting for a reply from the propeller
                if(tmrTimeout.read_ms() > 1000) {
                    printf("Timed out waiting for propeller to be ready after acknowledging data\n");
                    return;
                }
            }
        //}
    }

    if (PROPELLER_DEBUG_HIGHLEVEL) { printf("\n"); }

    // Set data pins to inputs again
    m_bus_PropellerDataBUS->input();
}

// Receive a response from the propller
int clsPropellerInterface::RX() {
    // Set TCLK high before we start processing (it should be anyway)
    m_out_PropellerControlTX->write(1);

    // Keep track of whether we are reading an upper or lower nibble
    //int intNibble = 0;
    
    // Keep track of byte position within the reply string
    int intByte = 0;
    
    Timer tmrTimeout;
    tmrTimeout.start();
    
    // Wait until the slave is transmitting data (RCLK LOW)
    while (m_in_PropellerControlRX->read() == 1) {
        // Check for a timeout waiting for a reply from the propeller
        if(tmrTimeout.read_ms() > 1000) {
            printf("Timed out waiting for propeller to start replying\n");
            return 0;
        }
    }

    // Data value being read
    int intData = 0;
    
    if (PROPELLER_DEBUG_HIGHLEVEL) { printf("Receiving packet from propeller: "); }
    
    // Check the slave is transmitting data (RCLK LOW)
    while (m_in_PropellerControlRX->read() == 0) {
        // Set the data character
        //if (intNibble == 0) {
        //    // Read data from the bus
        //    intData = m_bus_PropellerDataBUS->read();
        //} else {
        //    // Read data from the bus
        //    intData += (m_bus_PropellerDataBUS->read() << 4);
        //    m_strReply[intByte] = intData;
        //    if (PROPELLER_DEBUG_HIGHLEVEL) { printf("%d ", intData); }
        //}

        // Read data from the bus
        intData = m_bus_PropellerDataBUS->read();
        m_strReply[intByte] = intData;
        if (PROPELLER_DEBUG_HIGHLEVEL) { printf("%d ", intData); }
        
        
        // Zero terminate string so far
        m_strReply[intByte + 1] = 0;
        
        // Take TCLK low to ACK reception
        m_out_PropellerControlTX->write(0);
        
        tmrTimeout.reset();
        // Wait for RCLK to be high again (transmitting unit has received the ACK)
        while (m_in_PropellerControlRX->read() == 0) {
            // Check for a timeout waiting for a reply from the propeller
            if(tmrTimeout.read_ms() > 1000) {
                printf("Timed out waiting for propeller to send reply\n");
                return 0;
            }
        }
        
        // Reset TCLK to indicate we are ready for out next nibble
        m_out_PropellerControlTX->write(1);
        
        // Cycle nibble tracker variable
        //intNibble++;
        //if (intNibble > 1) { 
        //    // Increment byte count
        //    intByte++;
        //    intNibble = 0;
        //}
        
        intByte++;
        
        // If we have just received a full byte of data and it is the ETX character
        //if (intNibble == 0 && intData == 3) {
        if (intData == 3) {
            // Exit out of the receive loop
            if (PROPELLER_DEBUG_HIGHLEVEL) { printf(" - ETX Received"); }
            //printf(" - ETX Received");
            break;
        }
        
        // Wait until the slave is transmitting data (RCLK LOW)
        tmrTimeout.reset();
        while (m_in_PropellerControlRX->read() == 1) {
            // Check for a timeout waiting for a reply from the propeller
            if(tmrTimeout.read_us() > 1000) {
                printf("Timed out waiting for propeller to send reply\n");
                return 0;
            }
        }
    }
    
    //if (PROPELLER_DEBUG_HIGHLEVEL) { printf(" (Length: %d) Last Nibble: %d\n", intByte, intNibble); }
    if (PROPELLER_LOG_REPLY_FROM_PROPELLER) {
        for (int i=0; i<intByte; i++) {
            printf("%d ", m_strReply[i]);
        }
        printf("\n");
    }
    
    // Return the number of bytes received
    return intByte;
}

// Find the position of a specified character in the data buffer
int clsPropellerInterface::FindCharPosition(char *data, int length, int searchValue, int startPosition) {
    for (int i=startPosition; i<length; i++) {
        if (data[i] == searchValue) {
            return i;
        }
    }
    
    return -1;
}

// Validate that a propeller response is valid
int clsPropellerInterface::intValidatePacket(char *strData) {
   int              stxPosition, previousStxPosition, etxPosition, previousEtxPosition;

   int              intLength, intChecksum, intChecksum2, intIndex, intDestAdd;
   char             strPacket[TELNETBUFFERSIZE];

   if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("Parsing packet: '%s'\n", strData); }

   // Set default value for STX characters
   stxPosition = -1;
   etxPosition = -1;

   // While we don't have a valid STX/ETX structure
   while (etxPosition <= stxPosition || stxPosition < 0 || etxPosition < 0) {
      if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("Looking for STX/ETX... \n"); }

      // Store last ETX
      previousEtxPosition = etxPosition;

      // Get the position of the next STX/ETX characters in the buffer
      stxPosition = FindCharPosition(strData, strlen(strData), 2, stxPosition + 1);
      etxPosition = FindCharPosition(strData, strlen(strData), 3, etxPosition + 1);
      if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("STX: %d ETX: %d\n", stxPosition, etxPosition); }

      // If there is no STX or ETX character found
      if (stxPosition < 0 || etxPosition < 0) {
         // Return so more data can be received
         return 0;
      }

      // Now, find the last occurence of an STX before our ETX
      while (stxPosition < etxPosition) {
         // Store last STX
         previousStxPosition = stxPosition;

         // Get the position of the next STX character in the buffer    
         stxPosition = FindCharPosition(strData, strlen(strData), 2, stxPosition + 1);
         //etxPosition = FindCharPosition(strData, strlen(strData), 3, etxPosition + 1);
         if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("NEXT STX: %d ETX: %d\n", stxPosition, etxPosition); }
      
         // Check STX against ETX
         if (stxPosition >= etxPosition) {
            // Return STX to previous value as we have passed the end of the packet
            stxPosition = previousStxPosition;
            if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("STX is after ETX, restoring STX to previous value: %d\n", stxPosition); }
            break;
         }

         // Stop STXs oscillating back and forth
         if (stxPosition < previousStxPosition) {
            stxPosition = previousStxPosition;
            if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("STX Pos was less than the last one found, restoring last: %d\n", stxPosition); }
            break;
         }

         // Check for STX the same as the previous to stop never-ending loops
         if (stxPosition == previousStxPosition) {
            if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("STX Pos was the same as the last one: %d\n", stxPosition); }
            break;
         }
      }

      // Check for ETX the same as the previous to stop never-ending loops
      if (etxPosition == previousEtxPosition) {
         if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("ETX Pos was the same as the last one: %d\n", etxPosition); }
         break;
      }
   }

   // If a valid packet has been parsed
   if (stxPosition >= 0 && etxPosition >= 0 && etxPosition > stxPosition && etxPosition < strlen(strData)) {
      // Calculate the length of the packet (inc STX/ETX)
      intLength = etxPosition - stxPosition + 1;
      if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("\n\nDONE: STX: %d, ETX: %d, Packet Length: %d\n\n", stxPosition, etxPosition, intLength); }

      // Parse out the packet to send on the BUS
      strncpy(strPacket, (char*)&strData[stxPosition], intLength);
      strPacket[intLength] = 0;
      if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("DATA = '%s'\n", strPacket); }
   } else {
      if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("\n\nSTX/ETX INVALID: %d %d strlen: %d\n\n", stxPosition, etxPosition, strlen(strData)); }
      return 0;
   }

   if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("Packet Length: %d\n", intLength); }

   // Calculate the checksum of the received data
   intChecksum = 0;
   for (intIndex=1;intIndex<intLength-2;intIndex++) {
      intChecksum ^= (int)(strPacket[intIndex]);
      if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("Char: %d, Checksum: %d\n", (int)(strPacket[intIndex]), intChecksum); }
   }
   // Checksum always has bit 7 set so that it isnt in the readable char range.
   intChecksum |= 0x80;
   if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("Calculated checksum for packet: %d\n", intChecksum); }

   // Parse out the destination address
   intDestAdd = (int)((char*)&strPacket[1])[0] - 10;
   if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("Address: %d\n", intDestAdd); }
   
   // Parse the checksum char
   intChecksum2 = (int)((char*)&strPacket[intLength - 2])[0];
   if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("Checksum: %d\n", intChecksum2); }

   // If the checksum character is correct
   if (intChecksum != intChecksum2) {
      if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("THE CHECKSUMS DO NOT MATCH!!!!!!!!!\n"); }
      // Return so more data can be received
      return 0;
   }

   // Copy parsed packet into the comms buffer
   //strcpy(m_strCommsBuffer, strPacket);

   m_intLastPacketRXLength = intLength;
   if (PROPELLER_DEBUG_VALIDATE_PACKET) { printf("Checksums OK\n"); }
   return intDestAdd;
}

