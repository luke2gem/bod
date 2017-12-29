#include "clsNetworkInterface.h"

void clsNetworkInterface::SetupTCP(int intUseDHCP) {
    printf("Setting up Ethernet\n");
    printf("==================================================\n");
    
    // Create and initialise variables
    struct netif   *netif = &netif_data;
    struct ip_addr  ipIPAddress;
    struct ip_addr  ipNetmask;
    struct ip_addr  ipGateway;

    Ticker tickFast, tickSlow, tickARP, eth_tick, dns_tick, dhcp_coarse, dhcp_fine;
    char *strHostname = "pchilton mbed 001";

    // Ensure the client object starts off
    m_objClientConnection = NULL;

    // Setup network IP's to use
    IP4_ADDR(&ipIPAddress, m_arrIPAddress[0],m_arrIPAddress[1],m_arrIPAddress[2],m_arrIPAddress[3]);
    IP4_ADDR(&ipNetmask, 255,255,255,0);
    IP4_ADDR(&ipGateway, m_arrIPAddress[0],m_arrIPAddress[1],m_arrIPAddress[2],1);

    // Initialise after configuration 
    lwip_init();

    // Set MAC address
    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    device_address((char *)netif->hwaddr);

    // Configure interface with the IP addresses defined
    netif = netif_add(netif, &ipIPAddress, &ipNetmask, &ipGateway, NULL, device_init, ip_input);
    netif->hostname = strHostname;
    netif_set_default(netif);
    
    // Bring the interface up (either using DHCP or directly)
    if (intUseDHCP > 0) {
        dhcp_start(netif);
    } else {
        netif_set_up(netif);
    }
    
    printf("Network interface 'Up' state: %d\n", netif_is_up(netif));
    printf("Network flags: %d\n", netif->flags);
    
    if (netif->flags & NETIF_FLAG_LINK_UP) {
        printf("Network link 'Up'\n");
    } else {
        printf("Network link 'Down'\n");
    }
    
    // Initialise timers for TCP/IP
    tickARP.attach_us( &etharp_tmr,  ARP_TMR_INTERVAL  * 1000);
    tickFast.attach_us(&tcp_fasttmr, TCP_FAST_INTERVAL * 1000);
    tickSlow.attach_us(&tcp_slowtmr, TCP_SLOW_INTERVAL * 1000);
    dns_tick.attach_us(&dns_tmr, DNS_TMR_INTERVAL * 1000);
    dhcp_coarse.attach_us(&dhcp_coarse_tmr, DHCP_COARSE_TIMER_MSECS * 1000);
    dhcp_fine.attach_us(&dhcp_fine_tmr, DHCP_FINE_TIMER_MSECS * 1000);

    // Clear the temp telnet buffer
    strcpy(m_strCommsBuffer,"");
    m_strCommsBuffer[0] = 0;
    strcpy(m_strCommsInputTemp,"");
    m_strCommsInputTemp[0] = 0;
    // Reset input comms index
    m_intCommsBufferIndex = 0;

    // Bind the telnet TCP port to the network interface
    struct tcp_pcb *pcb = tcp_new();
    if (tcp_bind(pcb, IP_ADDR_ANY, 23) == ERR_OK) {
        // Start listening on the port
        pcb = tcp_listen(pcb);
        
        // Setup callback function to call when a new connection comes in on the TCP port
        tcp_accept(pcb, &accept_callback);
    } else {
        printf("Failed to bind TCP port to network interface\n");
    }
    
    // Bind the telnet TCP port to the network interface
    struct tcp_pcb *pcb1 = tcp_new();
    if (tcp_bind(pcb1, IP_ADDR_ANY, 10001) == ERR_OK) {
        // Start listening on the port
        pcb1 = tcp_listen(pcb1);
        
        // Setup callback function to call when a new connection comes in on the TCP port
        tcp_accept(pcb1, &accept_callbackSerialPort1);
    } else {
        printf("Failed to bind TCP Serial port 1 to network interface\n");
    }
    
    // Bind the telnet TCP port to the network interface
    struct tcp_pcb *pcb2 = tcp_new();
    if (tcp_bind(pcb2, IP_ADDR_ANY, 10002) == ERR_OK) {
        // Start listening on the port
        pcb2 = tcp_listen(pcb2);
        
        // Setup callback function to call when a new connection comes in on the TCP port
        tcp_accept(pcb2, &accept_callbackSerialPort2);
    } else {
        printf("Failed to bind TCP Serial port 2 to network interface\n");
    }
    
    // Bind the telnet TCP port to the network interface
    struct tcp_pcb *pcb3 = tcp_new();
    if (tcp_bind(pcb3, IP_ADDR_ANY, 10003) == ERR_OK) {
        // Start listening on the port
        pcb3 = tcp_listen(pcb3);
        
        // Setup callback function to call when a new connection comes in on the TCP port
        tcp_accept(pcb3, &accept_callbackSerialPort3);
    } else {
        printf("Failed to bind TCP Serial port 3 to network interface\n");
    }
    
}

long clsNetworkInterface::lngDecodeBase128ValueInReply(int intStartChar) {
    char strTemp[6];
    int intBase128[5];
    long lngValue = 0;
    
    strncpy(strTemp, (char*)&m_strCommsBuffer[intStartChar], 5); strTemp[5] = 0;
    if (TELNET_DEBUG) { printf("Parameter 1: %s (%d)\n", strTemp, (int)(strTemp)); }
    for (int i=0; i<5; i++) {
        intBase128[i] = (int)strTemp[i] - 32; // Encoded byte values are offset by 32 so they are away from control characters
        lngValue *= 128; // Shift value along by 7 bits (multiply by 128 does this)
        lngValue += intBase128[i]; // Append the byte value
    }

    // If number received is a negative one
    if ((intBase128[0] && 0x08) > 0) {
        lngValue = -(4294967296 - lngValue);
    }

    if (TELNET_DEBUG) { printf("Decoded Value: %ld\n", lngValue); }
    return lngValue;
}

void clsNetworkInterface::SendReplyValue(long lngValue) {
    int n;
    char intChecksum;
    char strPacket[9];

    // Clear the comms buffer
    strcpy(strPacket,"");
    n = 0;

    // Build up a reply packet
    strPacket[n++] = 2;  // STX

    // Fixed 5 character reply in base 128 format, values offset by 32 (so they are away from control characters)
    strPacket[n++] = (int)(((lngValue>>28) & 0x7F) + 32);
    strPacket[n++] = (int)(((lngValue>>21) & 0x7F) + 32);
    strPacket[n++] = (int)(((lngValue>>14) & 0x7F) + 32);
    strPacket[n++] = (int)(((lngValue>>7) & 0x7F) + 32);
    strPacket[n++] = (int)(((lngValue) & 0x7F) + 32);

    // Calculate the checksum of the packet data - from AFTER STX
    intChecksum = 0;
    for (int i=1; i<n; i++) {
        intChecksum ^= (int)(strPacket[i]);
        if (TELNET_DEBUG) { printf("Char: %d, Checksum: %d\n", (int)(strPacket[i]), intChecksum); }
    }

    // Checksum always has bit 7 set so that it isnt in the readable char range.
    intChecksum |= 0x80;
    if (TELNET_DEBUG) { printf("Calculated checksum for packet to transmit: %d\n", intChecksum); }

    // Add checksum and ETX to the packet, zero terminate
    strPacket[n++] = intChecksum;
    strPacket[n++] = 3;
    strPacket[n] = 0;
 
    if (TELNET_DEBUG) { printf("Sending reply to PC: %d ('%s')\n", n, strPacket); }

    if (m_objClientConnection != NULL) {
        if (tcp_write(m_objClientConnection, strPacket, strlen(strPacket), 1) != ERR_OK) {
            //error("Failed to write data\n");
        }
        
        tcp_output(m_objClientConnection);
    }
}

void clsNetworkInterface::SendReply(char *strData, int intDataLength) {
    strData[intDataLength] = 0;

    if (TELNET_DEBUG) { printf("Sending reply to PC: %d ('%s')\n", intDataLength, strData); }

    if (m_objClientConnection != NULL) {
        if (tcp_write(m_objClientConnection, strData, strlen(strData), 1) != ERR_OK) {
            //error("Failed to write data\n");
        }
        
        tcp_output(m_objClientConnection);
    }
}

// Find the position of a specified character in the data buffer
int clsNetworkInterface::FindCharPosition(char *data, int length, int searchValue, int startPosition) {
    for (int i=startPosition; i<length; i++) {
        if (data[i] == searchValue) {
            return i;
        }
    }
    
    return -1;
}

// Validate that a propeller response is valid
int clsNetworkInterface::intValidatePacket(char *strData) {
   int              stxPosition, previousStxPosition, etxPosition, previousEtxPosition;

   int              intLength, intChecksum, intChecksum2, intIndex, intDestAdd;
   char             packet[TELNETBUFFERSIZE];

   if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("Parsing packet: '%s', length: %d\n", strData, strlen(strData)); }
   if (NETWORK_DEBUG_VALIDATE_PACKET) {
        for (int i=0; i<strlen(strData); i++) {
            printf("%d ", strData[i]);
        }
        printf("\n");
   }

   // Set default value for STX characters
   stxPosition = -1;
   etxPosition = -1;

   // While we don't have a valid STX/ETX structure
   while (etxPosition <= stxPosition || stxPosition < 0 || etxPosition < 0) {
      if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("Looking for STX/ETX... \n"); }

      // Store last ETX
      previousEtxPosition = etxPosition;

      // Get the position of the next STX/ETX characters in the buffer
      stxPosition = FindCharPosition(strData, strlen(strData), 2, stxPosition + 1);
      etxPosition = FindCharPosition(strData, strlen(strData), 3, etxPosition + 1);
      if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("STX: %d ETX: %d\n", stxPosition, etxPosition); }

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
         if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("NEXT STX: %d ETX: %d\n", stxPosition, etxPosition); }
      
         // Check STX against ETX
         if (stxPosition >= etxPosition) {
            // Return STX to previous value as we have passed the end of the packet
            stxPosition = previousStxPosition;
            if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("STX is after ETX, restoring STX to previous value: %d\n", stxPosition); }
            break;
         }

         // Stop STXs oscillating back and forth
         if (stxPosition < previousStxPosition) {
            stxPosition = previousStxPosition;
            if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("STX Pos was less than the last one found, restoring last: %d\n", stxPosition); }
            break;
         }

         // Check for STX the same as the previous to stop never-ending loops
         if (stxPosition == previousStxPosition) {
            if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("STX Pos was the same as the last one: %d\n", stxPosition); }
            break;
         }
      }

      // Check for ETX the same as the previous to stop never-ending loops
      if (etxPosition == previousEtxPosition) {
         if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("ETX Pos was the same as the last one: %d\n", etxPosition); }
         break;
      }
   }

   // If a valid packet has been parsed
   if (stxPosition >= 0 && etxPosition >= 0 && etxPosition > stxPosition && etxPosition < strlen(strData)) {
      // Calculate the length of the packet (inc STX/ETX)
      intLength = etxPosition - stxPosition + 1;
      if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("\n\nDONE: STX: %d, ETX: %d, Packet Length: %d\n\n", stxPosition, etxPosition, intLength); }

      // Parse out the packet to send on the BUS
      strncpy(packet, (char*)&strData[stxPosition], intLength);
      packet[intLength] = 0;
      if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("DATA = '%s'\n", packet); }
   } else {
      if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("\n\nSTX/ETX INVALID: %d %d strlen: %d\n\n", stxPosition, etxPosition, strlen(strData)); }
      printf("\n\nSTX/ETX INVALID: %d %d strlen: %d\n\n", stxPosition, etxPosition, strlen(strData));
      return 0;
   }

   if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("Packet Length: %d\n", intLength); }

   // Calculate the checksum of the received data
   intChecksum = 0;
   for (intIndex=1;intIndex<intLength-2;intIndex++) {
      intChecksum ^= (int)(packet[intIndex]);
      if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("Char: %d, Checksum: %d\n", (int)(packet[intIndex]), intChecksum); }
   }
   // Checksum always has bit 7 set so that it isnt in the readable char range.
   intChecksum |= 0x80;
   if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("Calculated checksum for packet: %d\n", intChecksum); }

   // Parse out the destination address
   intDestAdd = (int)((char*)&packet[1])[0] - 10;
   if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("Address: %d\n", intDestAdd); }
   
   // Parse the checksum char
   intChecksum2 = (int)((char*)&packet[intLength - 2])[0];
   if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("Checksum: %d\n", intChecksum2); }

   // If the checksum character is correct
   if (intChecksum != intChecksum2) {
      if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("THE CHECKSUMS DO NOT MATCH!!!!!!!!!\n"); }
      printf("THE CHECKSUMS DO NOT MATCH!!!!!!!!!\n");
      // Return so more data can be received
      return 0;
   }

   // Copy parsed packet into the comms buffer
   strcpy(m_strCommsBuffer, packet);

   m_intLastPacketRXLength = intLength;
   if (NETWORK_DEBUG_VALIDATE_PACKET) { printf("Checksums OK\n"); }
   return intDestAdd;
}

// Parse the received data on the telnet socket
int clsNetworkInterface::intParseTelnetData() {
    int     intValidResult;
    
    // If there is space in the comms buffer
    if ((strlen(m_strCommsBuffer) + strlen(m_strCommsInputTemp)) < sizeof(m_strCommsBuffer)) {
        // Append the data to the comms buffer
        strcat(m_strCommsBuffer, m_strCommsInputTemp);
        // Debug received data
        if (TELNET_DEBUG) { printf("\n\n\nRX: Data not shown \n", m_strCommsInputTemp, m_strCommsBuffer); }
        //if (TELNET_DEBUG) { printf("\n\n\nRX: '%s' ('%s')\n", m_strCommsInputTemp, m_strCommsBuffer); }
        // Clear the temp telnet buffer
        strcpy(m_strCommsInputTemp,"");

        if (TELNET_DEBUG) { printf("Temp buffer cleared, parsing packet...\n"); }

        // Validate the packet and leave the valid packet in m_strCommsBuffer
        intValidResult = intValidatePacket(m_strCommsBuffer);
        if (TELNET_DEBUG) { printf("Validation Result: %d\n", intValidResult); }
        //if (TELNET_DEBUG) { printf("Buffer after parsing: '%s'\n", m_strCommsBuffer); }
        if (TELNET_DEBUG) { printf("Buffer after parsing: 'Data not shown'\n", m_strCommsBuffer); }

        if (intValidResult > 0) {
            // Valid packet, process command
            PacketReceived(m_strCommsBuffer, intValidResult, m_intLastPacketRXLength);
           
            // Clear the temp telnet buffer
            strcpy(m_strCommsBuffer,"");
            m_strCommsBuffer[0] = 0;
            strcpy(m_strCommsInputTemp,"");
            m_strCommsInputTemp[0] = 0;
            // Reset input comms index
            m_intCommsBufferIndex = 0;
        }

        // Result TRUE
        return 1;
    } else {
        if (TELNET_DEBUG) { printf("Buffer full, clearing comms input buffer\n"); }
        // Clear the comms buffer
        strcpy(m_strCommsBuffer,"");
    }

    return 0;
}


// This method is called each time data is received on the TCP connection
err_t recv_callback(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    int i;
    char *data;

    // Check if status is ok and data is arrived.
    if (err == ERR_OK && p != NULL) {
        if (TELNET_DEBUG) { printf("TCP RX:"); }
        
        // Inform TCP that we have taken the data
        tcp_recved(pcb, p->tot_len);
        
        // Get packet data
        data = static_cast<char *>(p->payload);
        
        if (TELNET_DEBUG) { printf("%s (%d)\n", data, p->tot_len); }
        
        // Get the length of data received
        i = p->tot_len;
        if (i > TELNETBUFFERSIZE) { i = TELNETBUFFERSIZE; }
        
        if (i<=0) {
            printf("Callback called with no data (%d)\n",i);
        }
        
        // Copy the received data into our temp buffer for parsing
        strncpy(m_objNetworkInterface->m_strCommsInputTemp, (char*)&data[0], i);
        
        // Clean up memory used for this data packet
        pbuf_free(p);
                    
        // Terminate received string
        m_objNetworkInterface->m_strCommsInputTemp[i] = 0;
        
        // Check whether the data received contains a valid packet
        m_objNetworkInterface->intParseTelnetData();

        /*
        // No data arrived 
            // That means the client closes the connection and sent us a packet with FIN flag set to 1. 
            // We have to cleanup and destroy out TCPConnection. 
            pbuf_free(p);
        */
    }

    return ERR_OK;
}


void clsNetworkInterface::SendSerialData(int port, char *data, int length) {
    data[length] = 0;
    
    if (TELNET_DEBUG) { printf("Sending reply to Ethernet Serial Port %d: '%s' (Length: %d)\n", port, data, length); }
    //printf("Sending reply to Ethernet Serial Port %d: '%s' (Length: %d)\n", port, data, length);
    
    if (port == 1) {
        if (_ethernetSerialPort1 != NULL) {
            if (tcp_sndbuf(_ethernetSerialPort1) > 0)
            {
                if (tcp_write(_ethernetSerialPort1, data, length, 1) != ERR_OK) {
                    //error("Failed to write data\n");
                }
                
                tcp_output(_ethernetSerialPort1);
            }
        }
    } else if (port == 2) {
        if (_ethernetSerialPort2 != NULL) {
            if (tcp_sndbuf(_ethernetSerialPort2) > 0)
            {
                if (tcp_write(_ethernetSerialPort2, data, length, 1) != ERR_OK) {
                    //error("Failed to write data\n");
                }
                
                tcp_output(_ethernetSerialPort2);
            }
        }
    } else if (port == 3) {
        if (_ethernetSerialPort3 != NULL) {
            if (tcp_sndbuf(_ethernetSerialPort3) > 0)
            {
                if (tcp_write(_ethernetSerialPort3, data, length, 1) != ERR_OK) {
                    //error("Failed to write data\n");
                }
                
                tcp_output(_ethernetSerialPort3);
            }
        }
    }
    
}

// This method is called each time data is received on the TCP connection
err_t recv_callbackSerialPort1(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    int i;
    char *data;

    // Check if status is ok and data is arrived.
    if (err == ERR_OK && p != NULL) {
        if (TELNET_DEBUG) { printf("Ethernet Serial 1 RX:"); }
        
        // Inform TCP that we have taken the data
        tcp_recved(pcb, p->tot_len);
        
        // Get packet data
        data = static_cast<char *>(p->payload);
 
        // Get the length of data received
        i = p->tot_len;
        if (i > TELNETBUFFERSIZE) { i = TELNETBUFFERSIZE; printf("Serial Port 1 Transmit Buffer Overflow\n"); }
        
        if (i<=0) {
            printf("Callback called with no data (%d)\n",i);
        }
        
        // Clean up memory used for this data packet
        pbuf_free(p);
                    
        // Termintate the string by setting a 0 at the end of the useful data
        data[i] = 0;            
 
        if (TELNET_DEBUG) { printf("%s (%d)\n", data, p->tot_len); }
                
        // Call data received method with the data
        m_objNetworkInterface->EthernetSerialPortDataReceived(1, data, i);
    }

    return ERR_OK;
}

// This method is called each time data is received on the TCP connection
err_t recv_callbackSerialPort2(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    int i;
    char *data;

    // Check if status is ok and data is arrived.
    if (err == ERR_OK && p != NULL) {
        if (TELNET_DEBUG) { printf("Ethernet Serial 2 RX:"); }
        
        // Inform TCP that we have taken the data
        tcp_recved(pcb, p->tot_len);
        
        // Get packet data
        data = static_cast<char *>(p->payload);
                
        // Get the length of data received
        i = p->tot_len;
        if (i > TELNETBUFFERSIZE) { i = TELNETBUFFERSIZE; printf("Serial Port 2 Transmit Buffer Overflow\n"); }
        
        if (i<=0) {
            printf("Callback called with no data (%d)\n",i);
        }
        
        // Clean up memory used for this data packet
        pbuf_free(p);

        // Termintate the string by setting a 0 at the end of the useful data
        data[i] = 0;            
 
        if (TELNET_DEBUG) { printf("%s (%d)\n", data, p->tot_len); }
        
        // Call data received method with the data
        m_objNetworkInterface->EthernetSerialPortDataReceived(2, data, i);
    }

    return ERR_OK;
}

// This method is called each time data is received on the TCP connection
err_t recv_callbackSerialPort3(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    int i;
    char *data;

    // Check if status is ok and data is arrived.
    if (err == ERR_OK && p != NULL) {
        if (TELNET_DEBUG) { printf("Ethernet Serial 3 RX:"); }
        
        // Inform TCP that we have taken the data
        tcp_recved(pcb, p->tot_len);
        
        // Get packet data
        data = static_cast<char *>(p->payload);
        
        // Get the length of data received
        i = p->tot_len;
        if (i > TELNETBUFFERSIZE) { i = TELNETBUFFERSIZE; printf("Serial Port 3 Transmit Buffer Overflow\n"); }
        
        if (i<=0) {
            printf("Callback called with no data (%d)\n",i);
        }
        
        // Clean up memory used for this data packet
        pbuf_free(p);
        
        // Termintate the string by setting a 0 at the end of the useful data
        data[i] = 0;            
 
        if (TELNET_DEBUG) { printf("%s (%d)\n", data, p->tot_len); }
        
        // Call data received method with the data
        m_objNetworkInterface->EthernetSerialPortDataReceived(3, data, i);
    }

    return ERR_OK;
}

// Accept an incoming call on the registered port 
err_t accept_callback(void *arg, struct tcp_pcb *objClientConnection, err_t err) {
    printf("Accepting new client connection\n");
    LWIP_UNUSED_ARG(arg);
    
    // Assign the callback function to call when data is received from this client connection
    tcp_recv(objClientConnection, &recv_callback);
    
    // Store the client connection object to use
    m_objNetworkInterface->m_objClientConnection = objClientConnection;
    
    return ERR_OK;
}

err_t dataSent_SerialPort1(void * arg, struct tcp_pcb * tpcb, u16_t len) {
    //printf("%d characters sent to Ethernet Serial Port 1\n", len);
    return NULL;
}

err_t dataSent_SerialPort2(void * arg, struct tcp_pcb * tpcb, u16_t len) {
    //printf("%d characters sent to Ethernet Serial Port 1\n", len);
    return NULL;
}

err_t dataSent_SerialPort3(void * arg, struct tcp_pcb * tpcb, u16_t len) {
    //printf("%d characters sent to Ethernet Serial Port 1\n", len);
    return NULL;
}

// Accept an incoming call on the registered port 
err_t accept_callbackSerialPort1(void *arg, struct tcp_pcb *clientConnection, err_t err) {
    printf("Accepting new serial port 1 connection\n");
    LWIP_UNUSED_ARG(arg);
    
    // Assign the callback function to call when data is received from this client connection
    tcp_recv(clientConnection, &recv_callbackSerialPort1);
    
    // Store the client connection object to use
    m_objNetworkInterface->_ethernetSerialPort1 = clientConnection;
    
    // Register a callback function when data has been sent through the connection
    tcp_sent(clientConnection, &dataSent_SerialPort1);
    
    return ERR_OK;
}

// Accept an incoming call on the registered port 
err_t accept_callbackSerialPort2(void *arg, struct tcp_pcb *clientConnection, err_t err) {
    printf("Accepting new serial port 2 connection\n");
    LWIP_UNUSED_ARG(arg);
    
    // Assign the callback function to call when data is received from this client connection
    tcp_recv(clientConnection, &recv_callbackSerialPort2);
    
    // Store the client connection object to use
    m_objNetworkInterface->_ethernetSerialPort2 = clientConnection;
    
    // Register a callback function when data has been sent through the connection
    tcp_sent(clientConnection, &dataSent_SerialPort2);
    
    return ERR_OK;
}

// Accept an incoming call on the registered port 
err_t accept_callbackSerialPort3(void *arg, struct tcp_pcb *clientConnection, err_t err) {
    printf("Accepting new serial port 3 connection\n");
    LWIP_UNUSED_ARG(arg);
    
    // Assign the callback function to call when data is received from this client connection
    tcp_recv(clientConnection, &recv_callbackSerialPort3);
    
    // Store the client connection object to use
    m_objNetworkInterface->_ethernetSerialPort3 = clientConnection;
    
    // Register a callback function when data has been sent through the connection
    tcp_sent(clientConnection, &dataSent_SerialPort3);
    
    return ERR_OK;
}
