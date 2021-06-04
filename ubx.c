#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/usart.h>

#include <stdlib.h>

#include "ubx.h"
#include "i2c.h"
#include "usart.h"
#include "delay.h"

uint8_t gpsI2Caddress = 0x42; //Default 7-bit unshifted address of the ublox 6/7/8/M8/F9 series
uint8_t i2cTransactionSize = 32; //Max i2c payload 

//The packet buffers
//These are pointed at from within the ubxPacket
uint8_t payloadAck[2];				  // Holds the requested ACK/NACK
uint8_t payloadCfg[MAX_PAYLOAD_SIZE]; // Holds the requested data packet
uint8_t payloadBuf[2];				  // Temporary buffer used to screen incoming packets or dump unrequested packets

//Init the packet structures and init them with pointers to the payloadAck, payloadCfg and payloadBuf arrays
ubxPacket packetAck = {0, 0, 0, 0, 0, payloadAck, 0, 0, UBLOX_PACKET_VALIDITY_NOT_DEFINED, UBLOX_PACKET_VALIDITY_NOT_DEFINED};
ubxPacket packetCfg = {0, 0, 0, 0, 0, payloadCfg, 0, 0, UBLOX_PACKET_VALIDITY_NOT_DEFINED, UBLOX_PACKET_VALIDITY_NOT_DEFINED};
ubxPacket packetBuf = {0, 0, 0, 0, 0, payloadBuf, 0, 0, UBLOX_PACKET_VALIDITY_NOT_DEFINED, UBLOX_PACKET_VALIDITY_NOT_DEFINED};

//Flag if this packet is unrequested (and so should be ignored and not copied into packetCfg or packetAck)
uint8_t ignoreThisPayload = 0;

uint8_t rollingChecksumA; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes
uint8_t rollingChecksumB; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes

//Identify which buffer is in use
//Data is stored in packetBuf until the requested class and ID can be validated
//If a match is seen, data is diverted into packetAck or packetCfg
ublox_packet_buffer_e activePacketBuffer = UBLOX_PACKET_PACKETBUF;

SentenceTypes currentSentence = NONE;

uint16_t ubxFrameCounter;

struct
{
    uint32_t gpsiTOW : 1;
    uint32_t gpsYear : 1;
    uint32_t gpsMonth : 1;
    uint32_t gpsDay : 1;
    uint32_t gpsHour : 1;
    uint32_t gpsMinute : 1;
    uint32_t gpsSecond : 1;
    uint32_t gpsDateValid : 1;
    uint32_t gpsTimeValid : 1;
    uint32_t gpsNanosecond : 1;

    uint32_t all : 1;
    uint32_t ubx_longitude : 1;
    uint32_t ubx_latitude : 1;
    uint32_t ubx_altitude : 1;
    uint32_t altitudeMSL : 1;
    uint32_t SIV : 1;
    uint32_t fixType : 1;
    uint32_t carrierSolution : 1;
    uint32_t groundSpeed : 1;
    uint32_t headingOfMotion : 1;
    uint32_t pDOP : 1;
    uint32_t versionNumber : 1;
} moduleQueried;

//The major datums we want to globally store
uint32_t timeOfWeek;
uint16_t gpsYear;
uint8_t gpsMonth;
uint8_t gpsDay;
uint8_t gpsHour;
uint8_t gpsMinute;
uint8_t gpsSecond;
uint16_t gpsMillisecond;
int32_t gpsNanosecond;
bool gpsDateValid;
bool gpsTimeValid;

int32_t ubx_latitude;		 //Degrees * 10^-7 (more accurate than floats)
int32_t ubx_longitude;		 //Degrees * 10^-7 (more accurate than floats)
int32_t ubx_altitude;		 //Number of mm above ellipsoid
int32_t altitudeMSL;	 //Number of mm above Mean Sea Level
uint8_t SIV;			 //Number of satellites used in position solution
uint8_t fixType;		 //Tells us when we have a solution aka lock
uint8_t carrierSolution; //Tells us when we have an RTK float/fixed solution
int32_t groundSpeed;	 //mm/s
int32_t headingOfMotion; //degrees * 10^-5
uint16_t pDOP;			 //Positional dilution of precision * 10^-2 (dimensionless)
uint8_t versionLow;		 //Loaded from getProtocolVersion().
uint8_t versionHigh;

void calcChecksum(ubxPacket *msg)
{
  msg->checksumA = 0;
  msg->checksumB = 0;

  msg->checksumA += msg->cls;
  msg->checksumB += msg->checksumA;

  msg->checksumA += msg->id;
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len & 0xFF);
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len >> 8);
  msg->checksumB += msg->checksumA;

  for (uint16_t i = 0; i < msg->len; i++)
  {
    msg->checksumA += msg->payload[i];
    msg->checksumB += msg->checksumA;
  }
}

uint16_t check_avail_bytes(void)
{
	uint8_t cmdWrite2[] = { 0xFD };
	uint8_t dataOut2[2];
	//i2c_transfer7(I2C1, 0x42, cmdWrite2, sizeof(cmdWrite2), NULL, 0);
    write_i2c(0x42, cmdWrite2);
    read_i2c(0x42, dataOut2);
	uint16_t bytesAvailable = 0;
	bytesAvailable = (uint16_t)dataOut2[0] << 8 | dataOut2[1];
	return bytesAvailable;
}

void sendI2cCommand(ubxPacket *outgoingUBX)
{
    //write_i2c(gpsI2Caddress, 0xFF);
    //if (_i2cPort->endTransmission() != 0)         //Don't release bus
    //  return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); //Sensor did not ACK

    //Write header bytes
    uint8_t headerbytes[] = {
        UBX_SYNCH_1,
        UBX_SYNCH_2,
        outgoingUBX->cls,
        outgoingUBX->id,
        outgoingUBX->len & 0xFF,
        outgoingUBX->len >> 8,

    };
    write_i2c(gpsI2Caddress, headerbytes);

    
    //Write payload. Limit the sends into 32 byte chunks
    //This code based on ublox: https://forum.u-blox.com/index.php/20528/how-to-use-i2c-to-get-the-nmea-frames
    uint16_t bytesToSend = outgoingUBX->len;

    //"The number of data bytes must be at least 2 to properly distinguish
    //from the write access to set the address counter in random read accesses."
    uint16_t startSpot = 0;
    while (bytesToSend > 1)
    {
        uint8_t len = bytesToSend;
        if (len > i2cTransactionSize)
        len = i2cTransactionSize;

        uint8_t payloadBytes[len];
        int x = 0;
        for (x = 0; x < len; x++)
        {
            payloadBytes[x] = outgoingUBX->payload[startSpot + x]; //Write a portion of the payload to the bus
        }
        write_i2c(gpsI2Caddress, payloadBytes);

        //*outgoingUBX->payload += len; //Move the pointer forward
        startSpot += len; //Move the pointer forward
        bytesToSend -= len;
    }

    //Write checksum
    if (bytesToSend == 1)
    {
        uint8_t finalByte[1];
        finalByte[0] = outgoingUBX->payload[startSpot];
        write_i2c(gpsI2Caddress, finalByte);
    }
        
    uint8_t checksumBytes[2];
    checksumBytes[0] = outgoingUBX->checksumA;
    checksumBytes[1] = outgoingUBX->checksumB;
    write_i2c(gpsI2Caddress, checksumBytes);

    ublox_status_e retVal = SFE_UBLOX_STATUS_SUCCESS;

    if (outgoingUBX->cls == UBX_CLASS_CFG)
    {
      serial0_sendString("sendCommand: Waiting for ACK response");
      retVal = waitForACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, 100); //Wait for Ack response
    }
    else
    {
        serial0_sendString("sendCommand: Waiting for No ACK response");
        retVal = waitForNoACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, 100); //Wait for Ack response
    }
}

//Loads the payloadCfg array with the current protocol bits located the UBX-CFG-PRT register for a given port
void getPortSettings()
{
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_PRT;
    packetCfg.len = 1;
    packetCfg.startingSpot = 0;

    payloadCfg[0] = 0x00;

    sendI2cCommand(&packetCfg);
    //checkUbloxI2C(&packetCfg, UBX_CLASS_CFG, UBX_CFG_PRT);
    //return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_RECEIVED); // We are expecting data and an ACK
}

//Configure a given port to output UBX, NMEA, RTCM3 or a combination thereof
//Port 0=I2c, 1=UART1, 2=UART2, 3=USB, 4=SPI
//Bit:0 = UBX, :1=NMEA, :5=RTCM3
/*void setPortOutput(uint8_t portID, uint8_t outStreamSettings, uint16_t maxWait)
{
  //Get the current config values for this port ID
  if (getPortSettings(portID, maxWait) == false)
    return (false); //Something went wrong. Bail.

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 20;
  packetCfg.startingSpot = 0;

  //payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[14] = outStreamSettings; //OutProtocolMask LSB - Set outStream bits

  return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}*/

void getPVT(void)
{
    //The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_PVT;
    packetCfg.len = 0;
    //packetCfg.startingSpot = 20; //Begin listening at spot 20 so we can record up to 20+MAX_PAYLOAD_SIZE = 84 bytes Note:now hard-coded in processUBX

    //The data is parsed as part of processing the response
    sendI2cCommand(&packetCfg);

    checkUbloxI2C(&packetCfg, UBX_CLASS_NAV, UBX_NAV_PVT); 
}

void checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
    uint16_t bytesAvailable = check_avail_bytes();
    /*serial0_sendString( "\nBytes available: ");
	char bytesString[10];
	itoa(bytesAvailable, bytesString, 10);
    serial0_sendString(bytesString);*/

    while (bytesAvailable)
    {
        uint16_t bytesToRead = bytesAvailable;
        if (bytesToRead > i2cTransactionSize)
        {
            bytesToRead = i2cTransactionSize;
        }

        uint8_t incoming[bytesToRead];
        read_i2c(gpsI2Caddress, incoming);
        for (uint16_t x = 0; x < bytesToRead; x++)
        {
          process(incoming[x], incomingUBX, requestedClass, requestedID);
        }


        
        bytesAvailable -= bytesToRead;
    }

}

void process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
    //usart_send(USART1, incoming);
    if ((currentSentence == NONE) || (currentSentence == NMEA))
    {
        if (incoming == 0xB5) //UBX binary frames start with 0xB5, aka μ
        {
            //This is the start of a binary sentence. Reset flags.
            //We still don't know the response class
            ubxFrameCounter = 0;
            currentSentence = UBX;
            //Reset the packetBuf.counter even though we will need to reset it again when ubxFrameCounter == 2
            packetBuf.counter = 0;
            ignoreThisPayload = 0; //We should not ignore this payload - yet
            //Store data in packetBuf until we know if we have a requested class and ID match
            activePacketBuffer = UBLOX_PACKET_PACKETBUF;
            serial0_sendString("Got UBX Packet\n");
        }
        else if (incoming == '$')
        {
            //serial0_sendString("Got NMEA Packet\n");
            currentSentence = NMEA;
        }
        else
        {
            //This character is unknown or we missed the previous start of a sentence
        }
    }

    //Depending on the sentence, pass the character to the individual processor
  if (currentSentence == UBX)
  {
    //Decide what type of response this is
    if ((ubxFrameCounter == 0) && (incoming != 0xB5))      //ISO 'μ'
      currentSentence = NONE;                              //Something went wrong. Reset.
    else if ((ubxFrameCounter == 1) && (incoming != 0x62)) //ASCII 'b'
      currentSentence = NONE;                              //Something went wrong. Reset.
    // Note to future self:
    // There may be some duplication / redundancy in the next few lines as processUBX will also
    // load information into packetBuf, but we'll do it here too for clarity
    else if (ubxFrameCounter == 2) //Class
    {
      // Record the class in packetBuf until we know what to do with it
      packetBuf.cls = incoming; // (Duplication)
      rollingChecksumA = 0;     //Reset our rolling checksums here (not when we receive the 0xB5)
      rollingChecksumB = 0;
      packetBuf.counter = 0;                                   //Reset the packetBuf.counter (again)
      packetBuf.valid = UBLOX_PACKET_VALIDITY_NOT_DEFINED; // Reset the packet validity (redundant?)
      packetBuf.startingSpot = incomingUBX->startingSpot;      //Copy the startingSpot
    }
    else if (ubxFrameCounter == 3) //ID
    {
      // Record the ID in packetBuf until we know what to do with it
      packetBuf.id = incoming; // (Duplication)
      //We can now identify the type of response
      //If the packet we are receiving is not an ACK then check for a class and ID match
      if (packetBuf.cls != UBX_CLASS_ACK)
      {
        //This is not an ACK so check for a class and ID match
        if ((packetBuf.cls == requestedClass) && (packetBuf.id == requestedID))
        {
          //This is not an ACK and we have a class and ID match
          //So start diverting data into incomingUBX (usually packetCfg)
          activePacketBuffer = UBLOX_PACKET_PACKETCFG;
          incomingUBX->cls = packetBuf.cls; //Copy the class and ID into incomingUBX (usually packetCfg)
          incomingUBX->id = packetBuf.id;
          incomingUBX->counter = packetBuf.counter; //Copy over the .counter too
        }
        //This is not an ACK and we do not have a complete class and ID match
        //So let's check for an HPPOSLLH message arriving when we were expecting PVT and vice versa
        else if ((packetBuf.cls == requestedClass) &&
          (((packetBuf.id == UBX_NAV_PVT) && (requestedID == UBX_NAV_HPPOSLLH)) ||
          ((packetBuf.id == UBX_NAV_HPPOSLLH) && (requestedID == UBX_NAV_PVT))))
        {
          //This is not the message we were expecting but we start diverting data into incomingUBX (usually packetCfg) and process it anyway
          activePacketBuffer = UBLOX_PACKET_PACKETCFG;
          incomingUBX->cls = packetBuf.cls; //Copy the class and ID into incomingUBX (usually packetCfg)
          incomingUBX->id = packetBuf.id;
          incomingUBX->counter = packetBuf.counter; //Copy over the .counter too
          serial0_sendString("process: auto PVT/HPPOSLLH collision: Requested ID: 0x");
        }
        else
        {
          //This is not an ACK and we do not have a class and ID match
          //so we should keep diverting data into packetBuf and ignore the payload
          ignoreThisPayload = 1;
        }
      }
      else
      {
        // This is an ACK so it is to early to do anything with it
        // We need to wait until we have received the length and data bytes
        // So we should keep diverting data into packetBuf
      }
    }
    else if (ubxFrameCounter == 4) //Length LSB
    {
      //We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len = incoming; // (Duplication)
    }
    else if (ubxFrameCounter == 5) //Length MSB
    {
      //We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len |= incoming << 8; // (Duplication)
    }
    else if (ubxFrameCounter == 6) //This should be the first byte of the payload unless .len is zero
    {
      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
      {
        serial0_sendString("process: ZERO LENGTH packet received: Class: 0x");
        //If length is zero (!) this will be the first byte of the checksum so record it
        packetBuf.checksumA = incoming;
      }
      else
      {
        //The length is not zero so record this byte in the payload
        packetBuf.payload[0] = incoming;
      }
    }
    else if (ubxFrameCounter == 7) //This should be the second byte of the payload unless .len is zero or one
    {
      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
      {
        //If length is zero (!) this will be the second byte of the checksum so record it
        packetBuf.checksumB = incoming;
      }
      else if (packetBuf.len == 1) // Check if length is one
      {
        //The length is one so this is the first byte of the checksum
        packetBuf.checksumA = incoming;
      }
      else // Length is >= 2 so this must be a payload byte
      {
        packetBuf.payload[1] = incoming;
      }
      // Now that we have received two payload bytes, we can check for a matching ACK/NACK
      if ((activePacketBuffer == UBLOX_PACKET_PACKETBUF) // If we are not already processing a data packet
          && (packetBuf.cls == UBX_CLASS_ACK)                // and if this is an ACK/NACK
          && (packetBuf.payload[0] == requestedClass)        // and if the class matches
          && (packetBuf.payload[1] == requestedID))          // and if the ID matches
      {
        if (packetBuf.len == 2) // Check if .len is 2
        {
          // Then this is a matching ACK so copy it into packetAck
          activePacketBuffer = UBLOX_PACKET_PACKETACK;
          packetAck.cls = packetBuf.cls;
          packetAck.id = packetBuf.id;
          packetAck.len = packetBuf.len;
          packetAck.counter = packetBuf.counter;
          packetAck.payload[0] = packetBuf.payload[0];
          packetAck.payload[1] = packetBuf.payload[1];
        }
        else // Length is not 2 (hopefully this is impossible!)
        {
            serial0_sendString("process: ACK received with .len != 2: Class: 0x");
        }
      }
    }

    //Divert incoming into the correct buffer
    if (activePacketBuffer == UBLOX_PACKET_PACKETACK)
    {
        serial0_sendString("ack packet process");
        processUBX(incoming, &packetAck, requestedClass, requestedID);
    }      
    else if (activePacketBuffer == UBLOX_PACKET_PACKETCFG)
    {
        serial0_sendString("CFG packet process");
        processUBX(incoming, incomingUBX, requestedClass, requestedID);
    }       
    else // if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF)
        processUBX(incoming, &packetBuf, requestedClass, requestedID);

    //Finally, increment the frame counter
    ubxFrameCounter++;
  }
  else if (currentSentence == NMEA)
  {
    //processNMEA(incoming); //Process each NMEA character
  }
}

void processUBX(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  //Add all incoming bytes to the rolling checksum
  //Stop at len+4 as this is the checksum bytes to that should not be added to the rolling checksum
  if (incomingUBX->counter < incomingUBX->len + 4)
    addToChecksum(incoming);

  if (incomingUBX->counter == 0)
  {
    incomingUBX->cls = incoming;
  }
  else if (incomingUBX->counter == 1)
  {
    incomingUBX->id = incoming;
  }
  else if (incomingUBX->counter == 2) //Len LSB
  {
    incomingUBX->len = incoming;
  }
  else if (incomingUBX->counter == 3) //Len MSB
  {
    incomingUBX->len |= incoming << 8;
  }
  else if (incomingUBX->counter == incomingUBX->len + 4) //ChecksumA
  {
    incomingUBX->checksumA = incoming;
  }
  else if (incomingUBX->counter == incomingUBX->len + 5) //ChecksumB
  {
    incomingUBX->checksumB = incoming;

    currentSentence = NONE; //We're done! Reset the sentence to being looking for a new start char

    //Validate this sentence
    if ((incomingUBX->checksumA == rollingChecksumA) && (incomingUBX->checksumB == rollingChecksumB))
    {
      incomingUBX->valid = UBLOX_PACKET_VALIDITY_VALID; // Flag the packet as valid

      // Let's check if the class and ID match the requestedClass and requestedID
      // Remember - this could be a data packet or an ACK packet
      if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
      {
        incomingUBX->classAndIDmatch = UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
      }

      // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
      }

      // If this is a NACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_NACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = UBLOX_PACKET_NOTACKNOWLEDGED; // If we have a match, set the classAndIDmatch flag to NOTACKNOWLEDGED

        serial0_sendString("processUBX: NACK received: Requested Class: 0x");
        /*serial0_sendString(itoa(incomingUBX->payload[0]));
        _debugSerial->print(F(" Requested ID: 0x"));
        _debugSerial->println(incomingUBX->payload[1], HEX);*/
      }

      //This is not an ACK and we do not have a complete class and ID match
      //So let's check for an HPPOSLLH message arriving when we were expecting PVT and vice versa
      else if ((incomingUBX->cls == requestedClass) &&
        (((incomingUBX->id == UBX_NAV_PVT) && (requestedID == UBX_NAV_HPPOSLLH)) ||
        ((incomingUBX->id == UBX_NAV_HPPOSLLH) && (requestedID == UBX_NAV_PVT))))
      {
        // This isn't the message we are looking for...
        // Let's say so and leave incomingUBX->classAndIDmatch _unchanged_
        /*
          _debugSerial->print(F("processUBX: auto PVT/HPPOSLLH collision: Requested ID: 0x"));
          _debugSerial->print(requestedID, HEX);
          _debugSerial->print(F(" Message ID: 0x"));
          _debugSerial->println(incomingUBX->id, HEX);
        */
      }

      /*
        _debugSerial->print(F("Incoming: Size: "));
        _debugSerial->print(incomingUBX->len);
        _debugSerial->print(F(" Received: "));
        printPacket(incomingUBX);

        if (incomingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetCfg now valid"));
        }
        if (packetAck.valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetAck now valid"));
        }
        if (incomingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetCfg classAndIDmatch"));
        }
        if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetAck classAndIDmatch"));
        }
      */

      //We've got a valid packet, now do something with it but only if ignoreThisPayload is false
      if (ignoreThisPayload == 0)
      {
        processUBXpacket(incomingUBX);
      }
    }
    else // Checksum failure
    {
      incomingUBX->valid = UBLOX_PACKET_VALIDITY_NOT_VALID;

      // Let's check if the class and ID match the requestedClass and requestedID.
      // This is potentially risky as we are saying that we saw the requested Class and ID
      // but that the packet checksum failed. Potentially it could be the class or ID bytes
      // that caused the checksum error!
      if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
      {
        incomingUBX->classAndIDmatch = UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
      }
      // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
      }

      /*
        //Drive an external pin to allow for easier logic analyzation
        if (checksumFailurePin >= 0)
        {
          digitalWrite((uint8_t)checksumFailurePin, LOW);
          delay(10);
          digitalWrite((uint8_t)checksumFailurePin, HIGH);
        }

        _debugSerial->print(F("Checksum failed:"));
        _debugSerial->print(F(" checksumA: "));
        _debugSerial->print(incomingUBX->checksumA);
        _debugSerial->print(F(" checksumB: "));
        _debugSerial->print(incomingUBX->checksumB);

        _debugSerial->print(F(" rollingChecksumA: "));
        _debugSerial->print(rollingChecksumA);
        _debugSerial->print(F(" rollingChecksumB: "));
        _debugSerial->print(rollingChecksumB);
        _debugSerial->println();

        _debugSerial->print(F("Failed  : "));
        _debugSerial->print(F("Size: "));
        _debugSerial->print(incomingUBX->len);
        _debugSerial->print(F(" Received: "));
        printPacket(incomingUBX);
      */
    }
  }
  else //Load this byte into the payload array
  {
    //If a UBX_NAV_PVT packet comes in asynchronously, we need to fudge the startingSpot
    uint16_t startingSpot = incomingUBX->startingSpot;
    if (incomingUBX->cls == UBX_CLASS_NAV && incomingUBX->id == UBX_NAV_PVT)
      startingSpot = 0;
    //Begin recording if counter goes past startingSpot
    if ((incomingUBX->counter - 4) >= startingSpot)
    {
      //Check to see if we have room for this byte
      if (((incomingUBX->counter - 4) - startingSpot) < MAX_PAYLOAD_SIZE) //If counter = 208, starting spot = 200, we're good to record.
      {
        // Check if this is payload data which should be ignored
        if (ignoreThisPayload == 0)
        {
          incomingUBX->payload[incomingUBX->counter - 4 - startingSpot] = incoming; //Store this byte into payload array
        }
      }
    }
  }

  //Increment the counter
  incomingUBX->counter++;

  if (incomingUBX->counter == MAX_PAYLOAD_SIZE)
  {
    //Something has gone very wrong
    currentSentence = NONE; //Reset the sentence to being looking for a new start char
    serial0_sendString("processUBX: counter hit MAX_PAYLOAD_SIZE");
  }
}

void processUBXpacket(ubxPacket *msg)
{
  switch (msg->cls)
  {
  case UBX_CLASS_NAV:
    if (msg->id == UBX_NAV_PVT && msg->len == 92)
    {
      //Parse various byte fields into global vars
      int startingSpot = 0; //fixed value used in processUBX

      timeOfWeek = extractLong(0);
      gpsMillisecond = extractLong(0) % 1000; //Get last three digits of iTOW
      gpsYear = extractInt(4);
      gpsMonth = extractByte(6);
      gpsDay = extractByte(7);
      gpsHour = extractByte(8);
      gpsMinute = extractByte(9);
      gpsSecond = extractByte(10);
      gpsDateValid = extractByte(11) & 0x01;
      gpsTimeValid = (extractByte(11) & 0x02) >> 1;
      gpsNanosecond = extractLong(16); //Includes milliseconds

      fixType = extractByte(20 - startingSpot);
      carrierSolution = extractByte(21 - startingSpot) >> 6; //Get 6th&7th bits of this byte
      SIV = extractByte(23 - startingSpot);
      ubx_longitude = extractLong(24 - startingSpot);
      ubx_latitude = extractLong(28 - startingSpot);
      ubx_altitude = extractLong(32 - startingSpot);
      altitudeMSL = extractLong(36 - startingSpot);
      groundSpeed = extractLong(60 - startingSpot);
      headingOfMotion = extractLong(64 - startingSpot);
      pDOP = extractInt(76 - startingSpot);

      //Mark all datums as fresh (not read before)
      moduleQueried.gpsiTOW = true;
      moduleQueried.gpsYear = true;
      moduleQueried.gpsMonth = true;
      moduleQueried.gpsDay = true;
      moduleQueried.gpsHour = true;
      moduleQueried.gpsMinute = true;
      moduleQueried.gpsSecond = true;
      moduleQueried.gpsDateValid = true;
      moduleQueried.gpsTimeValid = true;
      moduleQueried.gpsNanosecond = true;

      moduleQueried.all = true;
      moduleQueried.ubx_longitude = true;
      moduleQueried.ubx_latitude = true;
      moduleQueried.ubx_altitude = true;
      moduleQueried.altitudeMSL = true;
      moduleQueried.SIV = true;
      moduleQueried.fixType = true;
      moduleQueried.carrierSolution = true;
      moduleQueried.groundSpeed = true;
      moduleQueried.headingOfMotion = true;
      moduleQueried.pDOP = true;
    }
    break;
  }
}

//Given a spot in the payload array, extract four bytes and build a long
uint32_t extractLong(uint8_t spotToStart)
{
  uint32_t val = 0;
  val |= (uint32_t)payloadCfg[spotToStart + 0] << 8 * 0;
  val |= (uint32_t)payloadCfg[spotToStart + 1] << 8 * 1;
  val |= (uint32_t)payloadCfg[spotToStart + 2] << 8 * 2;
  val |= (uint32_t)payloadCfg[spotToStart + 3] << 8 * 3;
  return (val);
}

//Given a spot in the payload array, extract two bytes and build an int
uint16_t extractInt(uint8_t spotToStart)
{
  uint16_t val = 0;
  val |= (uint16_t)payloadCfg[spotToStart + 0] << 8 * 0;
  val |= (uint16_t)payloadCfg[spotToStart + 1] << 8 * 1;
  return (val);
}

//Given a spot, extract a byte from the payload
uint8_t extractByte(uint8_t spotToStart)
{
  return (payloadCfg[spotToStart]);
}

//Given a spot, extract a signed 8-bit value from the payload
int8_t extractSignedChar(uint8_t spotToStart)
{
  return ((int8_t)payloadCfg[spotToStart]);
}

//Given a message and a byte, add to rolling "8-Bit Fletcher" checksum
//This is used when receiving messages from module
void addToChecksum(uint8_t incoming)
{
  rollingChecksumA += incoming;
  rollingChecksumB += rollingChecksumA;
}

ublox_status_e waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  outgoingUBX->valid = UBLOX_PACKET_VALIDITY_NOT_DEFINED; //This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
  packetAck.valid = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.valid = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  outgoingUBX->classAndIDmatch = UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
  packetAck.classAndIDmatch = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.classAndIDmatch = UBLOX_PACKET_VALIDITY_NOT_DEFINED;

  unsigned long startTime = get_millis_elapsed();
  while (get_millis_elapsed() - startTime < maxTime)
  {
    checkUbloxI2C(outgoingUBX, requestedClass, requestedID);  //See if new data is available. Process bytes as they come in.

    // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
    // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
    // then we can be confident that the data in outgoingUBX is valid
    if ((outgoingUBX->classAndIDmatch == UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
    {
        serial0_sendString("waitForACKResponse: valid data and valid ACK received after ");

    return (SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data and a correct ACK!
    }

    // We can be confident that the data packet (if we are going to get one) will always arrive
    // before the matching ACK. So if we sent a config packet which only produces an ACK
    // then outgoingUBX->classAndIDmatch will be NOT_DEFINED and the packetAck.classAndIDmatch will VALID.
    // We should not check outgoingUBX->valid, outgoingUBX->cls or outgoingUBX->id
    // as these may have been changed by a PVT packet.
    else if ((outgoingUBX->classAndIDmatch == UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (packetAck.classAndIDmatch == UBLOX_PACKET_VALIDITY_VALID))
    {
        serial0_sendString("waitForACKResponse: no data and valid ACK after ");
    return (SFE_UBLOX_STATUS_DATA_SENT); //We got an ACK but no data...
    }

    // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
    // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
    // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
    // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
    // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
    // So we cannot use outgoingUBX->valid as part of this check.
    // Note: the addition of packetBuf should make this check redundant!
    else if ((outgoingUBX->classAndIDmatch == UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == UBLOX_PACKET_VALIDITY_VALID) && !((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
    {
        serial0_sendString("waitForACKResponse: data being OVERWRITTEN after ");
    return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
    }

    // If packetAck.classAndIDmatch is VALID but both outgoingUBX->valid and outgoingUBX->classAndIDmatch
    // are NOT_VALID then we can be confident we have had a checksum failure on the data packet
    else if ((packetAck.classAndIDmatch == UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->classAndIDmatch == UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == UBLOX_PACKET_VALIDITY_NOT_VALID))
    {
        serial0_sendString("waitForACKResponse: CRC failed after ");
    return (SFE_UBLOX_STATUS_CRC_FAIL); //Checksum fail
    }

    // If our packet was not-acknowledged (NACK) we do not receive a data packet - we only get the NACK.
    // So you would expect outgoingUBX->valid and outgoingUBX->classAndIDmatch to still be NOT_DEFINED
    // But if a full PVT packet arrives afterwards outgoingUBX->valid could be VALID (or just possibly NOT_VALID)
    // but outgoingUBX->cls and outgoingUBX->id would not match...
    // So I think this is telling us we need a special state for packetAck.classAndIDmatch to tell us
    // the packet was definitely NACK'd otherwise we are possibly just guessing...
    // Note: the addition of packetBuf changes the logic of this, but we'll leave the code as is for now.
    else if (packetAck.classAndIDmatch == UBLOX_PACKET_NOTACKNOWLEDGED)
    {
        serial0_sendString("waitForACKResponse: data was NOTACKNOWLEDGED (NACK) after ");
    return (SFE_UBLOX_STATUS_COMMAND_NACK); //We received a NACK!
    }

    // If the outgoingUBX->classAndIDmatch is VALID but the packetAck.classAndIDmatch is NOT_VALID
    // then the ack probably had a checksum error. We will take a gamble and return DATA_RECEIVED.
    // If we were playing safe, we should return FAIL instead
    else if ((outgoingUBX->classAndIDmatch == UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
    {
        serial0_sendString("waitForACKResponse: VALID data and INVALID ACK received after ");
    return (SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data and an invalid ACK!
    }

    // If the outgoingUBX->classAndIDmatch is NOT_VALID and the packetAck.classAndIDmatch is NOT_VALID
    // then we return a FAIL. This must be a double checksum failure?
    else if ((outgoingUBX->classAndIDmatch == UBLOX_PACKET_VALIDITY_NOT_VALID) && (packetAck.classAndIDmatch == UBLOX_PACKET_VALIDITY_NOT_VALID))
    {
        serial0_sendString("waitForACKResponse: INVALID data and INVALID ACK received after ");
    return (SFE_UBLOX_STATUS_FAIL); //We received invalid data and an invalid ACK!
    }

    // If the outgoingUBX->classAndIDmatch is VALID and the packetAck.classAndIDmatch is NOT_DEFINED
    // then the ACK has not yet been received and we should keep waiting for it
    else if ((outgoingUBX->classAndIDmatch == UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == UBLOX_PACKET_VALIDITY_NOT_DEFINED))
    {
        serial0_sendString("waitForACKResponse: valid data after ");
    }

    delay(10);
  } //while (millis() - startTime < maxTime)

  // We have timed out...
  // If the outgoingUBX->classAndIDmatch is VALID then we can take a gamble and return DATA_RECEIVED
  // even though we did not get an ACK
  if ((outgoingUBX->classAndIDmatch == UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == 
  UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
  {
      serial0_sendString("waitForACKResponse: TIMEOUT with valid data after ");
    return (SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data... But no ACK!
  }
    
    serial0_sendString("waitForACKResponse: TIMEOUT after ");

    return (SFE_UBLOX_STATUS_TIMEOUT);
}

//For non-CFG queries no ACK is sent so we use this function
//Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got a config packet full of response data that has CLS/ID match to our query packet
//Returns SFE_UBLOX_STATUS_CRC_FAIL if we got a corrupt config packet that has CLS/ID match to our query packet
//Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
//Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an a valid packetCfg but that the packetCfg has been
// or is currently being overwritten (remember that Serial data can arrive very slowly)
ublox_status_e waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  outgoingUBX->valid = UBLOX_PACKET_VALIDITY_NOT_DEFINED; //This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
  packetAck.valid = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.valid = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  outgoingUBX->classAndIDmatch = UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
  packetAck.classAndIDmatch = UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.classAndIDmatch = UBLOX_PACKET_VALIDITY_NOT_DEFINED;

  unsigned long startTime = get_millis_elapsed();
  while (get_millis_elapsed() - startTime < maxTime)
  {
    checkUbloxI2C(outgoingUBX, requestedClass, requestedID); //See if new data is available. Process bytes as they come in.

    // If outgoingUBX->classAndIDmatch is VALID
    // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
    // then we can be confident that the data in outgoingUBX is valid
    if ((outgoingUBX->classAndIDmatch == UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
    {
        serial0_sendString("waitForNoACKResponse: valid data with CLS/ID match after ");
    return (SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data!
    }

    // If the outgoingUBX->classAndIDmatch is VALID
    // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
    // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
    // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
    // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
    // So we cannot use outgoingUBX->valid as part of this check.
    // Note: the addition of packetBuf should make this check redundant!
    else if ((outgoingUBX->classAndIDmatch == UBLOX_PACKET_VALIDITY_VALID) && !((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
    {
        serial0_sendString("waitForNoACKResponse: data being OVERWRITTEN after ");
    return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
    }

    // If outgoingUBX->classAndIDmatch is NOT_DEFINED
    // and outgoingUBX->valid is VALID then this must be (e.g.) a PVT packet
    else if ((outgoingUBX->classAndIDmatch == UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == UBLOX_PACKET_VALIDITY_VALID))
    {
    serial0_sendString("waitForNoACKResponse: valid but UNWANTED data after ");
    }

    // If the outgoingUBX->classAndIDmatch is NOT_VALID then we return CRC failure
    else if (outgoingUBX->classAndIDmatch == UBLOX_PACKET_VALIDITY_NOT_VALID)
    {
        serial0_sendString("waitForNoACKResponse: CLS/ID match but failed CRC after ");
    return (SFE_UBLOX_STATUS_CRC_FAIL); //We received invalid data
    }

    delay(5);
  }

    serial0_sendString("waitForNoACKResponse: TIMEOUT after ");


    return (SFE_UBLOX_STATUS_TIMEOUT);
}