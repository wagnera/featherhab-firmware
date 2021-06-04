#ifndef UBX_H_
#define UBX_H_

#include <stdlib.h>
#include <stdint.h>

#define UBX_NAV_PVT  0x07		//All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites. Navigation Position Velocity Time Solution.
#define UBX_NAV_HPPOSLLH 0x14	//High Precision Geodetic Position Solution. Used for obtaining lat/long/alt in high precision

#define UBX_CFG_PRT 0x00		//Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port

#define UBX_CLASS_NAV 0x01	 //Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
#define UBX_CLASS_ACK  0x05	 //Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
#define UBX_CLASS_CFG 0x06	 //Configuration Input Messages: Configure the receiver.

#define MAX_PAYLOAD_SIZE 256 //We need ~220 bytes for getProtocolVersion on most ublox modules

//Registers
#define UBX_SYNCH_1 0xB5
#define UBX_SYNCH_2 0x62

#define UBX_ACK_NACK 0x00
#define UBX_ACK_ACK 0x01
#define UBX_ACK_NONE 0x02 //Not a real value

// ubxPacket validity
typedef enum
{
	UBLOX_PACKET_VALIDITY_NOT_VALID,
	UBLOX_PACKET_VALIDITY_VALID,
	UBLOX_PACKET_VALIDITY_NOT_DEFINED,
	UBLOX_PACKET_NOTACKNOWLEDGED // This indicates that we received a NACK
} ublox_packet_validity_e;

typedef enum
{
    NONE,
    NMEA,
    UBX,
    RTCM
} SentenceTypes;

// Identify which packet buffer is in use:
// packetCfg (or a custom packet), packetAck or packetBuf
typedef enum
{
	UBLOX_PACKET_PACKETCFG,
	UBLOX_PACKET_PACKETACK,
	UBLOX_PACKET_PACKETBUF
} ublox_packet_buffer_e;

//-=-=-=-=- UBX binary specific variables
typedef struct
{
	uint8_t cls;
	uint8_t id;
	uint16_t len;		   //Length of the payload. Does not include cls, id, or checksum bytes
	uint16_t counter;	   //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
	uint16_t startingSpot; //The counter value needed to go past before we begin recording into payload array
	uint8_t *payload;
	uint8_t checksumA; //Given to us from module. Checked against the rolling calculated A/B checksums.
	uint8_t checksumB;
	ublox_packet_validity_e valid;			 //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
	ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
} ubxPacket;

typedef enum
{
	SFE_UBLOX_STATUS_SUCCESS,
	SFE_UBLOX_STATUS_FAIL,
	SFE_UBLOX_STATUS_CRC_FAIL,
	SFE_UBLOX_STATUS_TIMEOUT,
	SFE_UBLOX_STATUS_COMMAND_NACK, // Indicates that the command was unrecognised, invalid or that the module is too busy to respond
	SFE_UBLOX_STATUS_OUT_OF_RANGE,
	SFE_UBLOX_STATUS_INVALID_ARG,
	SFE_UBLOX_STATUS_INVALID_OPERATION,
	SFE_UBLOX_STATUS_MEM_ERR,
	SFE_UBLOX_STATUS_HW_ERR,
	SFE_UBLOX_STATUS_DATA_SENT,		// This indicates that a 'set' was successful
	SFE_UBLOX_STATUS_DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
	SFE_UBLOX_STATUS_I2C_COMM_FAILURE,
	SFE_UBLOX_STATUS_DATA_OVERWRITTEN // This is an error - the data was valid but has been or _is being_ overwritten by another packet
} ublox_status_e;

typedef enum
{
    TRUE,
    FALSE
} ubx_bool;

//Functions:
void calcChecksum(ubxPacket *msg);
uint16_t check_avail_bytes(void);
void sendI2cCommand(ubxPacket *outgoingUBX);
void getPortSettings(void);
void getPVT(void);
void checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);
void process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);
void processUBX(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);
void processUBXpacket(ubxPacket *msg);
uint32_t extractLong(uint8_t spotToStart);																	 //Combine four bytes from payload into long
uint16_t extractInt(uint8_t spotToStart);																	 //Combine two bytes from payload into int
uint8_t extractByte(uint8_t spotToStart);																	 //Get byte from payload
int8_t extractSignedChar(uint8_t spotToStart);																 //Get signed 8-bit value from payload
void addToChecksum(uint8_t incoming);
ublox_status_e waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime);
ublox_status_e waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime);

#endif /* UBX_ */