
//
// Copyright (c) 2012 Andrew Tridgell, All Rights Reserved
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  o Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  o Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
//

///
/// mavlink reporting code
///

#define MAVLINK10_STX 254

#define MAVLINK_MSG_ID_RADIO 166
#define MAVLINK_RADIO_CRC_EXTRA 21

// new RADIO_STATUS common message
#define MAVLINK_MSG_ID_RADIO_STATUS 109
#define MAVLINK_RADIO_STATUS_CRC_EXTRA 185

// use '3D' for 3DRadio
#define RADIO_SOURCE_SYSTEM '3'
#define RADIO_SOURCE_COMPONENT 'D'

/*
    we use a hand-crafted MAVLink packet based on the following
    message definition

    <message name="RADIO" id="166">
      <description>Status generated by radio</description>
      <field type="uint8_t" name="rssi">local signal strength</field>
      <field type="uint8_t" name="remrssi">remote signal strength</field>
      <field type="uint8_t" name="txbuf">percentage free space in transmit buffer</field>
      <field type="uint8_t" name="noise">background noise level</field>
      <field type="uint8_t" name="remnoise">remote background noise level</field>
      <field type="uint16_t" name="rxerrors">receive errors</field>
      <field type="uint16_t" name="fixed">count of error corrected packets</field>
    </message>

    Note that the wire field ordering follows the MAVLink v1.0
    spec 

    The RADIO_STATUS message in common.xml is the same as the
    ardupilotmega.xml RADIO message, but is message ID 109 with
    a different CRC seed
*/
struct mavlink_RADIO_v10 {
  uint16_t rxerrors;
  uint16_t fixed;
  uint8_t rssi;
  uint8_t remrssi;
  uint8_t txbuf;
  uint8_t noise;
  uint8_t remnoise;
};

#define MAV_HEADER_SIZE       6
#define MAV_MAX_PACKET_LENGTH (MAV_HEADER_SIZE + sizeof(struct mavlink_RADIO_v10) + 2)

static uint8_t mav_pbuf[MAV_MAX_PACKET_LENGTH];
static uint8_t mav_seqnum = 0;

/*
 * Calculates the MAVLink checksum on a packet in pbuf[] 
 * and append it after the data
 */
static void mavlink_crc(register uint8_t crc_extra)
{
  register uint8_t length = mav_pbuf[1];
  uint16_t sum = 0xFFFF;
  uint8_t i, stoplen;

  stoplen = length + 6;

  // MAVLink 1.0 has an extra CRC seed
  mav_pbuf[length+6] = crc_extra;
  stoplen++;

  i = 1;
  while (i<stoplen) {
    register uint8_t tmp;
    tmp = mav_pbuf[i] ^ (uint8_t)(sum&0xff);
    tmp ^= (tmp<<4);
    sum = (sum>>8) ^ (tmp<<8) ^ (tmp<<3) ^ (tmp>>4);
    i++;
  }

  mav_pbuf[length+6] = sum&0xFF;
  mav_pbuf[length+7] = sum>>8;
}

/*
 * Return the amount of space left in the receive buffer
 */
uint16_t
serialReadSpace()
{
  uint16_t space = SERIAL_BUFFER_SIZE - TelemetrySerial.available();
  space = (100 * (space / 8)) / (SERIAL_BUFFER_SIZE / 8);
  return space;
}

/*
 * Borrow this from the Serial implementation for
 * computing TX space
 */
struct ring_buffer
{
  unsigned char buffer[SERIAL_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
};

uint16_t
serialWriteSpace()
{
  ring_buffer *ring = 0;
#if defined(USBCON)
  extern ring_buffer tx_buffer;

  if (&TelemetrySerial == &Serial1)
    ring = &tx_buffer;
#endif
#if defined(UBRRH) || defined(UBRR0H)
  extern ring_buffer tx_buffer;

  if (&TelemetrySerial == &Serial)
    ring = &tx_buffer;
#endif
#if defined(UBRR1H)
  extern ring_buffer tx_buffer1;

  if (&TelemetrySerial == &Serial1)
    ring = &tx_buffer1;
#endif
#if defined(UBRR2H)
  extern ring_buffer tx_buffer2;

  if (&TelemetrySerial == &Serial2)
    ring = &tx_buffer2;
#endif
#if defined(UBRR3H)
  extern ring_buffer tx_buffer3;

  if (&TelemetrySerial == &Serial3)
    ring = &tx_buffer3;
#endif

  if (ring)
    return SERIAL_BUFFER_SIZE - ((unsigned int)(SERIAL_BUFFER_SIZE + ring->head - ring->tail) % SERIAL_BUFFER_SIZE);

  return 0;
}

void
serialWriteBuf(uint8_t *buf, size_t size)
{
  size_t i;

  for (i = 0; i < size; i++)
    TelemetrySerial.write(buf[i]);
}

/// send a MAVLink status report packet
void MAVLink_report(uint8_t RSSI_remote, uint16_t RSSI_local, uint16_t rxerrors)
{
  struct mavlink_RADIO_v10 *m = (struct mavlink_RADIO_v10 *)&mav_pbuf[6];
  mav_pbuf[0] = MAVLINK10_STX;
  mav_pbuf[1] = sizeof(struct mavlink_RADIO_v10);
  mav_pbuf[2] = mav_seqnum++;
  mav_pbuf[3] = RADIO_SOURCE_SYSTEM;
  mav_pbuf[4] = RADIO_SOURCE_COMPONENT;
  mav_pbuf[5] = MAVLINK_MSG_ID_RADIO;

  m->rxerrors = rxerrors;
  m->fixed    = 0;
  m->txbuf    = serialReadSpace();
  m->rssi     = RSSI_local;
  m->remrssi  = RSSI_remote;
  m->noise    = 0;
  m->remnoise = 0;
  mavlink_crc(MAVLINK_RADIO_CRC_EXTRA);

  if (serialWriteSpace() < (int)sizeof(struct mavlink_RADIO_v10)+8) {
    // don't cause an overflow
    return;
  }

  serialWriteBuf(mav_pbuf, sizeof(struct mavlink_RADIO_v10)+8);

  // now the new RADIO_STATUS common message
  mav_pbuf[5] = MAVLINK_MSG_ID_RADIO_STATUS;
  mavlink_crc(MAVLINK_RADIO_STATUS_CRC_EXTRA);

  if (serialWriteSpace() < (int)sizeof(struct mavlink_RADIO_v10)+8) {
    // don't cause an overflow
    return;
  }

  serialWriteBuf(mav_pbuf, sizeof(struct mavlink_RADIO_v10)+8);
}

uint32_t last_mavlinkInject_time = 0;

//####
//#### MAVLink frame detector
//#### Pretty much lifted verbatim from APM
//####

#define X25_INIT_CRC         0xffff
#define X25_VALIDATE_CRC     0xf0b8
#define MAVLINK_PACKET_START 0xFE

/*
 * Frame detect logic
 */
enum MavlinkParseState
{
  MavParse_Idle,
  MavParse_PayloadLength,
  MavParse_PacketSequence,
  MavParse_SystemID,
  MavParse_ComponentID,
  MavParse_MessageID,
  MavParse_Payload,
  MavParse_CRC1,
  MavParse_CRC2
};

static uint8_t mav_payloadLength = 0;
static uint8_t mav_packetSequence = 0;
static uint8_t mav_systemID = 0;
static uint8_t mav_componentID = 0;
static uint8_t mav_messageID = 0;
static uint16_t mav_CRC = 0;
static uint8_t mav_payloadByteParsedCount = 0;
static enum MavlinkParseState mav_state = MavParse_Idle;

void MAVLink_FDReset()
{
  mav_payloadLength = 0;
  mav_packetSequence = 0;
  mav_systemID = 0;
  mav_componentID = 0;
  mav_messageID = 0;
  mav_CRC = 0;

  // Setup helpers
  mav_payloadByteParsedCount = 0; // clear helper
  mav_state = MavParse_Idle;
}

// Returns true if a mavlink frame has been detected.
bool MAVLink_detectFrame(uint8_t ch)
{
  switch (mav_state)
  {
  case MavParse_Idle:
    if (ch == MAVLINK_PACKET_START) {
      MAVLink_FDReset();
      mav_state = MavParse_PayloadLength;
    }
    break;
  case MavParse_PayloadLength:
    mav_payloadLength = ch;
    mav_state = MavParse_PacketSequence;
    break;
  case MavParse_PacketSequence:
    mav_packetSequence = ch;
    mav_state = MavParse_SystemID;
    break;
  case MavParse_SystemID:
    mav_systemID = ch;
    mav_state = MavParse_ComponentID;
    break;
  case MavParse_ComponentID:
    mav_componentID = ch;
    mav_state = MavParse_MessageID;
    break;
  case MavParse_MessageID:
    mav_messageID = ch;
    mav_state = MavParse_Payload;
    break;
  case MavParse_Payload:
    if (++mav_payloadByteParsedCount >= mav_payloadLength) {
      mav_state = MavParse_CRC1;
    }
    break;
  case MavParse_CRC1:
    mav_CRC = ch;
    mav_state = MavParse_CRC2;
    break;
  case MavParse_CRC2:
    mav_CRC |= (uint16_t)ch << 8;
    mav_state = MavParse_Idle;
    return true;
  }
  return false;
}

