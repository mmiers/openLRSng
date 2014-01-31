/*
    Implementation of PSP (Phoenix Serial Protocol)

    Protocol data structure:
    [SYNC1][SYNC2][CODE][LENGTH_L][LENGTH_H][DATA/DATA ARRAY][CRC]
*/

bool binary_mode_active = false;

#define PSP_SYNC1 0xB5
#define PSP_SYNC2 0x62

#define PSP_REQ_BIND_DATA             1
#define PSP_REQ_RX_CONFIG             2
#define PSP_REQ_RX_JOIN_CONFIGURATION 3
#define PSP_REQ_SCANNER_MODE          4
#define PSP_REQ_SPECIAL_PINS          5
#define PSP_REQ_FW_VERSION            6
#define PSP_REQ_NUMBER_OF_RX_OUTPUTS  7
#define PSP_REQ_ACTIVE_PROFILE        8

#define PSP_SET_BIND_DATA          101
#define PSP_SET_RX_CONFIG          102
#define PSP_SET_TX_SAVE_EEPROM     103
#define PSP_SET_RX_SAVE_EEPROM     104
#define PSP_SET_TX_RESTORE_DEFAULT 105
#define PSP_SET_RX_RESTORE_DEFAULT 106
#define PSP_SET_ACTIVE_PROFILE     107

#define PSP_SET_EXIT               199

#define PSP_INF_ACK           201
#define PSP_INF_REFUSED       202
#define PSP_INF_CRC_FAIL      203
#define PSP_INF_DATA_TOO_LONG 204

extern struct rxSpecialPinMap rxcSpecialPins[];
extern uint8_t rxcSpecialPinCount;
extern uint8_t rxcNumberOfOutputs;
extern uint16_t rxcVersion;
uint8_t rxcConnect();

typedef struct binary_com {
  uint8_t data; // variable used to store a single byte from serial

  uint8_t state;
  uint8_t code;
  uint8_t message_crc;
  uint8_t crc;

  uint16_t payload_length_expected;
  uint16_t payload_length_received;

  uint8_t data_buffer[100];
} BinaryPSP;

BinaryPSP binary_com;

/// Constructor
void
PSP_init(BinaryPSP *psp)
{
  psp->state = 0;

  psp->payload_length_expected = 0;
  psp->payload_length_received = 0;
}

void PSP_serialize_uint8(BinaryPSP *psp, uint8_t data)
{
  Serial.write(data);
  psp->crc ^= data;
}

void PSP_serialize_uint16(BinaryPSP *psp, uint16_t data)
{
  PSP_serialize_uint8(psp, lowByte(data));
  PSP_serialize_uint8(psp, highByte(data));
}

void PSP_serialize_uint32(BinaryPSP *psp, uint32_t data) {
  for (uint8_t i = 0; i < 4; i++) {
    PSP_serialize_uint8(psp, (uint8_t) (data >> (i * 8)));
  }
}

void PSP_serialize_uint64(BinaryPSP *psp, uint64_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    PSP_serialize_uint8(psp, (uint8_t) (data >> (i * 8)));
  }
}

void PSP_serialize_float32(BinaryPSP *psp, float f) {
  uint8_t *b = (uint8_t*)&f;

  for (uint8_t i = 0; i < sizeof(f); i++) {
    PSP_serialize_uint8(psp, b[i]);
  }
}

void PSP_protocol_head(BinaryPSP *psp, uint8_t code, uint16_t length)
{
  Serial.write(PSP_SYNC1);
  Serial.write(PSP_SYNC2);

  psp->crc = 0; // reset crc

  PSP_serialize_uint8(psp, code);
  PSP_serialize_uint16(psp, length);
}

void PSP_protocol_tail(BinaryPSP *psp)
{
  Serial.write(psp->crc);
}

void PSP_ACK(BinaryPSP *psp) {
  PSP_protocol_head(psp, PSP_INF_ACK, 1);

  PSP_serialize_uint8(psp, 0x01);
}

void PSP_REFUSED(BinaryPSP *psp) {
  PSP_protocol_head(psp, PSP_INF_REFUSED, 1);

  PSP_serialize_uint8(psp, 0x00);
}

void PSP_CRC_FAILED(BinaryPSP *psp, uint8_t code, uint8_t failed_crc)
{
  PSP_protocol_head(psp, PSP_INF_CRC_FAIL, 2);

  PSP_serialize_uint8(psp, code);
  PSP_serialize_uint8(psp, failed_crc);
}

void PSP_process_data(BinaryPSP *psp)
{
  switch (psp->code) {
  case PSP_REQ_BIND_DATA:
    PSP_protocol_head(psp, PSP_REQ_BIND_DATA, sizeof(bind_data));
    {
      char* array = (char*) &bind_data;
      for (uint16_t i = 0; i < sizeof(bind_data); i++) {
        PSP_serialize_uint8(psp, array[i]);
      }
    }
    break;
  case PSP_REQ_RX_CONFIG:
    PSP_protocol_head(psp, PSP_REQ_RX_CONFIG, sizeof(rx_config));
    {
      char* array = (char*) &rx_config;
      for (uint16_t i = 0; i < sizeof(rx_config); i++) {
        PSP_serialize_uint8(psp, array[i]);
      }
    }
    break;
  case PSP_REQ_RX_JOIN_CONFIGURATION:
    PSP_protocol_head(psp, PSP_REQ_RX_JOIN_CONFIGURATION, 1);
    // 1 success, 2 timeout, 3 failed response

    PSP_serialize_uint8(psp, rxcConnect());

    break;
  case PSP_REQ_SCANNER_MODE:
    PSP_protocol_head(psp, PSP_REQ_SCANNER_MODE, 1);
    PSP_serialize_uint8(psp, 0x01);
    PSP_protocol_tail(psp);

    scannerMode();

    return;
    break;
  case PSP_REQ_SPECIAL_PINS:
    PSP_protocol_head(psp, PSP_REQ_SPECIAL_PINS, sizeof(struct rxSpecialPinMap) * rxcSpecialPinCount);
    {
      char* array = (char*) &rxcSpecialPins;
      for (uint16_t i = 0; i < sizeof(struct rxSpecialPinMap) * rxcSpecialPinCount; i++) {
        PSP_serialize_uint8(psp, array[i]);
      }
    }
    break;
  case PSP_REQ_FW_VERSION:
    PSP_protocol_head(psp, PSP_REQ_FW_VERSION, sizeof(version));
    {
      PSP_serialize_uint16(psp, version);
    }
    break;
  case PSP_REQ_NUMBER_OF_RX_OUTPUTS:
    PSP_protocol_head(psp, PSP_REQ_NUMBER_OF_RX_OUTPUTS, 1);
    {
      PSP_serialize_uint8(psp, rxcNumberOfOutputs);
    }
    break;
  case PSP_REQ_ACTIVE_PROFILE:
    PSP_protocol_head(psp, PSP_REQ_ACTIVE_PROFILE, 1);
    {
      PSP_serialize_uint8(psp, activeProfile);
    }
    break;
    // SET
  case PSP_SET_BIND_DATA:
    PSP_protocol_head(psp, PSP_SET_BIND_DATA, 1);

    if (psp->payload_length_received == sizeof(bind_data)) {
      char* array = (char*) &bind_data;

      for (uint16_t i = 0; i < sizeof(bind_data); i++) {
        array[i] = psp->data_buffer[i];
      }

      PSP_serialize_uint8(psp, 0x01);
    } else {
      // fail (buffer size doesn't match struct memory size)
      PSP_serialize_uint8(psp, 0x00);
    }
    break;
  case PSP_SET_RX_CONFIG:
    PSP_protocol_head(psp, PSP_SET_RX_CONFIG, 1);

    if (psp->payload_length_received == sizeof(rx_config)) {
      char* array = (char*) &rx_config;

      for (uint16_t i = 0; i < sizeof(rx_config); i++) {
        array[i] = psp->data_buffer[i];
      }

      PSP_serialize_uint8(psp, 0x01);
    } else {
      // fail (buffer size doesn't match struct memory size)
      PSP_serialize_uint8(psp, 0x00);
    }
    break;
  case PSP_SET_TX_SAVE_EEPROM:
    PSP_protocol_head(psp, PSP_SET_TX_SAVE_EEPROM, 1);
    bindWriteEeprom();
    PSP_serialize_uint8(psp, 0x01); // success
    break;
  case PSP_SET_RX_SAVE_EEPROM:
    PSP_protocol_head(psp, PSP_SET_RX_SAVE_EEPROM, 1);
    // 1 success, 0 fail

    {
      uint8_t tx_buf[1 + sizeof(rx_config)];
      tx_buf[0] = 'u';
      memcpy(tx_buf + 1, &rx_config, sizeof(rx_config));
      tx_packet(tx_buf, sizeof(rx_config) + 1);
      rx_reset();
      RF_Mode = Receive;
      delay(200);

      if (RF_Mode == Received) {
        spiSendAddress(0x7f); // Send the package read command
        tx_buf[0] = spiReadData();
        if (tx_buf[0]=='U') {
          PSP_serialize_uint8(psp, 0x01); // success
        } else {
          PSP_serialize_uint8(psp, 0x00); // fail
        }
      } else {
        PSP_serialize_uint8(psp, 0x00); // fail
      }
    }
    break;
  case PSP_SET_TX_RESTORE_DEFAULT:
    PSP_protocol_head(psp, PSP_SET_TX_RESTORE_DEFAULT, 1);

    bindInitDefaults();

    PSP_serialize_uint8(psp, 0x01); // done
    break;
  case PSP_SET_RX_RESTORE_DEFAULT:
    PSP_protocol_head(psp, PSP_SET_RX_RESTORE_DEFAULT, 1);
    // 1 success, 0 fail

    uint8_t tx_buf[1 + sizeof(rx_config)];
    tx_buf[0] = 'i';
    tx_packet(tx_buf,1);
    rx_reset();
    RF_Mode = Receive;
    delay(200);

    if (RF_Mode == Received) {
      spiSendAddress(0x7f);   // Send the package read command
      tx_buf[0] = spiReadData();

      for (uint8_t i = 0; i < sizeof(rx_config); i++) {
        tx_buf[i + 1] = spiReadData();
      }

      memcpy(&rx_config, tx_buf + 1, sizeof(rx_config));

      if (tx_buf[0] == 'I') {
        PSP_serialize_uint8(psp, 0x01); // success
      } else {
        PSP_serialize_uint8(psp, 0x00); // fail
      }
    } else {
      PSP_serialize_uint8(psp, 0x00); // fail
    }
    break;
  case PSP_SET_ACTIVE_PROFILE:
    PSP_protocol_head(psp, PSP_SET_ACTIVE_PROFILE, 1);

    profileSwap(psp->data_buffer[0]);
    if (!bindReadEeprom()) {
      bindInitDefaults();
      bindWriteEeprom();
    }
    PSP_serialize_uint8(psp, 0x01); // done
    break;
  case PSP_SET_EXIT:
    PSP_protocol_head(psp, PSP_SET_EXIT, 1);
    PSP_serialize_uint8(psp, 0x01);
    PSP_protocol_tail(psp);

    binary_mode_active = false;

    return;
    break;
  default: // Unrecognized code
    PSP_REFUSED(psp);
  }

  // send over crc
  PSP_protocol_tail(psp);
}

void
PSP_read_packet(BinaryPSP *psp)
{
  while (Serial.available()) {
    psp->data = Serial.read();

    switch (psp->state) {
    case 0:
      if (psp->data == PSP_SYNC1) {
        psp->state++;
      }
      break;
    case 1:
      if (psp->data == PSP_SYNC2) {
        psp->state++;
      } else {
        psp->state = 0; // Restart and try again
      }
      break;
    case 2:
      psp->code = psp->data;
      psp->message_crc = psp->data;

      psp->state++;
      break;
    case 3: // LSB
      psp->payload_length_expected = psp->data;
      psp->message_crc ^= psp->data;

      psp->state++;
      break;
    case 4: // MSB
      psp->payload_length_expected |= psp->data << 8;
      psp->message_crc ^= psp->data;

      psp->state++;

      if (psp->payload_length_expected > sizeof(psp->data_buffer)) {
        // Message too long, we won't accept
        PSP_protocol_head(psp, PSP_INF_DATA_TOO_LONG, 1);
        PSP_serialize_uint8(psp, 0x01);
        PSP_protocol_tail(psp);

        psp->state = 0; // Restart
      }
      break;
    case 5:
      psp->data_buffer[psp->payload_length_received] = psp->data;
      psp->message_crc ^= psp->data;
      psp->payload_length_received++;

      if (psp->payload_length_received >= psp->payload_length_expected) {
        psp->state++;
      }
      break;
    case 6:
      if (psp->message_crc == psp->data) {
        // CRC is ok, process data
        PSP_process_data(psp);
      } else {
        // respond that CRC failed
        PSP_CRC_FAILED(psp, psp->code, psp->message_crc);
      }

      // reset variables
      memset(psp->data_buffer, 0, sizeof(psp->data_buffer));

      psp->payload_length_received = 0;
      psp->state = 0;
      break;
    }
  }
}
