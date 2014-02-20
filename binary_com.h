/*
    Implementation of PSP (Phoenix Serial Protocol)

    Protocol data structure:
    [SYNC1][SYNC2][CODE][LENGTH_L][LENGTH_H][DATA/DATA ARRAY][CRC]
*/

boolean binary_mode_active = false;

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


class binary_PSP
{
public:
  // Constructor
  binary_PSP() {
    state = 0;

    payload_length_expected = 0;
    payload_length_received = 0;
  };

  void read_packet() {
    while (Serial.available()) {
      data = Serial.read();

      switch (state) {
      case 0:
        if (data == PSP_SYNC1) {
          state++;
        }
        break;
      case 1:
        if (data == PSP_SYNC2) {
          state++;
        } else {
          state = 0; // Restart and try again
        }
        break;
      case 2:
        code = data;
        message_crc = data;

        state++;
        break;
      case 3: // LSB
        payload_length_expected = data;
        message_crc ^= data;

        state++;
        break;
      case 4: // MSB
        payload_length_expected |= data << 8;
        message_crc ^= data;

        state++;

        if (payload_length_expected > sizeof(data_buffer)) {
          // Message too long, we won't accept
          protocol_head(PSP_INF_DATA_TOO_LONG, 1);
          serialize_uint8(0x01);
          protocol_tail();

          state = 0; // Restart
        }
        break;
      case 5:
        data_buffer[payload_length_received] = data;
        message_crc ^= data;
        payload_length_received++;

        if (payload_length_received >= payload_length_expected) {
          state++;
        }
        break;
      case 6:
        if (message_crc == data) {
          // CRC is ok, process data
          process_data();
        } else {
          // respond that CRC failed
          CRC_FAILED(code, message_crc);
        }

        // reset variables
        memset(data_buffer, 0, sizeof(data_buffer));

        payload_length_received = 0;
        state = 0;
        break;
      }
    }
  };

  void process_data() {
    switch (code) {
    case PSP_REQ_BIND_DATA:
      protocol_head(PSP_REQ_BIND_DATA, sizeof(bind_data));
      {
        char* array = (char*) &bind_data;
        for (uint16_t i = 0; i < sizeof(bind_data); i++) {
          serialize_uint8(array[i]);
        }
      }
      break;
    case PSP_REQ_RX_CONFIG:
      protocol_head(PSP_REQ_RX_CONFIG, sizeof(rx_config));
      {
        char* array = (char*) &rx_config;
        for (uint16_t i = 0; i < sizeof(rx_config); i++) {
          serialize_uint8(array[i]);
        }
      }
      break;
    case PSP_REQ_RX_JOIN_CONFIGURATION:
      protocol_head(PSP_REQ_RX_JOIN_CONFIGURATION, 1);
      // 1 success, 2 timeout, 3 failed response

      serialize_uint8(rxcConnect());

      break;
    case PSP_REQ_SCANNER_MODE:
      protocol_head(PSP_REQ_SCANNER_MODE, 1);
      serialize_uint8(0x01);
      protocol_tail();

      scannerMode();

      return;
      break;
    case PSP_REQ_SPECIAL_PINS:
      protocol_head(PSP_REQ_SPECIAL_PINS, sizeof(struct rxSpecialPinMap) * rxcSpecialPinCount);
      {
        char* array = (char*) &rxcSpecialPins;
        for (uint16_t i = 0; i < sizeof(struct rxSpecialPinMap) * rxcSpecialPinCount; i++) {
          serialize_uint8(array[i]);
        }
      }
      break;
    case PSP_REQ_FW_VERSION:
      protocol_head(PSP_REQ_FW_VERSION, sizeof(version));
      {
        serialize_uint16(version);
      }
      break;
    case PSP_REQ_NUMBER_OF_RX_OUTPUTS:
      protocol_head(PSP_REQ_NUMBER_OF_RX_OUTPUTS, 1);
      {
        serialize_uint8(rxcNumberOfOutputs);
      }
      break;
    case PSP_REQ_ACTIVE_PROFILE:
      protocol_head(PSP_REQ_ACTIVE_PROFILE, 1);
      {
        serialize_uint8(activeProfile);
      }
      break;
      // SET
    case PSP_SET_BIND_DATA:
      protocol_head(PSP_SET_BIND_DATA, 1);

      if (payload_length_received == sizeof(bind_data)) {
        char* array = (char*) &bind_data;

        for (uint16_t i = 0; i < sizeof(bind_data); i++) {
          array[i] = data_buffer[i];
        }

        serialize_uint8(0x01);
      } else {
        // fail (buffer size doesn't match struct memory size)
        serialize_uint8(0x00);
      }
      break;
    case PSP_SET_RX_CONFIG:
      protocol_head(PSP_SET_RX_CONFIG, 1);

      if (payload_length_received == sizeof(rx_config)) {
        char* array = (char*) &rx_config;

        for (uint16_t i = 0; i < sizeof(rx_config); i++) {
          array[i] = data_buffer[i];
        }

        serialize_uint8(0x01);
      } else {
        // fail (buffer size doesn't match struct memory size)
        serialize_uint8(0x00);
      }
      break;
    case PSP_SET_TX_SAVE_EEPROM:
      protocol_head(PSP_SET_TX_SAVE_EEPROM, 1);
      bindWriteEeprom();
      serialize_uint8(0x01); // success
      break;
    case PSP_SET_RX_SAVE_EEPROM:
      protocol_head(PSP_SET_RX_SAVE_EEPROM, 1);
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
          rx_packet_simple(tx_buf, 1);
          if (tx_buf[0]=='U') {
            serialize_uint8(0x01); // success
          } else {
            serialize_uint8(0x00); // fail
          }
        } else {
          serialize_uint8(0x00); // fail
        }
      }
      break;
    case PSP_SET_TX_RESTORE_DEFAULT:
      protocol_head(PSP_SET_TX_RESTORE_DEFAULT, 1);

      bindInitDefaults();

      serialize_uint8(0x01); // done
      break;
    case PSP_SET_RX_RESTORE_DEFAULT:
      protocol_head(PSP_SET_RX_RESTORE_DEFAULT, 1);
      // 1 success, 0 fail

      uint8_t tx_buf[1 + sizeof(rx_config)];
      tx_buf[0] = 'i';
      tx_packet(tx_buf,1);
      rx_reset();
      RF_Mode = Receive;
      delay(200);

      if (RF_Mode == Received) {
        rx_packet_simple(tx_buf, 1 + sizeof(rx_config));
        memcpy(&rx_config, tx_buf + 1, sizeof(rx_config));

        if (tx_buf[0] == 'I') {
          serialize_uint8(0x01); // success
        } else {
          serialize_uint8(0x00); // fail
        }
      } else {
        serialize_uint8(0x00); // fail
      }
      break;
    case PSP_SET_ACTIVE_PROFILE:
      protocol_head(PSP_SET_ACTIVE_PROFILE, 1);

      profileSwap(data_buffer[0]);
      if (!bindReadEeprom()) {
        bindInitDefaults();
        bindWriteEeprom();
      }
      serialize_uint8(0x01); // done
      break;
    case PSP_SET_EXIT:
      protocol_head(PSP_SET_EXIT, 1);
      serialize_uint8(0x01);
      protocol_tail();

      binary_mode_active = false;

      return;
      break;
    default: // Unrecognized code
      REFUSED();
    }

    // send over crc
    protocol_tail();
  };

  void protocol_head(uint8_t code, uint16_t length) {
    Serial.write(PSP_SYNC1);
    Serial.write(PSP_SYNC2);

    crc = 0; // reset crc

    serialize_uint8(code);
    serialize_uint16(length);
  };

  void protocol_tail() {
    Serial.write(crc);
  };

  void serialize_uint8(uint8_t data) {
    Serial.write(data);
    crc ^= data;
  };

  void serialize_uint16(uint16_t data) {
    serialize_uint8(lowByte(data));
    serialize_uint8(highByte(data));
  };

  void serialize_uint32(uint32_t data) {
    for (uint8_t i = 0; i < 4; i++) {
      serialize_uint8((uint8_t) (data >> (i * 8)));
    }
  };

  void serialize_uint64(uint64_t data) {
    for (uint8_t i = 0; i < 8; i++) {
      serialize_uint8((uint8_t) (data >> (i * 8)));
    }
  };

  void serialize_float32(float f) {
    uint8_t *b = (uint8_t*) & f;

    for (uint8_t i = 0; i < sizeof(f); i++) {
      serialize_uint8(b[i]);
    }
  };

  void ACK() {
    protocol_head(PSP_INF_ACK, 1);

    serialize_uint8(0x01);
  };

  void REFUSED() {
    protocol_head(PSP_INF_REFUSED, 1);

    serialize_uint8(0x00);
  };

  void CRC_FAILED(uint8_t code, uint8_t failed_crc) {
    protocol_head(PSP_INF_CRC_FAIL, 2);

    serialize_uint8(code);
    serialize_uint8(failed_crc);
  };

private:
  uint8_t data; // variable used to store a single byte from serial

  uint8_t state;
  uint8_t code;
  uint8_t message_crc;
  uint8_t crc;

  uint16_t payload_length_expected;
  uint16_t payload_length_received;

  uint8_t data_buffer[100];
} binary_com;
