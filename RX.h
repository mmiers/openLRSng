/****************************************************
 * OpenLRSng receiver code
 ****************************************************/

#include <avr/eeprom.h>

uint16_t rxerrors = 0;

uint8_t RF_channel = 0;

uint32_t lastPacketTimeUs = 0;
uint32_t lastRSSITimeUs = 0;
uint32_t linkLossTimeMs;


uint32_t lastBeaconTimeMs;

uint8_t  RSSI_count = 0;
uint16_t RSSI_sum = 0;
uint8_t  lastRSSIvalue = 0;
uint8_t  smoothRSSI = 0;
uint8_t  compositeRSSI = 0;
uint16_t lastAFCCvalue = 0;

uint16_t linkQuality = 0;

uint8_t  ppmCountter = 0;
uint16_t ppmSync = 40000;
uint8_t  ppmChannels = 8;

volatile uint8_t disablePWM = 0;
volatile uint8_t disablePPM = 0;
uint8_t failsafeActive = 0;

uint16_t failsafePPM[PPM_CHANNELS];
uint8_t  failsafeIsValid = 0;

uint8_t linkAcquired = 0;
uint8_t numberOfLostPackets = 0;

volatile uint8_t slaveState = 0; // 0 - no slave, 1 - slave initializing, 2 - slave running, 3- errored
uint32_t slaveFailedMs = 0;

boolean willhop = 0, fs_saved = 0;

pinMask_t chToMask[PPM_CHANNELS];
pinMask_t clearMask;

void outputUp(uint8_t no)
{
  PORTB |= chToMask[no].B;
  PORTC |= chToMask[no].C;
  PORTD |= chToMask[no].D;
}

void outputDownAll()
{
  PORTB &= clearMask.B;
  PORTC &= clearMask.C;
  PORTD &= clearMask.D;
}

volatile uint16_t nextICR1;

ISR(TIMER1_OVF_vect)
{
  if (ppmCountter < ppmChannels) {
    ICR1 = nextICR1;
    nextICR1 = servoBits2Us(PPM[ppmCountter]) * 2;
    ppmSync -= nextICR1;
    if (ppmSync < (rx_config.minsync * 2)) {
      ppmSync = rx_config.minsync * 2;
    }
    if ((disablePPM) || ((rx_config.flags & PPM_MAX_8CH) && (ppmCountter >= 8))) {
      OCR1A = 65535; //do not generate a pulse
    } else {
      OCR1A = nextICR1 - 600;
    }

    while (TCNT1 < 32);
    outputDownAll();
    if ((!disablePWM) && (ppmCountter > 0)) {
      outputUp(ppmCountter - 1);
    }

    ppmCountter++;
  } else {
    ICR1 = nextICR1;
    nextICR1 = ppmSync;
    if (disablePPM) {
      OCR1A = 65535; //do not generate a pulse
    } else {
      OCR1A = nextICR1 - 600;
    }
    ppmSync = 40000;

    while (TCNT1 < 32);
    outputDownAll();
    if (!disablePWM) {
      outputUp(ppmChannels - 1);
    }

    ppmCountter = 0 ;
  }
}

uint16_t RSSI2Bits(uint8_t rssi)
{
  uint16_t ret = (uint16_t)rssi << 2;
  if (ret < 12) {
    ret = 12;
  } else if (ret > 1012) {
    ret = 1012;
  }
  return ret;
}

void set_PPM_RSSI_output()
{
  if (rx_config.RSSIpwm < 16) {
    cli();
    if (bind_data.flags & REVERSE_PPM_RSSI)
      PPM[rx_config.RSSIpwm] = 1024 - RSSI2Bits(compositeRSSI);
    else
      PPM[rx_config.RSSIpwm] = RSSI2Bits(compositeRSSI);
    sei();
  }
}

void set_RSSI_output()
{
  uint8_t linkq = countSetBits(linkQuality & 0x7fff);
  if (linkq == 15) {
    // RSSI 0 - 255 mapped to 192 - ((255>>2)+192) == 192-255
    compositeRSSI = (smoothRSSI >> 2) + 192;
  } else {
    // linkquality gives 0 to 14*13 == 182
    compositeRSSI = linkq * 13;
  }

  set_PPM_RSSI_output();

  if (rx_config.pinMapping[RSSI_OUTPUT] == PINMAP_RSSI) {
    if ((compositeRSSI == 0) || (compositeRSSI == 255)) {
      TCCR2A &= ~(1 << COM2B1); // disable RSSI PWM output
      digitalWrite(OUTPUT_PIN[RSSI_OUTPUT], (compositeRSSI == 0) ? LOW : HIGH);
    } else {
      OCR2B = compositeRSSI;
      TCCR2A |= (1 << COM2B1); // enable RSSI PWM output
    }
  }
}

void failsafeSave(void)
{
  uint32_t start = millis();
  uint8_t ee_buf[20];

  for (int16_t i = 0; i < PPM_CHANNELS; i++) {
    failsafePPM[i] = PPM[i];
  }

  failsafeIsValid = 1;

  packChannels(6, failsafePPM, ee_buf);
  for (int16_t i = 0; i < 20; i++) {
    myEEPROMwrite(EEPROM_FAILSAFE_OFFSET + 4 + i, ee_buf[i]);
  }

  ee_buf[0] = 0xFA;
  ee_buf[1] = 0x11;
  ee_buf[2] = 0x5A;
  ee_buf[3] = 0xFE;
  for (int16_t i = 0; i < 4; i++) {
    myEEPROMwrite(EEPROM_FAILSAFE_OFFSET + i, ee_buf[i]);
  }

  // make this last at least 200ms for user to see it
  // needed as optimized eeprom code can be real fast if no changes are done
  start = millis() - start;
  if (start < 200) {
    delay(200 - start);
  }
}

void failsafeLoad(void)
{
  uint8_t ee_buf[20];

  for (int16_t i = 0; i < 4; i++) {
    ee_buf[i] = eeprom_read_byte((uint8_t *)(EEPROM_FAILSAFE_OFFSET + i));
  }

  if ((ee_buf[0] == 0xFA) && (ee_buf[1] == 0x11) && (ee_buf[2] == 0x5A) && (ee_buf[3] == 0xFE)) {
    for (int16_t i = 0; i < 20; i++) {
      ee_buf[i] = eeprom_read_byte((uint8_t *)(EEPROM_FAILSAFE_OFFSET + 4 + i));
    }
    unpackChannels(6, failsafePPM, ee_buf);
    failsafeIsValid = 1;
  } else {
    failsafeIsValid = 0;
  }
}

void failsafeApply()
{
  if (failsafeIsValid) {
    for (int16_t i = 0; i < PPM_CHANNELS; i++) {
      if (i != rx_config.RSSIpwm) {
        cli();
        PPM[i] = failsafePPM[i];
        sei();
      }
    }
  }
}

void setupOutputs()
{
  uint8_t i;

  ppmChannels = getChannelCount(&bind_data);
  if (rx_config.RSSIpwm == ppmChannels) {
    ppmChannels += 1;
  }

  for (i = 0; i < OUTPUTS; i++) {
    chToMask[i].B = 0;
    chToMask[i].C = 0;
    chToMask[i].D = 0;
  }
  clearMask.B = 0xff;
  clearMask.C = 0xff;
  clearMask.D = 0xff;
  for (i = 0; i < OUTPUTS; i++) {
    if (rx_config.pinMapping[i] < PPM_CHANNELS) {
      chToMask[rx_config.pinMapping[i]].B |= OUTPUT_MASKS[i].B;
      chToMask[rx_config.pinMapping[i]].C |= OUTPUT_MASKS[i].C;
      chToMask[rx_config.pinMapping[i]].D |= OUTPUT_MASKS[i].D;
      clearMask.B &= ~OUTPUT_MASKS[i].B;
      clearMask.C &= ~OUTPUT_MASKS[i].C;
      clearMask.D &= ~OUTPUT_MASKS[i].D;
    }
  }

  for (i = 0; i < OUTPUTS; i++) {
    switch (rx_config.pinMapping[i]) {
    case PINMAP_ANALOG:
      pinMode(OUTPUT_PIN[i], INPUT);
      break;
    case PINMAP_TXD:
    case PINMAP_RXD:
    case PINMAP_SDA:
    case PINMAP_SCL:
      break; //ignore serial/I2C for now
    default:
      if (i == RXD_OUTPUT) {
        UCSR0B &= 0xEF; //disable serial RXD
      }
      if (i == TXD_OUTPUT) {
        UCSR0B &= 0xF7; //disable serial TXD
      }
      pinMode(OUTPUT_PIN[i], OUTPUT); //PPM,PWM,RSSI,LBEEP
      break;
    }
  }

  if (rx_config.pinMapping[PPM_OUTPUT] == PINMAP_PPM) {
    digitalWrite(OUTPUT_PIN[PPM_OUTPUT], HIGH);
    TCCR1A = (1 << WGM11) | (1 << COM1A1);
  } else {
    TCCR1A = (1 << WGM11);
  }

  disablePWM = 1;
  disablePPM = 1;

  if ((rx_config.pinMapping[RSSI_OUTPUT] == PINMAP_RSSI) ||
      (rx_config.pinMapping[RSSI_OUTPUT] == PINMAP_LBEEP)) {
    pinMode(OUTPUT_PIN[RSSI_OUTPUT], OUTPUT);
    digitalWrite(OUTPUT_PIN[RSSI_OUTPUT], LOW);
    if (rx_config.pinMapping[RSSI_OUTPUT] == PINMAP_RSSI) {
      TCCR2B = (1 << CS20);
      TCCR2A = (1 << WGM20);
    } else { // LBEEP
      TCCR2A = (1 << WGM21); // mode=CTC
      TCCR2B = (1 << CS22) | (1 << CS20); // prescaler = 128
      OCR2A = 62; // 1KHz
    }
  }

  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  OCR1A = 65535;  // no pulse =)
  ICR1 = 2000; // just initial value, will be constantly updated
  ppmSync = 40000;
  nextICR1 = 40000;
  ppmCountter = 0;
  TIMSK1 |= (1 << TOIE1);

  if ((rx_config.flags & IMMEDIATE_OUTPUT) && failsafeIsValid) {
    failsafeApply();
    disablePPM=0;
    disablePWM=0;
  }
}

void updateLBeep(boolean packetLost)
{
  if (rx_config.pinMapping[LLIND_OUTPUT] == PINMAP_LLIND) {
    digitalWrite(LLIND_OUTPUT,packetLost);
  }
  if (rx_config.pinMapping[RSSI_OUTPUT] == PINMAP_LBEEP) {
    if (packetLost) {
      TCCR2A |= (1 << COM2B0); // enable tone
    } else {
      TCCR2A &= ~(1 << COM2B0); // disable tone
    }
  }
}

void updateSwitches()
{
  uint8_t i;
  for (i = 0; i < OUTPUTS; i++) {
    uint8_t map = rx_config.pinMapping[i];
    if ((map & 0x10) == 0x10) { // 16-31
      digitalWrite(i, (PPM[map & 0x0f] > 255) ? HIGH : LOW);
    }
  }
}


uint8_t bindReceive(uint32_t timeout)
{
  uint32_t start = millis();
  uint8_t  rxb;
  init_rfm(1);
  RF_Mode = Receive;
  to_rx_mode();
  Serial.println("Waiting bind\n");

  while ((!timeout) || ((millis() - start) < timeout)) {
    if (RF_Mode == Received) {
      Serial.println("Got pkt\n");
      rx_packet_simple((uint8_t *)&rxb, 1);
      if (rxb == 'b') {
        rx_packet_more((uint8_t *)&bind_data, sizeof(bind_data));

        if (bind_data.version == BINDING_VERSION) {
          Serial.println("data good\n");
          rxb = 'B';
          tx_packet(&rxb, 1); // ACK that we got bound
          Green_LED_ON; //signal we got bound on LED:s
          return 1;
        }
      } else if ((rxb == 'p') || (rxb == 'i')) {
        uint8_t rxc_buf[sizeof(rx_config) + 1];
        if (rxb == 'p') {
          rxc_buf[0] = 'P';
          timeout = 0;
        } else {
          rxInitDefaults(1);
          rxc_buf[0] = 'I';
        }
        memcpy(rxc_buf + 1, &rx_config, sizeof(rx_config));
        tx_packet(rxc_buf, sizeof(rx_config) + 1);
      } else if (rxb == 't') {
        uint8_t rxc_buf[sizeof(rxSpecialPins) + 5];
        timeout = 0;
        rxc_buf[0] = 'T';
        rxc_buf[1] = (version >> 8);
        rxc_buf[2] = (version & 0xff);
        rxc_buf[3] = OUTPUTS;
        rxc_buf[4] = sizeof(rxSpecialPins) / sizeof(rxSpecialPins[0]);
        memcpy(rxc_buf + 5, &rxSpecialPins, sizeof(rxSpecialPins));
        tx_packet(rxc_buf, sizeof(rxSpecialPins) + 5);
      } else if (rxb == 'u') {
        rx_packet_more((uint8_t *)&rx_config, sizeof(rx_config));
        rxWriteEeprom();
        rxb = 'U';
        tx_packet(&rxb, 1); // ACK that we updated settings
      }
      RF_Mode = Receive;
      rx_reset();

    }
  }
  return 0;
}

int8_t checkIfConnected(uint8_t pin1, uint8_t pin2)
{
  int8_t ret = 0;
  pinMode(pin1, OUTPUT);
  digitalWrite(pin1, 1);
  digitalWrite(pin2, 1);
  delayMicroseconds(10);

  if (digitalRead(pin2)) {
    digitalWrite(pin1, 0);
    delayMicroseconds(10);

    if (!digitalRead(pin2)) {
      ret = 1;
    }
  }

  pinMode(pin1, INPUT);
  digitalWrite(pin1, 0);
  digitalWrite(pin2, 0);
  return ret;
}

uint8_t rx_buf[21]; // RX buffer (uplink)
// First byte of RX buf is
// MSB..LSB [1bit uplink seqno.] [1bit downlink seqno] [6bits type)
// type 0x00 normal servo, 0x01 failsafe set
// type 0x38..0x3f uplinkked serial data

uint8_t tx_buf[64]; // TX buffer (downlink)(type plus 8 x data)
// First byte is meta
// MSB..LSB [1 bit uplink seq] [1bit downlink seqno] [6b telemtype]
// 0x00 link info [RSSI] [AFCC]*2 etc...
// type 0x38-0x3f downlink serial data 1-8 bytes

#define SERIAL_BUFSIZE 32
uint8_t serial_buffer[SERIAL_BUFSIZE];
uint8_t serial_head;
uint8_t serial_tail;

uint8_t hopcount;


uint8_t slaveAct = 0;
uint8_t slaveCnt = 0;

uint8_t slaveHandler(uint8_t *data, uint8_t flags)
{
  if (flags & MYI2C_SLAVE_ISTX) {
    if (flags & MYI2C_SLAVE_ISFIRST) {
      *data = slaveState;
      slaveCnt=0;
    } else {
      if (slaveCnt < getPacketSize(&bind_data)) {
        *data = rx_buf[slaveCnt++];
      } else {
        return 0;
      }
    }
  } else {
    if (flags & MYI2C_SLAVE_ISFIRST) {
      slaveAct = *data;
      slaveCnt = 0;
      if ((slaveAct & 0xe0) == 0x60) {
        if (slaveState >= 2) {
          RF_channel = (*data & 0x1f);
          slaveState=3; // to RX mode
        }
        return 0;
      } else if (slaveAct==0xfe) {
        // deinitialize
        slaveState=0;
        return 0;
      }
    } else {
      if (slaveAct==0xff) {
        // load bind_data
        if (slaveCnt<sizeof(bind_data)) {
          ((uint8_t *)(&bind_data))[slaveCnt++] = *data;
          if (slaveCnt == sizeof(bind_data)) {
            slaveState=1;
            return 0;
          }
        } else {
          return 0;
        }
      }
    }
  }
  return 1;
}

void slaveLoop()
{
  myI2C_slaveSetup(32, 0, 0, slaveHandler);
  slaveState=0;
  while(1) {
    if (slaveState == 1) {
      init_rfm(0);   // Configure the RFM22B's registers for normal operation
      slaveState = 2; // BIND applied
      Red_LED_OFF;
    } else if (slaveState == 3) {
      Green_LED_OFF;
      rfmSetChannel(RF_channel);
      RF_Mode = Receive;
      rx_reset();
      slaveState = 4; // in RX mode
    } else if (slaveState == 4) {
      if (RF_Mode == Received) {
        rx_packet(rx_buf, getPacketSize(&bind_data));
        slaveState = 5;
        Green_LED_ON;
      }
    }
  }
}

void reinitSlave()
{
  uint8_t ret, buf[sizeof(bind_data)+1];
  buf[0] = 0xff;
  memcpy(buf+1,&bind_data,sizeof(bind_data));
  ret = myI2C_writeTo(32, buf, sizeof(bind_data)+1, MYI2C_WAIT);
  if (ret==0) {
    ret = myI2C_readFrom(32, buf, 1, MYI2C_WAIT);
    if ((ret==0)) {
      slaveState = 2;
    } else {
      slaveState = 255;
    }
  } else {
    slaveState = 255;
  }
  if (slaveState==2) {
  } else {
    slaveFailedMs = millis();
  }
}

void setup()
{
  //LEDs
  pinMode(Green_LED, OUTPUT);
  pinMode(Red_LED, OUTPUT);

  setupSPI();

#ifdef SDN_pin
  pinMode(SDN_pin, OUTPUT);  //SDN
  digitalWrite(SDN_pin, 0);
#endif

  pinMode(0, INPUT);   // Serial Rx
  pinMode(1, OUTPUT);  // Serial Tx

  Serial.begin(115200);

  rxReadEeprom();
  failsafeLoad();
  Serial.print("OpenLRSng RX starting ");
  printVersion(version);
  Serial.print(" on HW ");
  Serial.println(BOARD_TYPE);

  setupRfmInterrupt();

  sei();
  Red_LED_ON;

  if (checkIfConnected(OUTPUT_PIN[2], OUTPUT_PIN[3])) { // ch1 - ch2 --> force scannerMode
    while (1) {
      Red_LED_OFF;
      Green_LED_OFF;
      scannerMode();
    }
  }

  if (checkIfConnected(OUTPUT_PIN[0], OUTPUT_PIN[1]) || (!bindReadEeprom())) {
    Serial.print("EEPROM data not valid or bind jumpper set, forcing bind\n");

    if (bindReceive(0)) {
      bindWriteEeprom();
      Serial.println("Saved bind data to EEPROM\n");
      Green_LED_ON;
    }
    setupOutputs();
  } else {
    setupOutputs();

    if ((rx_config.pinMapping[SDA_OUTPUT] != PINMAP_SDA) ||
        (rx_config.pinMapping[SCL_OUTPUT] != PINMAP_SCL)) {
      rx_config.flags &= ~SLAVE_MODE;
    }

    if ((rx_config.flags & ALWAYS_BIND) && (!(rx_config.flags & SLAVE_MODE))) {
      if (bindReceive(500)) {
        bindWriteEeprom();
        Serial.println("Saved bind data to EEPROM\n");
        setupOutputs(); // parameters may have changed
        Green_LED_ON;
      }
    }
  }

  if ((rx_config.pinMapping[SDA_OUTPUT] == PINMAP_SDA) &&
      (rx_config.pinMapping[SCL_OUTPUT] == PINMAP_SCL)) {
    myI2C_init(1);
    if (rx_config.flags & SLAVE_MODE) {
      Serial.println("I am slave");
      slaveLoop();
    } else {
      uint8_t ret,buf;
      delay(20);
      ret = myI2C_readFrom(32, &buf, 1, MYI2C_WAIT);
      if (ret==0) {
        slaveState = 1;
      }
    }
  }

  Serial.print("Entering normal mode");

  init_rfm(0);   // Configure the RFM22B's registers for normal operation
  RF_channel = 0;
  rfmSetChannel(RF_channel);

  // Count hopchannels as we need it later
  hopcount=0;
  while ((hopcount < MAXHOPS) && (bind_data.hopchannel[hopcount] != 0)) {
    hopcount++;
  }

  //################### RX SYNC AT STARTUP #################
  RF_Mode = Receive;
  to_rx_mode();

  if (slaveState) {
    reinitSlave();
  }

  if ((rx_config.pinMapping[TXD_OUTPUT] == PINMAP_SPKTRM) ||
      (rx_config.pinMapping[TXD_OUTPUT] == PINMAP_SUMD)) {
    Serial.begin(115200);
  } else if (rx_config.pinMapping[TXD_OUTPUT] == PINMAP_SBUS) {
    Serial.begin(100000);
    UCSR0C |= 1<<UPM01; // set even parity
  } else if ((bind_data.flags & TELEMETRY_MASK) == TELEMETRY_FRSKY) {
    Serial.begin(9600);
  } else {
    if (bind_data.serial_baudrate < 10) {
      Serial.begin(9600);
    } else {
      Serial.begin(bind_data.serial_baudrate);
    }
  }

  while (Serial.available()) {
    Serial.read();
  }

  serial_head = 0;
  serial_tail = 0;
  linkAcquired = 0;
  lastPacketTimeUs = micros();

}

void checkSerial()
{
  while (Serial.available() && (((serial_tail + 1) % SERIAL_BUFSIZE) != serial_head)) {
    serial_buffer[serial_tail] = Serial.read();
    serial_tail = (serial_tail + 1) % SERIAL_BUFSIZE;
  }
}

void slaveHop()
{
  if (slaveState == 2) {
    uint8_t buf;
    buf = 0x60 + RF_channel;
    if (myI2C_writeTo(32, &buf, 1, MYI2C_WAIT)) {
      slaveState = 255;
      slaveFailedMs = millis();
    }
  }
}

// Return slave state or 255 in case of error
uint8_t readSlaveState()
{
  uint8_t ret = 255, buf;
  if (slaveState == 2) {
    ret = myI2C_readFrom(32, &buf, 1, MYI2C_WAIT);
    if (ret) {
      slaveState = 255;
      slaveFailedMs = millis();
      ret=255;
    } else {
      ret=buf;
    }
  }
  return ret;
}

//#define SLAVE_STATISTICS
#ifdef SLAVE_STATISTICS
uint16_t rxBoth   = 0;
uint16_t rxSlave  = 0;
uint16_t rxMaster = 0;
uint32_t rxStatsMs = 0;
#endif

//############ MAIN LOOP ##############
void loop()
{
  uint32_t timeUs, timeMs;

  if (spiReadRegister(RFM2X_REG_GPIO1_CFG) == 0) {     // detect the locked module and reboot
    Serial.println("RX hang");
    init_rfm(0);
    to_rx_mode();
  }

  checkSerial();

  timeUs = micros();

  uint8_t slaveReceived = 0;
  if (5 == readSlaveState()) {
    slaveReceived = 1;
  }
retry:
  if ((RF_Mode == Received) || (slaveReceived)) {
    uint32_t timeTemp = micros();

    if (RF_Mode == Received) {
      rx_packet(rx_buf, getPacketSize(&bind_data));

      lastAFCCvalue = rfmGetAFCC();
    } else {
      uint8_t ret, slave_buf[22];
      ret = myI2C_readFrom(32, slave_buf, getPacketSize(&bind_data) + 1, MYI2C_WAIT);
      if (ret) {
        slaveState = 255;
        slaveFailedMs = millis();
        goto retry; //slave failed when reading packet...
      } else {
        memcpy(rx_buf, slave_buf + 1, getPacketSize(&bind_data));
      }
    }

    lastPacketTimeUs = timeTemp; // used saved timestamp to avoid skew by I2C
    numberOfLostPackets = 0;
    linkQuality <<= 1;
    linkQuality |= 1;

    Red_LED_OFF;
    Green_LED_ON;

    updateLBeep(false);

#ifdef SLAVE_STATISTICS
    if (5 == readSlaveState()) {
      if (RF_Mode == Received) {
        rxBoth++;
      } else {
        rxSlave++;
      }
    } else {
      rxMaster++;
    }
#endif

    if ((rx_buf[0] & 0x3e) == 0x00) {
      cli();
      unpackChannels(bind_data.flags & 7, PPM, rx_buf + 1);
      set_PPM_RSSI_output(); // Override PPM from TX with RSSI value.
      sei();
      if (rx_buf[0] & 0x01) {
        if (!fs_saved) {
          failsafeSave();
          fs_saved = 1;
        }
      } else if (fs_saved) {
        fs_saved = 0;
      }
    } else {
      // something else than servo data...
      if ((rx_buf[0] & 0x38) == 0x38) {
        if ((rx_buf[0] ^ tx_buf[0]) & 0x80) {
          // We got new data... (not retransmission)
          uint8_t i;
          tx_buf[0] ^= 0x80; // signal that we got it
          if (rx_config.pinMapping[TXD_OUTPUT] == PINMAP_TXD) {
            for (i = 0; i <= (rx_buf[0] & 7);) {
              i++;
              const uint8_t ch = rx_buf[i];
              Serial.write(ch);
              if (bind_data.flags & MAVLINK_FRAMING) {
                // Check mavlink frames of incoming serial stream before injection of mavlink radio status packet.
                // Inject packet right after a completed packet
                if (MAVLink_detectFrame(ch) && timeUs - last_mavlinkInject_time > MAVLINK_INJECT_INTERVAL) {
                  // Inject Mavlink radio modem status package.
                  MAVLink_report(0, compositeRSSI, rxerrors); // uint8_t RSSI_remote, uint16_t RSSI_local, uint16_t rxerrors)
                  last_mavlinkInject_time = timeUs;
                }
              }
            }
          }
        }
      }
    }

    if (linkAcquired == 0) {
      linkAcquired = 1;
    }
    failsafeActive = 0;
    disablePWM = 0;
    disablePPM = 0;

    if (bind_data.flags & TELEMETRY_MASK) {
      if ((tx_buf[0] ^ rx_buf[0]) & 0x40) {
        // resend last message
      } else {
        tx_buf[0] &= 0xc0;
        tx_buf[0] ^= 0x40; // swap sequence as we have new data
        if (!(bind_data.flags & MAVLINK_FRAMING)) {
          if (serial_head != serial_tail) {
            uint8_t bytes = 0;
            while ((bytes < 8) && serial_head != serial_tail) {
              bytes++;
              tx_buf[bytes] = serial_buffer[serial_head];
              serial_head = (serial_head + 1) % SERIAL_BUFSIZE;
            }
            tx_buf[0] |= (0x37 + bytes);
          } else {
            // tx_buf[0] lowest 6 bits left at 0
            tx_buf[1] = lastRSSIvalue;

            if (rx_config.pinMapping[ANALOG0_OUTPUT] == PINMAP_ANALOG) {
              tx_buf[2] = analogRead(OUTPUT_PIN[ANALOG0_OUTPUT]) >> 2;
#ifdef ANALOG0_OUTPUT_ALT
            } else if (rx_config.pinMapping[ANALOG0_OUTPUT_ALT] == PINMAP_ANALOG) {
              tx_buf[2] = analogRead(OUTPUT_PIN[ANALOG0_OUTPUT_ALT]) >> 2;
#endif
            } else {
              tx_buf[2] = 0;
            }

            if (rx_config.pinMapping[ANALOG1_OUTPUT] == PINMAP_ANALOG) {
              tx_buf[3] = analogRead(OUTPUT_PIN[ANALOG1_OUTPUT]) >> 2;
#ifdef ANALOG1_OUTPUT_ALT
            } else if (rx_config.pinMapping[ANALOG1_OUTPUT_ALT] == PINMAP_ANALOG) {
              tx_buf[3] = analogRead(OUTPUT_PIN[ANALOG1_OUTPUT_ALT]) >> 2;
#endif
            } else {
              tx_buf[3] = 0;
            }
            tx_buf[4] = (lastAFCCvalue >> 8);
            tx_buf[5] = lastAFCCvalue & 0xff;
            tx_buf[6] = countSetBits(linkQuality & 0x7fff);
          }
        } else { // MAVlink data
          uint8_t bytes = 0;
          while ((bytes < bind_data.serial_downlink - 1) && serial_head != serial_tail) {
            bytes++;
            tx_buf[bytes] = serial_buffer[serial_head];
            serial_head = (serial_head + 1) % SERIAL_BUFSIZE;
          }
          tx_buf[0] |= (0x3F & bytes);
        }
      }
#ifdef TEST_NO_ACK_BY_CH0
      if (PPM[0]<900) {
        tx_packet_async(tx_buf, 9);
        while(!tx_done()) {
          checkSerial();
        }
      }
#else
      tx_packet_async(tx_buf, 9);
      while(!tx_done()) {
        checkSerial();
      }
#endif
      updateSwitches();
    }

    RF_Mode = Receive;
    rx_reset();

    willhop = 1;

    Green_LED_OFF;
  }

  timeUs = micros();
  timeMs = millis();

  // sample RSSI when packet is in the 'air'
  if ((numberOfLostPackets < 2) && (lastRSSITimeUs != lastPacketTimeUs) &&
      (timeUs - lastPacketTimeUs) > (getInterval(&bind_data) - 1500)) {
    lastRSSITimeUs = lastPacketTimeUs;
    lastRSSIvalue = rfmGetRSSI(); // Read the RSSI value
    RSSI_sum += lastRSSIvalue;    // tally up for average
    RSSI_count++;

    if (RSSI_count > 8) {
      RSSI_sum /= RSSI_count;
      smoothRSSI = (((uint16_t)smoothRSSI * 3 + (uint16_t)RSSI_sum * 1) / 4);
      set_RSSI_output();
      RSSI_sum = 0;
      RSSI_count = 0;
    }
  }

  if (linkAcquired) {
    if ((numberOfLostPackets < hopcount) && ((timeUs - lastPacketTimeUs) > (getInterval(&bind_data) + 1000))) {
      // we lost packet, hop to next channel
      linkQuality <<= 1;
      willhop = 1;
      if (numberOfLostPackets == 0) {
        linkLossTimeMs = timeMs;
        lastBeaconTimeMs = 0;
        rxerrors++;
      }
      numberOfLostPackets++;
      lastPacketTimeUs += getInterval(&bind_data);
      willhop = 1;
      Red_LED_ON;
      updateLBeep(true);
      set_RSSI_output();
    } else if ((numberOfLostPackets == hopcount) && ((timeUs - lastPacketTimeUs) > (getInterval(&bind_data) * hopcount))) {
      // hop slowly to allow resync with TX
      linkQuality = 0;
      willhop = 1;
      set_RSSI_output();
      lastPacketTimeUs = timeUs;
    }

    if (numberOfLostPackets) {
      if (rx_config.failsafeDelay && (!failsafeActive) && ((timeMs - linkLossTimeMs) > delayInMs(rx_config.failsafeDelay))) {
        failsafeActive = 1;
        failsafeApply();
        lastBeaconTimeMs = (timeMs + delayInMsLong(rx_config.beacon_deadtime)) | 1; //beacon activating...
      }
      if (rx_config.pwmStopDelay && (!disablePWM) && ((timeMs - linkLossTimeMs) > delayInMs(rx_config.pwmStopDelay))) {
        disablePWM = 1;
      }
      if (rx_config.ppmStopDelay && (!disablePPM) && ((timeMs - linkLossTimeMs) > delayInMs(rx_config.ppmStopDelay))) {
        disablePPM = 1;
      }

      if ((rx_config.beacon_frequency) && (lastBeaconTimeMs)) {
        if (((timeMs - lastBeaconTimeMs) < 0x80000000) && // last beacon is future during deadtime
            (timeMs - lastBeaconTimeMs) > (1000UL * rx_config.beacon_interval)) {
          beacon_send();
          init_rfm(0);   // go back to normal RX
          rx_reset();
          lastBeaconTimeMs = millis() | 1; // avoid 0 in time
        }
      }
    }
  } else {
    // Waiting for first packet, hop slowly
    if ((timeUs - lastPacketTimeUs) > (getInterval(&bind_data) * hopcount)) {
      lastPacketTimeUs = timeUs;
      willhop = 1;
    }
  }

  if (!disablePPM) {
    if (rx_config.pinMapping[TXD_OUTPUT] == PINMAP_SPKTRM) {
      sendSpektrumFrame();
    } else if (rx_config.pinMapping[TXD_OUTPUT] == PINMAP_SBUS) {
      sendSBUSFrame(failsafeActive, numberOfLostPackets);
    } else if (rx_config.pinMapping[TXD_OUTPUT] == PINMAP_SUMD) {
      sendSUMDFrame(failsafeActive);
    }
  }

  if (willhop == 1) {
    RF_channel++;

    if ((RF_channel == MAXHOPS) || (bind_data.hopchannel[RF_channel] == 0)) {
      RF_channel = 0;
    }
    rfmSetChannel(RF_channel);
    slaveHop();
    willhop = 0;
  }

  if ((slaveState == 255) && ((millis() - slaveFailedMs) > 1000)) {
    slaveFailedMs=millis();
    reinitSlave();
  }

#ifdef SLAVE_STATISTICS
  if ((millis() - rxStatsMs) > 5000) {
    rxStatsMs = millis();
    Serial.print(rxBoth);
    Serial.print(',');
    Serial.print(rxMaster);
    Serial.print(',');
    Serial.println(rxSlave);
    rxBoth = rxMaster = rxSlave = 0;
  }
#endif
}
