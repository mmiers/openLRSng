//####### COMMON FUNCTIONS #########

void rfmSetCarrierFrequency(uint32_t f);
uint8_t rfmGetRSSI(void);
void RF22B_init_parameter(void);
uint8_t spiReadRegister(uint8_t address);
void spiWriteRegister(uint8_t address, uint8_t data);
void tx_packet(uint8_t* pkt, uint8_t size);
void to_rx_mode(void);

#define PPM_CHANNELS 16
volatile uint16_t PPM[PPM_CHANNELS] = { 512, 512, 512, 512, 512, 512, 512, 512 , 512, 512, 512, 512, 512, 512, 512, 512 };

const static uint8_t pktsizes[8] = { 0, 7, 11, 12, 16, 17, 21, 0 };


uint8_t getPacketSize(struct bind_data *bd)
{
  return pktsizes[(bd->flags & 0x07)];
}

uint8_t getChannelCount(struct bind_data *bd)
{
  return (((bd->flags & 7) / 2) + 1 + (bd->flags & 1)) * 4;
}

uint32_t getInterval(struct bind_data *bd)
{
  uint32_t ret;
  // Sending a x byte packet on bps y takes about (emperical)
  // usec = (x + 15) * 8200000 / baudrate
#define BYTES_AT_BAUD_TO_USEC(bytes, bps) ((uint32_t)((bytes) + 15) * 8200000L / (uint32_t)(bps))

  ret = (BYTES_AT_BAUD_TO_USEC(getPacketSize(bd), modem_params[bd->modem_params].bps) + 2000);

  if (bd->flags & TELEMETRY_MASK) {
    ret += (BYTES_AT_BAUD_TO_USEC(TELEMETRY_PACKETSIZE, modem_params[bd->modem_params].bps) + 1000);
  }

  // round up to ms
  ret = ((ret + 999) / 1000) * 1000;

  // enable following to limit packet rate to 50Hz at most
#ifdef LIMIT_RATE_TO_50HZ
  if (ret < 20000) {
    ret = 20000;
  }
#endif

  return ret;
}
uint8_t twoBitfy(uint16_t in)
{
  if (in < 256) {
    return 0;
  } else if (in < 512) {
    return 1;
  } else if (in < 768) {
    return 2;
  } else {
    return 3;
  }
}

void packChannels(uint8_t config, volatile uint16_t PPM[], uint8_t *p)
{
  uint8_t i;
  for (i = 0; i <= (config / 2); i++) { // 4ch packed in 5 bytes
    p[0] = (PPM[0] & 0xff);
    p[1] = (PPM[1] & 0xff);
    p[2] = (PPM[2] & 0xff);
    p[3] = (PPM[3] & 0xff);
    p[4] = ((PPM[0] >> 8) & 3) | (((PPM[1] >> 8) & 3) << 2) | (((PPM[2] >> 8) & 3) << 4) | (((PPM[3] >> 8) & 3) << 6);
    p += 5;
    PPM += 4;
  }
  if (config & 1) { // 4ch packed in 1 byte;
    p[0] = (twoBitfy(PPM[0]) << 6) | (twoBitfy(PPM[1]) << 4) | (twoBitfy(PPM[2]) << 2) | twoBitfy(PPM[3]);
  }
}

void unpackChannels(uint8_t config, volatile uint16_t PPM[], uint8_t *p)
{
  uint8_t i;
  for (i=0; i<=(config/2); i++) { // 4ch packed in 5 bytes
    PPM[0] = (((uint16_t)p[4] & 0x03) << 8) + p[0];
    PPM[1] = (((uint16_t)p[4] & 0x0c) << 6) + p[1];
    PPM[2] = (((uint16_t)p[4] & 0x30) << 4) + p[2];
    PPM[3] = (((uint16_t)p[4] & 0xc0) << 2) + p[3];
    p+=5;
    PPM+=4;
  }
  if (config & 1) { // 4ch packed in 1 byte;
    PPM[0] = (((uint16_t)p[0] >> 6) & 3) * 333 + 12;
    PPM[1] = (((uint16_t)p[0] >> 4) & 3) * 333 + 12;
    PPM[2] = (((uint16_t)p[0] >> 2) & 3) * 333 + 12;
    PPM[3] = (((uint16_t)p[0] >> 0) & 3) * 333 + 12;
  }
}

// conversion between microseconds 800-2200 and value 0-1023
// 808-1000 == 0 - 11     (16us per step)
// 1000-1999 == 12 - 1011 ( 1us per step)
// 2000-2192 == 1012-1023 (16us per step)

uint16_t servoUs2Bits(uint16_t x)
{
  uint16_t ret;

  if (x < 800) {
    ret = 0;
  } else if (x < 1000) {
    ret = (x - 799) / 16;
  } else if (x < 2000) {
    ret = (x - 988);
  } else if (x < 2200) {
    ret = (x - 1992) / 16 + 1011;
  } else {
    ret = 1023;
  }

  return ret;
}

uint16_t servoBits2Us(uint16_t x)
{
  uint16_t ret;

  if (x < 12) {
    ret = 808 + x * 16;
  } else if (x < 1012) {
    ret = x + 988;
  } else if (x < 1024) {
    ret = 2000 + (x - 1011) * 16;
  } else {
    ret = 2192;
  }

  return ret;
}

uint8_t countSetBits(uint16_t x)
{
  x  = x - ((x >> 1) & 0x5555);
  x  = (x & 0x3333) + ((x >> 2) & 0x3333);
  x  = x + (x >> 4);
  x &= 0x0F0F;
  return (x * 0x0101) >> 8;
}

static uint16_t CRC16_value;

inline void CRC16_reset()
{
  CRC16_value = 0;
}

void CRC16_add(uint8_t c) // CCITT polynome
{
  uint8_t i;
  CRC16_value ^= (uint16_t)c << 8;
  for (i = 0; i < 8; i++) {
    if (CRC16_value & 0x8000) {
      CRC16_value = (CRC16_value << 1) ^ 0x1021;
    } else {
      CRC16_value = (CRC16_value << 1);
    }
  }
}

// Halt and blink failure code
void fatalBlink(uint8_t blinks)
{
  while (1) {
    for (uint8_t i=0; i < blinks; i++) {
      Red_LED_ON;
      delay(100);
      Red_LED_OFF;
      delay(100);
    }
    delay(300);
  }
}

// Spectrum analyser 'submode'
void scannerMode(void)
{
  char c;
  uint32_t nextConfig[4] = { 0, 0, 0, 0 };
  uint32_t startFreq = MIN_RFM_FREQUENCY, endFreq = MAX_RFM_FREQUENCY, nrSamples = 500, stepSize = 50000;
  uint32_t currentFrequency = startFreq;
  uint32_t currentSamples = 0;
  uint8_t nextIndex = 0;
  uint8_t rssiMin = 0, rssiMax = 0;
  uint32_t rssiSum = 0;
  Serial.println("scanner mode");
  to_rx_mode();

  while (startFreq != 1000) { // if startFreq == 1000, break (used to exit scannerMode)
    while (Serial.available()) {
      c = Serial.read();

      switch (c) {
      case 'D':
        Serial.print('D');
        Serial.print(MIN_RFM_FREQUENCY);
        Serial.print(',');
        Serial.print(MAX_RFM_FREQUENCY);
        Serial.println(',');
        break;

      case 'S':
        currentFrequency = startFreq;
        currentSamples = 0;
        break;

      case '#':
        nextIndex = 0;
        nextConfig[0] = 0;
        nextConfig[1] = 0;
        nextConfig[2] = 0;
        nextConfig[3] = 0;
        break;

      case ',':
        nextIndex++;

        if (nextIndex == 4) {
          nextIndex = 0;
          startFreq = nextConfig[0] * 1000UL; // kHz -> Hz
          endFreq   = nextConfig[1] * 1000UL; // kHz -> Hz
          nrSamples = nextConfig[2]; // count
          stepSize  = nextConfig[3] * 1000UL;   // kHz -> Hz

          // set IF filtter BW (kha)
          if (stepSize < 20000) {
            spiWriteRegister(RFM2X_REG_IF_FILT_BW, RFM2X_MAKE_BITS(IFB_NDEC, 3) | RFM2X_MAKE_BITS(IFB_FILSET, 2));   // 10.6kHz
          } else if (stepSize < 30000) {
            spiWriteRegister(RFM2X_REG_IF_FILT_BW, RFM2X_MAKE_BITS(IFB_NDEC, 2) | RFM2X_MAKE_BITS(IFB_FILSET, 2));   // 21.0kHz
          } else if (stepSize < 40000) {
            spiWriteRegister(RFM2X_REG_IF_FILT_BW, RFM2X_MAKE_BITS(IFB_NDEC, 2) | RFM2X_MAKE_BITS(IFB_FILSET, 6));   // 32.2kHz
          } else if (stepSize < 50000) {
            spiWriteRegister(RFM2X_REG_IF_FILT_BW, RFM2X_MAKE_BITS(IFB_NDEC, 1) | RFM2X_MAKE_BITS(IFB_FILSET, 2));   // 41.7kHz
          } else if (stepSize < 60000) {
            spiWriteRegister(RFM2X_REG_IF_FILT_BW, RFM2X_MAKE_BITS(IFB_NDEC, 1) | RFM2X_MAKE_BITS(IFB_FILSET, 5));   // 56.2kHz
          } else if (stepSize < 70000) {
            spiWriteRegister(RFM2X_REG_IF_FILT_BW, RFM2X_MAKE_BITS(IFB_NDEC, 0) | RFM2X_MAKE_BITS(IFB_FILSET, 1));   // 75.2kHz
          } else if (stepSize < 100000) {
            spiWriteRegister(RFM2X_REG_IF_FILT_BW, RFM2X_MAKE_BITS(IFB_NDEC, 0) | RFM2X_MAKE_BITS(IFB_FILSET, 3));   // 90.0kHz
          } else {
            spiWriteRegister(RFM2X_REG_IF_FILT_BW, RFM2X_MAKE_BITS(IFB_NDEC, 0) | RFM2X_MAKE_BITS(IFB_FILSET, 5));   // 112.1kHz
          }
        }

        break;

      default:
        if ((c >= '0') && (c <= '9')) {
          c -= '0';
          nextConfig[nextIndex] = nextConfig[nextIndex] * 10 + c;
        }
      }
    }

    if (currentSamples == 0) {
      // retune base
      rfmSetCarrierFrequency(currentFrequency);
      rssiMax = 0;
      rssiMin = 255;
      rssiSum = 0;
      delay(1);
    }

    if (currentSamples < nrSamples) {
      uint8_t val = rfmGetRSSI();
      rssiSum += val;

      if (val > rssiMax) {
        rssiMax = val;
      }

      if (val < rssiMin) {
        rssiMin = val;
      }

      currentSamples++;
    } else {
      Serial.print(currentFrequency / 1000UL);
      Serial.print(',');
      Serial.print(rssiMax);
      Serial.print(',');
      Serial.print(rssiSum / currentSamples);
      Serial.print(',');
      Serial.print(rssiMin);
      Serial.println(',');
      currentFrequency += stepSize;

      if (currentFrequency > endFreq) {
        currentFrequency = startFreq;
      }

      currentSamples = 0;
    }
  }
}

#define NOP() __asm__ __volatile__("nop")

#define RF22B_PWRSTATE_POWERDOWN    0x00
#define RF22B_PWRSTATE_READY        0x01
#define RF22B_PACKET_SENT_INTERRUPT 0x04
#define RF22B_PWRSTATE_RX           0x05
#define RF22B_PWRSTATE_TX           0x09

#define RF22B_Rx_packet_received_interrupt   0x02

uint8_t ItStatus1, ItStatus2;

void spiWriteBit(uint8_t b);

void spiSendCommand(uint8_t command);
void spiSendAddress(uint8_t i);
uint8_t spiReadData(void);
void spiWriteData(uint8_t i);

void to_sleep_mode(void);
void rx_reset(void);

// **** SPI bit banging functions

void spiWriteBit(uint8_t b)
{
  if (b) {
    SCK_off;
    NOP();
    SDI_on;
    NOP();
    SCK_on;
    NOP();
  } else {
    SCK_off;
    NOP();
    SDI_off;
    NOP();
    SCK_on;
    NOP();
  }
}

uint8_t spiReadBit(void)
{
  uint8_t r = 0;
  SCK_on;
  NOP();

  if (SDO_1) {
    r = 1;
  }

  SCK_off;
  NOP();
  return r;
}

void spiSendCommand(uint8_t command)
{
  nSEL_on;
  SCK_off;
  nSEL_off;

  for (uint8_t n = 0; n < 8 ; n++) {
    spiWriteBit(command & 0x80);
    command = command << 1;
  }

  SCK_off;
}

void spiSendAddress(uint8_t i)
{
  spiSendCommand(i & 0x7f);
}

void spiWriteData(uint8_t i)
{
  for (uint8_t n = 0; n < 8; n++) {
    spiWriteBit(i & 0x80);
    i = i << 1;
  }

  SCK_off;
}

uint8_t spiReadData(void)
{
  uint8_t Result = 0;
  SCK_off;

  for (uint8_t i = 0; i < 8; i++) {   //read fifo data byte
    Result = (Result << 1) + spiReadBit();
  }

  return(Result);
}

uint8_t spiReadRegister(uint8_t address)
{
  uint8_t result;
  spiSendAddress(address);
  result = spiReadData();
  nSEL_on;
  return(result);
}

void spiWriteRegister(uint8_t address, uint8_t data)
{
  address |= 0x80; //
  spiSendCommand(address);
  spiWriteData(data);
  nSEL_on;
}

// **** RFM22 access functions

void rfmSetChannel(uint8_t ch)
{
  uint8_t magicLSB = (bind_data.rf_magic & 0xff) ^ ch;
  spiWriteRegister(RFM2X_REG_FREQ_HOP_CH_SEL,    bind_data.hopchannel[ch]);
  spiWriteRegister(RFM2X_REG_TRANSMIT_HDR_3 + 3, magicLSB);
  spiWriteRegister(RFM2X_REG_CHECK_HDR_3 + 3,    magicLSB);
}

uint8_t rfmGetRSSI(void)
{
  return spiReadRegister(RFM2X_REG_RSSI);
}

uint16_t rfmGetAFCC(void)
{
  return (((uint16_t)spiReadRegister(RFM2X_REG_AFC_CORR_READ) << 2) | ((uint16_t)spiReadRegister(RFM2X_REG_OOK_CTR_VAL_1) >> 6));
}

void setModemRegs(struct rfm22_modem_regs* r)
{

  spiWriteRegister(RFM2X_REG_IF_FILT_BW,         r->r_1c);
  spiWriteRegister(RFM2X_REG_AFC_LOOP_GEAR,      r->r_1d);
  spiWriteRegister(RFM2X_REG_AFC_TIME_CTRL,      r->r_1e);
  spiWriteRegister(RFM2X_REG_CLK_RECOV_OVER_RAT, r->r_20);
  spiWriteRegister(RFM2X_REG_CLK_RECOV_OFF_2,    r->r_21);
  spiWriteRegister(RFM2X_REG_CLK_RECOV_OFF_1,    r->r_22);
  spiWriteRegister(RFM2X_REG_CLK_RECOV_OFF_0,    r->r_23);
  spiWriteRegister(RFM2X_REG_CLK_RECOV_TLG_1,    r->r_24);
  spiWriteRegister(RFM2X_REG_CLK_RECOV_TLG_0,    r->r_25);
  spiWriteRegister(RFM2X_REG_AFC_LIMITER,        r->r_2a);
  spiWriteRegister(RFM2X_REG_TX_DATA_RATE_1,     r->r_6e);
  spiWriteRegister(RFM2X_REG_TX_DATA_RATE_2,     r->r_6f);
  spiWriteRegister(RFM2X_REG_MOD_MODE_CTRL_1,    r->r_70);
  spiWriteRegister(RFM2X_REG_MOD_MODE_CTRL_2,    r->r_71);
  spiWriteRegister(RFM2X_REG_FREQ_DEV,           r->r_72);
}

void rfmSetCarrierFrequency(uint32_t f)
{
  uint16_t fb, fc, hbsel;
  if (f < 480000000) {
    hbsel = 0;
    fb = f / 10000000 - 24;
    fc = (f - (fb + 24) * 10000000) * 4 / 625;
  } else {
    hbsel = 1;
    fb = f / 20000000 - 24;
    fc = (f - (fb + 24) * 20000000) * 2 / 625;
  }
  spiWriteRegister(RFM2X_REG_FREQ_BAND_SEL, RFM2X_FBS_SBSEL + (hbsel ? RFM2X_FBS_HBSEL : 0) + RFM2X_MAKE_BITS(FBS_FB, fb));
  spiWriteRegister(RFM2X_REG_NOM_CAR_FREQ_1, highByte(fc));
  spiWriteRegister(RFM2X_REG_NOM_CAR_FREQ_2, lowByte(fc));
}

void init_rfm(uint8_t isbind)
{
  ItStatus1 = spiReadRegister(RFM2X_REG_INT_STAT_1);                            // read status, clear interrupt
  ItStatus2 = spiReadRegister(RFM2X_REG_INT_STAT_2);
  spiWriteRegister(RFM2X_REG_INT_EN_2,       RFM2X_INT_NONE);                   // disable interrupts
  spiWriteRegister(RFM2X_REG_OP_FUNC_CTRL_1, RF22B_PWRSTATE_READY);             // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
  spiWriteRegister(RFM2X_REG_OSC_LOAD_CAP,   RFM2X_MAKE_BITS(OLC_XLC, 0x7f));   // c = 12.5p
  spiWriteRegister(RFM2X_REG_MIC_OUTPUT_CLK, RFM2X_MAKE_BITS(MOC_MCLK, 0x05));
#ifdef SWAP_GPIOS
  spiWriteRegister(RFM2X_REG_GPIO0_CFG,      RFM2X_MAKE_BITS(GPIO, 0x15));      // gpio0 RX State
  spiWriteRegister(RFM2X_REG_GPIO1_CFG,      RFM2X_MAKE_BITS(GPIO, 0x12));      // gpio1 TX State
#else
  spiWriteRegister(RFM2X_REG_GPIO0_CFG,      RFM2X_MAKE_BITS(GPIO, 0x12));      // gpio0 TX State
  spiWriteRegister(RFM2X_REG_GPIO1_CFG,      RFM2X_MAKE_BITS(GPIO, 0x15));      // gpio1 RX State
#endif
  spiWriteRegister(RFM2X_REG_GPIO2_CFG,      RFM2X_MAKE_BITS(GPIO_DRV, 3) |
                                             RFM2X_GPIO_PUP |
                                             RFM2X_MAKE_BITS(GPIO, 0x1d));      // gpio 2 micro-controller clk output
  spiWriteRegister(RFM2X_REG_IO_PORT_CFG,    RFM2X_IOPC_NONE);                  // gpio    0, 1,2 NO OTHER FUNCTION.

  if (isbind) {
    setModemRegs(&bind_params);
  } else {
    setModemRegs(&modem_params[bind_data.modem_params]);
  }

  // Packet settings
  spiWriteRegister(RFM2X_REG_DATA_ACC_CTRL,  RFM2X_DAC_ENPACRX |
                                             RFM2X_DAC_ENPACTX |
                                             RFM2X_DAC_ENCRC);                  // enable packet handler, msb first, enable crc,
  spiWriteRegister(RFM2X_REG_HEADER_CTRL_1,  RFM2X_MAKE_BITS(HC1_HDCH, 0x0f));  // no broadcast, check header bytes 3,2,1,0
  spiWriteRegister(RFM2X_REG_HEADER_CTRL_2,  RFM2X_MAKE_BITS(HC2_HDLEN, 4) |
                                             RFM2X_MAKE_BITS(HC2_SYNCLEN, 2));  // 4 byte header, 2 byte synch, variable pkt size
  spiWriteRegister(RFM2X_REG_PREAMBLE_LEN,   0x0a);    // 10 nibbles (40 bit preamble)
  spiWriteRegister(RFM2X_REG_PREAMBLE_DET_CTRL,
                                             RFM2X_MAKE_BITS(PDC_PREATH, 5) |
                                             RFM2X_MAKE_BITS(PDC_RSSI_OFF, 2)); // preath = 5 (20bits), rssioff = 2
  spiWriteRegister(RFM2X_REG_SYNC_WORD_3,    0x2d);    // synchronize word 3
  spiWriteRegister(RFM2X_REG_SYNC_WORD_2,    0xd4);    // synchronize word 2
  spiWriteRegister(RFM2X_REG_SYNC_WORD_1,    RFM2X_REGV_NONE);    // synch word 1 (not used)
  spiWriteRegister(RFM2X_REG_SYNC_WORD_0,    RFM2X_REGV_NONE);    // synch word 0 (not used)

  uint32_t magic = isbind ? BIND_MAGIC : bind_data.rf_magic;
  for (uint8_t i = 0; i < 4; i++) {
    spiWriteRegister(RFM2X_REG_TRANSMIT_HDR_3 + i, (magic >> 24) & 0xff);   // tx header
    spiWriteRegister(RFM2X_REG_CHECK_HDR_3 + i,    (magic >> 24) & 0xff);   // rx header
    magic = magic << 8; // advance to next byte
  }

  spiWriteRegister(RFM2X_REG_HDR_EN_3,       RFM2X_REGV_ALL);    // all the bit to be checked
  spiWriteRegister(RFM2X_REG_HDR_EN_2,       RFM2X_REGV_ALL);    // all the bit to be checked
  spiWriteRegister(RFM2X_REG_HDR_EN_1,       RFM2X_REGV_ALL);    // all the bit to be checked
  spiWriteRegister(RFM2X_REG_HDR_EN_0,       RFM2X_REGV_ALL);    // all the bit to be checked

  if (isbind) {
    spiWriteRegister(RFM2X_REG_TX_PWR,       RFM2X_MAKE_BITS(TP_TXPOW, BINDING_POWER));
  } else {
    spiWriteRegister(RFM2X_REG_TX_PWR,       bind_data.rf_power); // NOTE: CLAMPING THIS IS A CODE CHANGE
  }

  spiWriteRegister(RFM2X_REG_FREQ_HOP_CH_SEL, 0);

  spiWriteRegister(RFM2X_REG_FREQ_HOP_ST_SZ, bind_data.rf_channel_spacing);   // channel spacing

  spiWriteRegister(RFM2X_REG_FREQ_OFF_1,     0x00);
  spiWriteRegister(RFM2X_REG_FREQ_OFF_2,     RFM2X_MAKE_BITS(FO2_FO, 0x00));  // no offset

  rfmSetCarrierFrequency(isbind ? BINDING_FREQUENCY : bind_data.rf_frequency);

}

void to_rx_mode(void)
{
  ItStatus1 = spiReadRegister(RFM2X_REG_INT_STAT_1);
  ItStatus2 = spiReadRegister(RFM2X_REG_INT_STAT_2);
  spiWriteRegister(RFM2X_REG_OP_FUNC_CTRL_1, RF22B_PWRSTATE_READY);
  delay(10);
  rx_reset();
  NOP();
}

void rx_reset(void)
{
  spiWriteRegister(RFM2X_REG_OP_FUNC_CTRL_1, RF22B_PWRSTATE_READY);
  spiWriteRegister(RFM2X_REG_RX_FIFO_CTRL,   36);      // threshold for rx almost full, interrupt when 1 byte received
  spiWriteRegister(RFM2X_REG_OP_FUNC_CTRL_2, RFM2X_OFC_1_PLLON |
                                             RFM2X_OFC_1_XTON);    //clear fifo disable multi packet
  spiWriteRegister(RFM2X_REG_OP_FUNC_CTRL_2, RFM2X_REGV_NONE);     // clear fifo, disable multi packet
  spiWriteRegister(RFM2X_REG_OP_FUNC_CTRL_1, RF22B_PWRSTATE_RX);   // to rx mode
  spiWriteRegister(RFM2X_REG_INT_EN_1,       RF22B_Rx_packet_received_interrupt);
  ItStatus1 = spiReadRegister(RFM2X_REG_INT_STAT_1);   //read the Interrupt Status1 register
  ItStatus2 = spiReadRegister(RFM2X_REG_INT_STAT_2);
}

uint32_t tx_start = 0;

void tx_packet_async(uint8_t* pkt, uint8_t size)
{
  spiWriteRegister(RFM2X_REG_TRANSMIT_PKT_LEN, size);   // total tx size

  for (uint8_t i = 0; i < size; i++) {
    spiWriteRegister(RFM2X_REG_FIFO_ACCESS,    pkt[i]);
  }

  spiWriteRegister(RFM2X_REG_INT_EN_1,         RF22B_PACKET_SENT_INTERRUPT);
  ItStatus1 = spiReadRegister(RFM2X_REG_INT_STAT_1);      //read the Interrupt Status1 register
  ItStatus2 = spiReadRegister(RFM2X_REG_INT_STAT_2);
  tx_start = micros();
  spiWriteRegister(RFM2X_REG_OP_FUNC_CTRL_1,   RF22B_PWRSTATE_TX);    // to tx mode

  RF_Mode = Transmit;
}

void tx_packet(uint8_t* pkt, uint8_t size)
{
  tx_packet_async(pkt, size);
  while ((RF_Mode == Transmit) && ((micros() - tx_start) < 100000));
  if (RF_Mode == Transmit) {
    Serial.println("TX timeout!");
  }

#ifdef TX_TIMING
  Serial.print("TX took:");
  Serial.println(micros() - tx_start);
#endif
}

uint8_t tx_done()
{
  if (RF_Mode != Transmit) {
#ifdef TX_TIMING
    Serial.print("TX took:");
    Serial.println(micros() - tx_start);
#endif
    return 1; // success
  }
  if ((micros() - tx_start) > 100000) {
    return 2; // timeout
  }
  return 0;
}

uint8_t rx_length()
{
  return spiReadRegister(RFM2X_REG_RCVD_PKT_LEN);
}

uint8_t rx_packet_simple(uint8_t *pkt, uint8_t size)
{
  spiSendAddress(RFM2X_REG_FIFO_ACCESS); // Send the package read command
  for (int16_t i = 0; i < size; i++) {
    pkt[i] = spiReadData();
  }

  return size;
}

uint8_t rx_packet_more(uint8_t *pkt, uint8_t size)
{
  for (int16_t i = 0; i < size; i++) {
    pkt[i] = spiReadData();
  }

  return size;
}

// Returns the number of bytes remaining after the read
uint8_t rx_packet(uint8_t* pkt, uint8_t size)
{
  register uint8_t ready;

  // get packet len
  ready = rx_length();

  // If we were asked to read more than len, don't do it
  // however, we MAY have been asked to read fewer bytes.
  if (size > ready)
    size = ready;

  // read packet bytes
  return rx_packet_simple(pkt, size);
}

void beacon_tone(int16_t hz, int16_t len) //duration is now in half seconds.
{
  int16_t d = 500000 / hz; // better resolution

  if (d < 1) {
    d = 1;
  }

  int16_t cycles = (len * 500000 / d);

  for (int16_t i = 0; i < cycles; i++) {
    SDI_on;
    delayMicroseconds(d);
    SDI_off;
    delayMicroseconds(d);
  }
}

void beacon_send(void)
{
  Green_LED_ON
  ItStatus1 = spiReadRegister(RFM2X_REG_INT_STAT_1);                             // read status, clear interrupt
  ItStatus2 = spiReadRegister(RFM2X_REG_INT_STAT_2);

  spiWriteRegister(RFM2X_REG_INT_EN_2,        RFM2X_INT_NONE);                   // no wakeup up, lbd,
  spiWriteRegister(RFM2X_REG_OP_FUNC_CTRL_1,  RF22B_PWRSTATE_READY);             // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
  spiWriteRegister(RFM2X_REG_OSC_LOAD_CAP,    RFM2X_MAKE_BITS(OLC_XLC, 0x7f));   // (default) c = 12.5p
  spiWriteRegister(RFM2X_REG_MIC_OUTPUT_CLK,  RFM2X_MAKE_BITS(MOC_MCLK, 0x05));
  spiWriteRegister(RFM2X_REG_GPIO0_CFG,       RFM2X_MAKE_BITS(GPIO, 0x12));      // gpio0 TX State
  spiWriteRegister(RFM2X_REG_GPIO1_CFG,       RFM2X_MAKE_BITS(GPIO, 0x15));      // gpio1 RX State
  spiWriteRegister(RFM2X_REG_GPIO2_CFG,       RFM2X_MAKE_BITS(GPIO_DRV, 3) |
                                              RFM2X_GPIO_PUP |
                                              RFM2X_MAKE_BITS(GPIO, 0x1d));      // gpio 2 micro-controller clk output
  spiWriteRegister(RFM2X_REG_IO_PORT_CFG,     RFM2X_IOPC_NONE);                  // gpio    0, 1,2 NO OTHER FUNCTION.

  spiWriteRegister(RFM2X_REG_MOD_MODE_CTRL_1, RFM2X_MMC1_TXDLRTSCALE |
                                              RFM2X_MMC1_MANPPOL |
                                              RFM2X_MMC1_ENMANINV);              // disable manchest

  spiWriteRegister(RFM2X_REG_DATA_ACC_CTRL,   RFM2X_REGV_NONE);    //disable packet handling

  spiWriteRegister(RFM2X_REG_FREQ_HOP_CH_SEL, 0);    // start channel

  spiWriteRegister(RFM2X_REG_FREQ_HOP_ST_SZ,  0x05);   // 50khz step size (10khz x value) // no hopping

  spiWriteRegister(RFM2X_REG_MOD_MODE_CTRL_2, RFM2X_MAKE_BITS(MMC2_TRCLK, 0) |
                                              RFM2X_MAKE_BITS(MMC2_DTMOD, 1) |
                                              RFM2X_MAKE_BITS(MMC2_MODTYP, 2));  // trclk=[00] no clock, dtmod=[01] direct using SPI, fd8=0 eninv=0 modtyp=[10] FSK
  spiWriteRegister(RFM2X_REG_FREQ_DEV,        0x02);   // fd (frequency deviation) 2*625Hz == 1.25kHz

  spiWriteRegister(RFM2X_REG_FREQ_OFF_1,      RFM2X_REGV_NONE);
  spiWriteRegister(RFM2X_REG_FREQ_OFF_2,      RFM2X_REGV_NONE);    // no offset

  rfmSetCarrierFrequency(rx_config.beacon_frequency);

  spiWriteRegister(RFM2X_REG_TX_PWR,          RFM2X_MAKE_BITS(TP_TXPOW, 7));   // 7 set max power 100mW

  delay(10);
  spiWriteRegister(RFM2X_REG_OP_FUNC_CTRL_1,  RF22B_PWRSTATE_TX);    // to tx mode
  delay(10);

  //close encounters tune
  //  G, A, F, F(lower octave), C
  //octave 3:  392  440  349  175   261

  beacon_tone(392, 1);

  spiWriteRegister(RFM2X_REG_TX_PWR,          RFM2X_MAKE_BITS(TP_TXPOW, 5));   // 5 set mid power 25mW
  delay(10);
  beacon_tone(440,1);

  spiWriteRegister(RFM2X_REG_TX_PWR,          RFM2X_MAKE_BITS(TP_TXPOW, 4));   // 4 set mid power 13mW
  delay(10);
  beacon_tone(349, 1);

  spiWriteRegister(RFM2X_REG_TX_PWR,          RFM2X_MAKE_BITS(TP_TXPOW, 2));   // 2 set min power 3mW
  delay(10);
  beacon_tone(175,1);

  spiWriteRegister(RFM2X_REG_TX_PWR,          RFM2X_MAKE_BITS(TP_TXPOW, 0));   // 0 set min power 1.3mW
  delay(10);
  beacon_tone(261, 2);


  spiWriteRegister(RFM2X_REG_OP_FUNC_CTRL_1,  RF22B_PWRSTATE_READY);
  Green_LED_OFF
}

// Print version, either x.y or x.y.z (if z != 0)
void printVersion(uint16_t v)
{
  Serial.print(v >> 8);
  Serial.print('.');
  Serial.print((v >> 4) & 0x0f);
  if (version & 0x0f) {
    Serial.print('.');
    Serial.print(v & 0x0f);
  }
}


