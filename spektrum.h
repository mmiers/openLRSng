#define SPKTRM_SYNC1 0x03
#define SPKTRM_SYNC2 0x01

uint32_t spektrumLast = 0;
uint8_t  spektrumSendHi = 0;

void sendSpektrumFrame()
{
  uint32_t now = micros();
  if ((now - spektrumLast) > 10000) {
    uint8_t  ch = ((spektrumSendHi++) & 1) ? 7 : 0;
    spektrumLast = now;
    SerialWrite(Serial, SPKTRM_SYNC1);
    SerialWrite(Serial, SPKTRM_SYNC2);
    for (uint8_t ch = 0; ch < 7; ch++) {
      SerialWrite(Serial, (ch << 2) | ((PPM[ch] >> 8) & 0x03));
      SerialWrite(Serial, PPM[ch] & 0xff);
    }
  }
}

