// Generic definitions needed always

#define Available 0
#define Transmit 1
#define Transmitted 2
#define Receive 3
#define Received 4

volatile uint8_t RF_Mode = 0;

void RFM22B_Int()
{
  if (RF_Mode == Transmit) {
    RF_Mode = Transmitted;
  }

  if (RF_Mode == Receive) {
    RF_Mode = Received;
  }
}

typedef struct pinMask {
  uint8_t B,C,D;
} pinMask_t;


#define RX_FLYTRON8CH 0x01
#define RX_OLRSNG4CH  0x02
#define RX_OLRSNG12CH 0x03
#define RX_DTFUHF10CH 0x04

#define PINMAP_PPM    0x20
#define PINMAP_RSSI   0x21
#define PINMAP_SDA    0x22
#define PINMAP_SCL    0x23
#define PINMAP_RXD    0x24
#define PINMAP_TXD    0x25
#define PINMAP_ANALOG 0x26
#define PINMAP_LBEEP  0x27 // packetloss beeper
#define PINMAP_SPKTRM 0x28 // spektrum satellit output

// Following table is used by the dialog code to
// determine possible extra functions for each output.


struct rxSpecialPinMap {
  uint8_t output;
  uint8_t type;
};

#ifdef COMPILE_TX
// Needed by dialog code
static const char *specialStrs[] = { "PPM","RSSI","SDA","SCL","RXD","TXD","AIN","LBEEP",
                                     "SPKTRM", "", "", "", "", "", "", ""
                                   };
#define SPECIALSTR(x) (specialStrs[(x)&0x0f]) // note must be changed if not 16 strings
#endif

//####### Board Pinouts #########

#if (BOARD_TYPE == 0) // Flytron M1 TX
#if (__AVR_ATmega328P__ != 1) || (F_CPU != 16000000)
#error Wrong board selected, select Arduino Pro/Pro Mini 5V/16MHz w/ ATMega328
#endif

#ifndef COMPILE_TX
#error TX module cannot be used as RX
#endif

#define TelemetrySerial Serial

#define PPM_IN     A5
#define BUZZER_ACT 9
#define BTN        10
#define Red_LED    12
#define Green_LED  11

#define Red_LED_ON  PORTB |= _BV(4);
#define Red_LED_OFF  PORTB &= ~_BV(4);

#define Green_LED_ON   PORTB |= _BV(3);
#define Green_LED_OFF  PORTB &= ~_BV(3);

#define PPM_Pin_Interrupt_Setup  PCMSK1 = 0x20;PCICR|=(1<<PCIE1);
#define PPM_Signal_Interrupt PCINT1_vect
#define PPM_Signal_Edge_Check ((PINC & 0x20)==0x20)

void buzzerInit()
{
  pinMode(BUZZER_ACT, OUTPUT);
  digitalWrite(BUZZER_ACT, LOW);
}

void buzzerOn(uint16_t freq)
{
  if (freq) {
    digitalWrite(BUZZER_ACT,HIGH);
  } else {
    digitalWrite(BUZZER_ACT,LOW);
  }
}

#define buzzerOff(foo) buzzerOn(0)

//## RFM22B Pinouts for Public Edition (M1 or Rx v1)
#define  nIRQ_1 (PIND & 0x08)==0x08 //D3
#define  nIRQ_0 (PIND & 0x08)==0x00 //D3

#define  nSEL_on PORTD |= (1<<4) //D4
#define  nSEL_off PORTD &= 0xEF //D4

#define  SCK_on PORTD |= (1<<2) //D2
#define  SCK_off PORTD &= 0xFB //D2

#define  SDI_on PORTC |= (1<<1) //C1
#define  SDI_off PORTC &= 0xFD //C1

#define  SDO_1 (PINC & 0x01) == 0x01 //C0
#define  SDO_0 (PINC & 0x01) == 0x00 //C0

#define SDO_pin A0
#define SDI_pin A1
#define SCLK_pin 2
#define IRQ_pin 3
#define nSel_pin 4

void setupSPI()
{
  pinMode(SDO_pin, INPUT);   //SDO
  pinMode(SDI_pin, OUTPUT);   //SDI
  pinMode(SCLK_pin, OUTPUT);   //SCLK
  pinMode(IRQ_pin, INPUT);   //IRQ
  pinMode(nSel_pin, OUTPUT);   //nSEL
}

#define IRQ_interrupt 0
void setupRfmInterrupt()
{
  attachInterrupt(IRQ_interrupt, RFM22B_Int, FALLING);
}

#endif

#if (BOARD_TYPE == 1) // Flytron M1 RX
#if (__AVR_ATmega328P__ != 1) || (F_CPU != 16000000)
#error Wrong board selected, select Arduino Pro/Pro Mini 5V/16MHz w/ ATMega328
#endif

#ifndef COMPILE_TX
#error M1 RX not verified yet
#endif

#define TelemetrySerial Serial

#define PPM_IN     5
#define BUZZER_ACT 7
#define BTN        8

#define Red_LED A3
#define Green_LED A2

#define Red_LED_ON  PORTC &= ~_BV(2);PORTC |= _BV(3);
#define Red_LED_OFF  PORTC &= ~_BV(2);PORTC &= ~_BV(3);

#define Green_LED_ON  PORTC &= ~_BV(3);PORTC |= _BV(2);
#define Green_LED_OFF  PORTC &= ~_BV(3);PORTC &= ~_BV(2);

#define PPM_Pin_Interrupt_Setup  PCMSK2 = 0x20;PCICR|=(1<<PCIE2);
#define PPM_Signal_Interrupt PCINT2_vect
#define PPM_Signal_Edge_Check ((PIND & 0x20)==0x20)

void buzzerInit()
{
  pinMode(BUZZER_ACT, OUTPUT);
  digitalWrite(BUZZER_ACT, LOW);
}

void buzzerOn(uint16_t freq)
{
  if (freq) {
    digitalWrite(BUZZER_ACT,HIGH);
  } else {
    digitalWrite(BUZZER_ACT,LOW);
  }
}

#define buzzerOff(foo) buzzerOn(0)

//## RFM22B Pinouts for Public Edition (M1 or Rx v1)
#define  nIRQ_1 (PIND & 0x08)==0x08 //D3
#define  nIRQ_0 (PIND & 0x08)==0x00 //D3

#define  nSEL_on PORTD |= (1<<4) //D4
#define  nSEL_off PORTD &= 0xEF //D4

#define  SCK_on PORTD |= (1<<2) //D2
#define  SCK_off PORTD &= 0xFB //D2

#define  SDI_on PORTC |= (1<<1) //C1
#define  SDI_off PORTC &= 0xFD //C1

#define  SDO_1 (PINC & 0x01) == 0x01 //C0
#define  SDO_0 (PINC & 0x01) == 0x00 //C0

#define SDO_pin A0
#define SDI_pin A1
#define SCLK_pin 2
#define IRQ_pin 3
#define nSel_pin 4

void setupSPI()
{
  pinMode(SDO_pin, INPUT);   //SDO
  pinMode(SDI_pin, OUTPUT);   //SDI
  pinMode(SCLK_pin, OUTPUT);   //SCLK
  pinMode(IRQ_pin, INPUT);   //IRQ
  pinMode(nSel_pin, OUTPUT);   //nSEL
}

#define IRQ_interrupt 0
void setupRfmInterrupt()
{
  attachInterrupt(IRQ_interrupt, RFM22B_Int, FALLING);
}

#endif

#if (BOARD_TYPE == 2)
#if (__AVR_ATmega328P__ != 1) || (F_CPU != 16000000)
#error Wrong board selected, select Arduino Pro/Pro Mini 5V/16MHz w/ ATMega328
#endif

#ifndef COMPILE_TX
#error TX module cannot be used as RX
#endif

#define TelemetrySerial Serial

#define PPM_IN           3
#define RF_OUT_INDICATOR A0
#define BUZZER_ACT       10
#define BTN              11
#define Red_LED          13
#define Green_LED        12

#define Red_LED_ON  PORTB |= _BV(5);
#define Red_LED_OFF  PORTB &= ~_BV(5);

#define Green_LED_ON   PORTB |= _BV(4);
#define Green_LED_OFF  PORTB &= ~_BV(4);

#define PPM_Pin_Interrupt_Setup  PCMSK2 = 0x08;PCICR|=(1<<PCIE2);
#define PPM_Signal_Interrupt PCINT2_vect
#define PPM_Signal_Edge_Check ((PIND & 0x08)==0x08)

void buzzerInit()
{
  pinMode(BUZZER_ACT, OUTPUT);
  digitalWrite(BUZZER_ACT, LOW);
}

void buzzerOn(uint16_t freq)
{
  if (freq) {
    digitalWrite(BUZZER_ACT,HIGH);
  } else {
    digitalWrite(BUZZER_ACT,LOW);
  }
}

#define buzzerOff(foo) buzzerOn(0)

//## RFM22B Pinouts for Public Edition (M2)
#define  nIRQ_1 (PIND & 0x04)==0x04 //D2
#define  nIRQ_0 (PIND & 0x04)==0x00 //D2

#define  nSEL_on PORTD |= (1<<4) //D4
#define  nSEL_off PORTD &= 0xEF //D4

#define  SCK_on PORTD |= (1<<7) //D7
#define  SCK_off PORTD &= 0x7F //D7

#define  SDI_on PORTB |= (1<<0) //B0
#define  SDI_off PORTB &= 0xFE //B0

#define  SDO_1 (PINB & 0x02) == 0x02 //B1
#define  SDO_0 (PINB & 0x02) == 0x00 //B1

#define SDO_pin 9
#define SDI_pin 8
#define SCLK_pin 7
#define IRQ_pin 2
#define nSel_pin 4

void setupSPI()
{
  pinMode(SDO_pin, INPUT);   //SDO
  pinMode(SDI_pin, OUTPUT);   //SDI
  pinMode(SCLK_pin, OUTPUT);   //SCLK
  pinMode(IRQ_pin, INPUT);   //IRQ
  pinMode(nSel_pin, OUTPUT);   //nSEL
}

#define IRQ_interrupt 0
void setupRfmInterrupt()
{
  attachInterrupt(IRQ_interrupt, RFM22B_Int, FALLING);
}

#endif

#if (BOARD_TYPE == 3)
#if (__AVR_ATmega328P__ != 1) || (F_CPU != 16000000)
#error Wrong board selected, select Arduino Pro/Pro Mini 5V/16MHz w/ ATMega328
#endif

#define TelemetrySerial Serial

#ifdef COMPILE_TX

#define USE_ICP1 // use ICP1 for PPM input for less jitter

#define PPM_IN 8 // ICP1

#define BUZZER_ACT 6
#define BUZZER_PAS 3
#define BTN 7

void buzzerInit()
{
  pinMode(BUZZER_ACT, OUTPUT);
  digitalWrite(BUZZER_ACT, LOW);
  TCCR2A = (1<<WGM21); // mode=CTC
  TCCR2B = (1<<CS22) | (1<<CS20); // prescaler = 128
  pinMode(BUZZER_PAS, OUTPUT);
  digitalWrite(BUZZER_PAS, LOW);
}

void buzzerOn(uint16_t freq)
{
  if (freq) {
    digitalWrite(BUZZER_ACT,HIGH);
    uint32_t ocr = 125000L / freq;
    if (ocr>255) {
      ocr=255;
    }
    if (!ocr) {
      ocr=1;
    }
    OCR2A = ocr;
    TCCR2A |= (1<<COM2B0); // enable output on buzzer2
  } else {
    digitalWrite(BUZZER_ACT,LOW);
    TCCR2A &= ~(1<<COM2B0); // disable output buzzer2
  }
}

#define buzzerOff(foo) buzzerOn(0)
#else // RX
#define PPM_OUT 9 // OCP1A
#define RSSI_OUT 3 // PD3 OC2B

#define OUTPUTS 13 // outputs available

const pinMask_t OUTPUT_MASKS[OUTPUTS] = {
  {0x00,0x00,0x08},{0x00,0x00,0x20},{0x00,0x00,0x40}, // RSSI, CH1, CH2
  {0x00,0x00,0x80},{0x01,0x00,0x00},{0x02,0x00,0x00}, // CH2, CH3, CH4
  {0x04,0x00,0x00},{0x08,0x00,0x00},{0x10,0x00,0x00}, // CH5, CH6, CH7
  {0x00,0x10,0x00},{0x00,0x20,0x00},{0x00,0x00,0x01}, // SDA, SCL, RXD
  {0x00,0x00,0x02},                                   // TXD
};

const uint8_t OUTPUT_PIN[OUTPUTS] = { 3, 5, 6, 7, 8, 9, 10, 11, 12 , A4, A5, 0, 1};

#define PPM_OUTPUT  5
#define RSSI_OUTPUT 0
#define ANALOG0_OUTPUT 9
#define ANALOG1_OUTPUT 10
#define SDA_OUTPUT 9
#define SCL_OUTPUT 10
#define RXD_OUTPUT 11
#define TXD_OUTPUT 12

struct rxSpecialPinMap rxSpecialPins[] = {
  {  0, PINMAP_RSSI},
  {  0, PINMAP_LBEEP},
  {  5, PINMAP_PPM},
  {  9, PINMAP_SDA},
  {  9, PINMAP_ANALOG}, // AIN0
  { 10, PINMAP_SCL},
  { 10, PINMAP_ANALOG}, // AIN1
  { 11, PINMAP_RXD},
  { 12, PINMAP_TXD},
  { 12, PINMAP_SPKTRM},
};

#endif

#define Red_LED    A3
#define Green_LED  13

#ifndef COMPILE_TX
#define Red_LED_ON  PORTC |= _BV(3);
#define Red_LED_OFF  PORTC &= ~_BV(3);
#define Green_LED_ON  PORTB |= _BV(5);
#define Green_LED_OFF  PORTB &= ~_BV(5);
#else
#define Red_LED2   9
#define Green_LED2 10
#define Red_LED_ON  { PORTC |= _BV(3); PORTB |= _BV(1); }
#define Red_LED_OFF { PORTC &= ~_BV(3); PORTB &= ~_BV(1); }
#define Green_LED_ON  PORTB |= (_BV(5) | _BV(2));
#define Green_LED_OFF PORTB &= ~(_BV(5) | _BV(2));
#endif

//## RFM22B Pinouts for Public Edition (Rx v2)
#define  nIRQ_1 (PIND & 0x04)==0x04 //D2
#define  nIRQ_0 (PIND & 0x04)==0x00 //D2

#define  nSEL_on PORTD |= (1<<4) //D4
#define  nSEL_off PORTD &= 0xEF //D4

#define  SCK_on PORTC |= (1<<2) //A2
#define  SCK_off PORTC &= 0xFB //A2

#define  SDI_on PORTC |= (1<<1) //A1
#define  SDI_off PORTC &= 0xFD //A1

#define  SDO_1 (PINC & 0x01) == 0x01 //A0
#define  SDO_0 (PINC & 0x01) == 0x00 //A0

#define SDO_pin A0
#define SDI_pin A1
#define SCLK_pin A2
#define IRQ_pin 2
#define nSel_pin 4

void setupSPI()
{
  pinMode(SDO_pin, INPUT);   //SDO
  pinMode(SDI_pin, OUTPUT);   //SDI
  pinMode(SCLK_pin, OUTPUT);   //SCLK
  pinMode(IRQ_pin, INPUT);   //IRQ
  pinMode(nSel_pin, OUTPUT);   //nSEL
}

#define IRQ_interrupt 0
void setupRfmInterrupt()
{
  attachInterrupt(IRQ_interrupt, RFM22B_Int, FALLING);
}

#endif

#if (BOARD_TYPE == 4) // kha:s openLRSngTX & clones
#if (__AVR_ATmega328P__ != 1) || (F_CPU != 16000000)
#error Wrong board selected, select Arduino Pro/Pro Mini 5V/16MHz w/ ATMega328
#endif

#ifndef COMPILE_TX
#error TX module cannot be used as RX
#endif

#define TelemetrySerial Serial

#define USE_ICP1 // use ICP1 for PPM input for less jitter
#define PPM_IN 8 // ICP1

#define BUZZER_PAS 3 // OCR2B
#define BTN A0
#define Red_LED 6
#define Green_LED 5

#define RF_OUT_INDICATOR A3 // only used for Futaba

void buzzerInit()
{
  TCCR2A = (1<<WGM21); // mode=CTC
  TCCR2B = (1<<CS22) | (1<<CS20); // prescaler = 128
  pinMode(BUZZER_PAS, OUTPUT);
  digitalWrite(BUZZER_PAS, LOW);
}

void buzzerOn(uint16_t freq)
{
  if (freq) {
    uint32_t ocr = 125000L / freq;
    if (ocr>255) {
      ocr=255;
    }
    if (!ocr) {
      ocr=1;
    }
    OCR2A = ocr;
    TCCR2A |= (1<<COM2B0); // enable output
  } else {
    TCCR2A &= ~(1<<COM2B0); // disable output
  }
}

#define buzzerOff(foo) buzzerOn(0)

#define Red_LED_ON  PORTD |= _BV(6);
#define Red_LED_OFF  PORTD &= ~_BV(6);

#define Green_LED_ON   PORTD |= _BV(5);
#define Green_LED_OFF  PORTD &= ~_BV(5);

#define RF_OUT_INDICATOR A3 // only used for Futaba

//## RFM22B Pinouts for Public Edition (M2)
#define  nIRQ_1 (PIND & 0x04)==0x04 //D2
#define  nIRQ_0 (PIND & 0x04)==0x00 //D2

#define  nSEL_on PORTD |= (1<<4) //D4
#define  nSEL_off PORTD &= 0xEF //D4

#define  SCK_on  PORTB |= _BV(5)  //B5
#define  SCK_off PORTB &= ~_BV(5) //B5

#define  SDI_on  PORTB |= _BV(3)  //B3
#define  SDI_off PORTB &= ~_BV(3) //B3

#define  SDO_1 (PINB & _BV(4)) == _BV(4) //B4
#define  SDO_0 (PINB & _BV(4)) == 0x00  //B4

#define SDO_pin 12
#define SDI_pin 11
#define SCLK_pin 13
#define IRQ_pin 2
#define nSel_pin 4
#define SDN_pin 9

void setupSPI()
{
  pinMode(SDO_pin, INPUT);   //SDO
  pinMode(SDI_pin, OUTPUT);   //SDI
  pinMode(SCLK_pin, OUTPUT);   //SCLK
  pinMode(IRQ_pin, INPUT);   //IRQ
  pinMode(nSel_pin, OUTPUT);   //nSEL
}

#define IRQ_interrupt 0
void setupRfmInterrupt()
{
  attachInterrupt(IRQ_interrupt, RFM22B_Int, FALLING);
}

#define SWAP_GPIOS

#endif

#if (BOARD_TYPE == 5) // openLRSngRX-4ch
#if (__AVR_ATmega328P__ != 1) || (F_CPU != 16000000)
#error Wrong board selected, select Arduino Pro/Pro Mini 5V/16MHz w/ ATMega328
#endif

#define TelemetrySerial Serial

#ifdef COMPILE_TX
// TX operation

#define USE_ICP1 // use ICP1 for PPM input for less jitter
#define PPM_IN 8 // ICP1

#define BUZZER_PAS  3  // OCR2B
#define BUZZER_ACT A5
#define BTN     A4

void buzzerInit()
{
  pinMode(BUZZER_ACT, OUTPUT);
  digitalWrite(BUZZER_ACT, LOW);
  TCCR2A = (1<<WGM21); // mode=CTC
  TCCR2B = (1<<CS22) | (1<<CS20); // prescaler = 128
  pinMode(BUZZER_PAS, OUTPUT);
  digitalWrite(BUZZER_PAS, LOW);
}

void buzzerOn(uint16_t freq)
{
  if (freq) {
    uint32_t ocr = 125000L / freq;
    digitalWrite(BUZZER_ACT,HIGH);
    if (ocr>255) {
      ocr=255;
    }
    if (!ocr) {
      ocr=1;
    }
    OCR2A = ocr;
    TCCR2A |= (1<<COM2B0); // enable output
  } else {
    digitalWrite(BUZZER_ACT,LOW);
    TCCR2A &= ~(1<<COM2B0); // disable output
  }
}

#else
// RX operation
#define PPM_OUT 9 // OCP1A
#define RSSI_OUT 3 // PD3 OC2B

#define PWM_1 9 // PB1 - also PPM
#define PWM_2 A4 // PC4 - also SDA
#define PWM_3 3 // PD3 - also RSSI
#define PWM_4 A5 // PC5 - also SCL
#define PWM_5 A0 // PC0
#define PWM_6 A1 // PC1

#define OUTPUTS 8 // outputs available

const pinMask_t OUTPUT_MASKS[OUTPUTS] = {
  {0x02,0x00,0x00}, {0x00,0x10,0x00}, {0x00,0x00,0x08},// CH1/PPM, CH2/SDA, CH3/RSSI
  {0x00,0x20,0x00}, {0x00,0x01,0x00}, {0x00,0x02,0x00},// CH4/SCL, CH5/AIN, CH6/AIN,
  {0x00,0x00,0x01}, {0x00,0x00,0x02},                  // CH7/RXD, CH8/TXD - only on 6ch

};

#define PPM_OUTPUT 0
#define RSSI_OUTPUT 2
#define ANALOG0_OUTPUT 1 // actually input
#define ANALOG1_OUTPUT 3 // actually input
#define ANALOG0_OUTPUT_ALT 4 // actually input
#define ANALOG1_OUTPUT_ALT 5 // actually input
#define SDA_OUTPUT 1
#define SCL_OUTPUT 3
#define RXD_OUTPUT 6
#define TXD_OUTPUT 7

const uint8_t OUTPUT_PIN[OUTPUTS] = { 9, A4, 3, A5, A0, A1, 0, 1};

struct rxSpecialPinMap rxSpecialPins[] = {
  { 0, PINMAP_PPM},
  { 1, PINMAP_SDA},
  { 1, PINMAP_ANALOG}, // AIN0
  { 2, PINMAP_RSSI},
  { 2, PINMAP_LBEEP},
  { 3, PINMAP_SCL},
  { 3, PINMAP_ANALOG}, // AIN1
  { 4, PINMAP_ANALOG},
  { 5, PINMAP_ANALOG},
  { 6, PINMAP_RXD},
  { 7, PINMAP_TXD},
  { 7, PINMAP_SPKTRM},
};

#endif

#define Red_LED 6
#define Green_LED 5

#ifndef COMPILE_TX
#define Red_LED_ON    PORTD |=  _BV(6);
#define Red_LED_OFF   PORTD &= ~_BV(6);
#define Green_LED_ON  PORTD |=  _BV(5);
#define Green_LED_OFF PORTD &= ~_BV(5);
#else
#define Red_LED2   A0
#define Green_LED2 A1
#define Red_LED_ON    { PORTD |=  _BV(6); PORTC |=  _BV(0); }
#define Red_LED_OFF   { PORTD &= ~_BV(6); PORTC &= ~_BV(0); }
#define Green_LED_ON  { PORTD |=  _BV(5); PORTC |=  _BV(1); }
#define Green_LED_OFF { PORTD &= ~_BV(5); PORTC &= ~_BV(1); }
#endif

#define buzzerOff(foo) buzzerOn(0)

//## RFM22B Pinouts for Public Edition (M2)
#define  nIRQ_1 (PIND & 0x04)==0x04 //D2
#define  nIRQ_0 (PIND & 0x04)==0x00 //D2

#define  nSEL_on PORTD |= (1<<4) //D4
#define  nSEL_off PORTD &= 0xEF //D4

#define  SCK_on  PORTB |= _BV(5)  //B5
#define  SCK_off PORTB &= ~_BV(5) //B5

#define  SDI_on  PORTB |= _BV(3)  //B3
#define  SDI_off PORTB &= ~_BV(3) //B3

#define  SDO_1 (PINB & _BV(4)) == _BV(4) //B4
#define  SDO_0 (PINB & _BV(4)) == 0x00  //B4

#define SDO_pin 12
#define SDI_pin 11
#define SCLK_pin 13
#define IRQ_pin 2
#define nSel_pin 4

void setupSPI()
{
  pinMode(SDO_pin, INPUT);   //SDO
  pinMode(SDI_pin, OUTPUT);   //SDI
  pinMode(SCLK_pin, OUTPUT);   //SCLK
  pinMode(IRQ_pin, INPUT);   //IRQ
  pinMode(nSel_pin, OUTPUT);   //nSEL
}

#define IRQ_interrupt 0
void setupRfmInterrupt()
{
  attachInterrupt(IRQ_interrupt, RFM22B_Int, FALLING);
}

#endif

#if (BOARD_TYPE == 6) // DTF UHF DeluxeTX
#if (__AVR_ATmega32U4__ != 1)
#error Wrong board selected, select Arduino Leonardo
#endif

#ifndef COMPILE_TX
#error TX module cannot be used as RX
#endif

#define TelemetrySerial Serial1

#define USE_ICP1 // use ICP1 for PPM input for less jitter
#define PPM_IN 4 // ICP1

#define BUZZER_PAS 10 // OCR4B
#define BTN A0
#define Red_LED 6 //PD7
#define Green_LED 5 //PC6

void buzzerInit()
{
  TCCR4B = (1<<CS43); // prescaler = 128
  pinMode(BUZZER_PAS, OUTPUT);
  digitalWrite(BUZZER_PAS, LOW);
}

void buzzerOn(uint16_t freq)
{
  if (freq) {
    uint32_t ocr = 125000L / freq;
    if (ocr>255) {
      ocr=255;
    }
    if (!ocr) {
      ocr=1;
    }
    OCR4C = ocr;
    TCCR4A |= (1<<COM4B0); // enable output
  } else {
    TCCR4A &= ~(1<<COM4B0); // disable output
  }
}

#define buzzerOff(foo) buzzerOn(0)

#define Red_LED_ON  PORTD |= (1<<PORTD7);
#define Red_LED_OFF  PORTD &= ~(1<<PORTD7);

#define Green_LED_ON   PORTC |= (1<<PORTC6);
#define Green_LED_OFF  PORTC &= ~(1<<PORTC6);

//## RFM22B Pinouts for Public Edition (M2)
#define  nIRQ_1 (PINB & (1<<PINB7))==(1<<PINB7) //PB7
#define  nIRQ_0 (PINB & (1<<PINB7))==0x00 //PB7

#define  nSEL_on PORTD |= (1<<PORTD6) //PD6
#define  nSEL_off PORTD &= ~(1<<PORTD6) //PD6

#define  SCK_on  PORTB |= (1<<PORTB1)  //PB1
#define  SCK_off PORTB &= ~(1<<PORTB1) //PB1

#define  SDI_on  PORTB |= (1<<PORTB2)  //PB2 MOSI
#define  SDI_off PORTB &= ~(1<<PORTB2) //PB2 MOSI

#define  SDO_1 (PINB & (1<<PINB3)) == (1<<PINB3) //PB3 MISO
#define  SDO_0 (PINB & (1<<PINB3)) == 0x00  //PB3 MISO

//can't do this, they are not D-pins on leonardo
//#define SDO_pin x //PB3
//#define SDI_pin x //PB2
//#define SCLK_pin x //PB1
#define IRQ_pin 11 //PB7
#define nSel_pin 12


void setupSPI()
{
  DDRB |= (1<<DDB1); // SCK PB1 output
  DDRB |= (1<<DDB2); // SDI/MOSI PB2 output
  DDRB &= ~(1<<DDB3); // SDO/MISO PB3 input
  pinMode(IRQ_pin, INPUT);   //IRQ
  pinMode(nSel_pin, OUTPUT);   //nSEL
}

void setupRfmInterrupt()
{
  PCMSK0 |= (1<<PCINT7); //enable pin change interrupt
  PCICR |= (1<<PCIE0);
}

ISR(PCINT0_vect)
{
  if(nIRQ_0) { //check if pin is low
    RFM22B_Int();
  }
}

#define SWAP_GPIOS

#endif

