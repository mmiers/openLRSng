/*
  Simple CLI dialog
*/

#define EDIT_BUFFER_SIZE 100

int8_t  CLI_menu = 0;
char    CLI_buffer[EDIT_BUFFER_SIZE + 1];
uint8_t CLI_buffer_position = 0;
bool    CLI_magic_set = 0;

#ifdef HEXGET
const static char hexTab[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
#endif
const static char *chConfStr[8] = { "N/A", "4+4", "8", "8+4", "12", "12+4", "16", "N/A" };

#define RXC_MAX_SPECIAL_PINS 16
struct rxSpecialPinMap rxcSpecialPins[RXC_MAX_SPECIAL_PINS];
uint8_t rxcSpecialPinCount;
uint8_t rxcNumberOfOutputs;
uint16_t rxcVersion;

#ifdef HEXGET
void hexDump(void *in, uint16_t bytes)
{
  uint16_t check=0;
  uint8_t  *p = (uint8_t*)in;
  lrs_printf("@S:%u", bytes);
  if (bytes) {
    lrs_printf("H:");
    while (bytes) {
      lrs_putc(hexTab[*(p)>>4]);
      lrs_putc(hexTab[*(p)&15]);
      lrs_putc(',');
      check = ((check << 1) + ((check & 0x8000) ? 1 : 0));
      check ^= *p;
      p++;
      bytes--;
    }
  }
  lrs_printf("T:%x:", check);
}

void hexGet(void *out, uint16_t expected)
{
  uint32_t start = millis();
  uint16_t bytes = 0;
  uint8_t  state = 0;
  uint16_t numin = 0;
  uint8_t  buffer[expected];
  uint16_t check = 0;
  char     ch;
  while ((millis() - start) < 2000) {
    if (lrs_inputPending(Serial)) {
      ch=lrs_getc();
      switch (state) {
      case 0: // wait for S
        if (ch == 'S') {
          state = 1;
        } else {
          goto fail;
        }
        break;
      case 1: // wait for ':'
      case 3: // -"-
      case 6: // -"-
        if (ch == ':') {
          state++;
        } else {
          goto fail;
        }
        break;
      case 2: // bytecount receving (decimal number)
        if ((ch >= '0') && (ch <= '9')) {
          bytes = bytes * 10 + (ch -'0');
        } else if (ch == 'H') {
          if (bytes == expected) {
            state = 3;
            numin = 0;
            bytes = 0;
            check = 0;
          } else {
            goto fail;
          }
        } else {
          goto fail;
        }
        break;
      case 4: // reading bytes (hex)
        if ((ch >= '0') && (ch <= '9')) {
          numin = numin * 16 + (ch - '0');
        } else if ((ch >= 'A') && (ch <= 'F')) {
          numin = numin * 16 + (ch - 'A' + 10);
        } else if (ch == ',') {
          buffer[bytes++] = numin;
          check = ((check << 1) + ((check & 0x8000) ? 1 : 0));
          check ^= (numin & 0xff);
          numin = 0;
          if (bytes == expected) {
            state = 5;
          }
        } else {
          goto fail;
        }
        break;
      case 5:
        if (ch == 'T') {
          state = 6;
          numin = 0;
        } else {
          goto fail;
        }
        break;
      case 7:
        if ((ch >= '0') && (ch <= '9')) {
          numin = numin * 16 + (ch - '0');
        } else if ((ch >= 'A') && (ch <= 'F')) {
          numin = numin * 16 + (ch - 'A' + 10);
        } else if (ch == ':') {
          if (check == numin) {
            lrs_puts("BINARY LOAD OK");
            memcpy(out, buffer, expected);
            return;
          }
          goto fail;
        }
      }
    }
  }
fail:
  lrs_puts("Timeout or error!!");
  delay(10);
  while (lrs_inputPending(Serial)) {
    lrs_getc();
  }
}
#endif

void bindPrint(void)
{

  lrs_printf("1) Base frequency:     %lu\r\n", bind_data.rf_frequency);
  lrs_printf("2) RF magic:           %lx\r\n", bind_data.rf_magic);
  lrs_printf("3) RF power (0-7):     %hu\r\n", bind_data.rf_power);
  lrs_printf("4) Channel spacing:    %hu\r\n", bind_data.rf_channel_spacing);
  lrs_printf("5) Hop channels:       ");
  for (uint8_t c = 0; (c < MAXHOPS) && (bind_data.hopchannel[c] != 0); c++) {
    if (c) {
      lrs_putc(',');
    }
    lrs_printf("%hu", bind_data.hopchannel[c]);
  }
  lrs_putc('\r');
  lrs_putc('\n');

  lrs_printf("6) Datarate (0-2):     %hu\r\n", bind_data.modem_params);
  lrs_printf("7) Channel config:     %s\r\n", chConfStr[bind_data.flags & 0x07]);
  lrs_printf("8) Telemetry:          ");
  switch (bind_data.flags & TELEMETRY_MASK) {
  case TELEMETRY_OFF:
    lrs_puts("Disabled");
    break;
  case TELEMETRY_PASSTHRU:
    lrs_puts("Transparent");
    break;
  case TELEMETRY_FRSKY:
    lrs_puts("FrSky");
    break;
  case TELEMETRY_SMARTPORT:
    lrs_puts("smartPort");
    break;
  }

  lrs_printf("9) Serial baudrate:      %ld\r\n", bind_data.serial_baudrate);
  lrs_printf("0) Mute buzzer (mostly): %s\r\n", (bind_data.flags & MUTE_TX) ? "Yes" : "No");
  lrs_printf("A) Inverted PPM in:      %s\r\n", (bind_data.flags & INVERTED_PPMIN) ? "Yes" : "No");
  lrs_printf("B) Micro (half) PPM:     %s\r\n", (bind_data.flags & MICROPPM) ? "Yes" : "No");
  lrs_printf("C) MAVlink framing     :", (bind_data.flags & MAVLINK_FRAMING) ? "Yes" : "No");
  lrs_printf("D) Reverse PPM RSSI    :", (bind_data.flags & REVERSE_PPM_RSSI) ? "Yes" : "No");
  lrs_printf("E) Telemetry packet size:", bind_data.serial_downlink);
  lrs_printf("Calculated packet interval: %lu == %lu Hz\r\n",
             getInterval(&bind_data), 1000000L / getInterval(&bind_data));
}

void rxPrintDTime(uint8_t val)
{
  if (!val) {
    lrs_puts("Disabled");
  } else {
    uint32_t ms = delayInMs(val) / 100;
    lrs_printf("%lu.%lus\r\n", ms / 10, ms % 10);
  }
}

void rxPrint(void)
{
  uint8_t i;
  lrs_printf("RX type: ");
  if (rx_config.rx_type == RX_FLYTRON8CH) {
    lrs_puts("Flytron/OrangeRX UHF 8ch");
  } else if (rx_config.rx_type == RX_OLRSNG4CH) {
    lrs_puts("OpenLRSngRX mini 4/6ch");
  } else if (rx_config.rx_type == RX_DTFUHF10CH) {
    lrs_puts("DTF UHF 32-bit 10ch");
  }
  for (i=0; i < rxcNumberOfOutputs; i++) {
    lrs_putc((char)(((i + 1) > 9) ? (i + 'A' - 9) : (i + '1')));
    lrs_printf(") port %d function: ", i + 1);
    if (rx_config.pinMapping[i] < 16) {
      lrs_printf("PWM channel %d\r\n", rx_config.pinMapping[i] + 1);
    } else {
      lrs_printf("%s\r\n", SPECIALSTR(rx_config.pinMapping[i]));
    }
  }
  lrs_printf("F) Failsafe delay       : "); rxPrintDTime(rx_config.failsafeDelay);
  lrs_printf("G) PPM stop delay       : "); rxPrintDTime(rx_config.ppmStopDelay);
  lrs_printf("H) PWM stop delay       : "); rxPrintDTime(rx_config.pwmStopDelay);
  lrs_printf("I) Failsafe beacon frq. : ");
  if (rx_config.beacon_frequency) {
    lrs_printf("%lu\r\n", rx_config.beacon_frequency);
    lrs_printf("J) Failsafe beacon delay (0-255 => 10s - 150min): %lu\r\n", delayInMsLong(rx_config.beacon_deadtime));
    lrs_printf("K) Failsafe beacon intv. (1-255s): %hu\r\n", rx_config.beacon_interval);
  } else {
    lrs_printf("DISABLED\r\n");
  }
  lrs_printf("L) PPM minimum sync (us)  : %u\r\n", rx_config.minsync);
  lrs_printf("M) PPM RSSI to channel    : ");
  if (rx_config.RSSIpwm < 16) {
    lrs_printf("%i\r\n", rx_config.RSSIpwm + 1);
  } else {
    lrs_puts("DISABLED");
  }
  lrs_printf("N) PPM output limited       : %s\r\n", (rx_config.flags & PPM_MAX_8CH) ? "8ch" : "N/A");
  lrs_printf("O) Timed BIND at startup    : %s\r\n", (rx_config.flags & ALWAYS_BIND) ? "Enabled" : "Disabled");
  lrs_printf("P) Slave mode (experimental): %s\r\n", (rx_config.flags & SLAVE_MODE) ? "Enabled" : "Disabled");
  lrs_printf("Q) Output before link (=FS) : %s\r\n", (rx_config.flags & IMMEDIATE_OUTPUT) ? "Enabled" : "Disabled");
}

void CLI_menu_headers(void)
{

  switch (CLI_menu) {
  case -1:
    lrs_printf("\n\nopenLRSng ");
    printVersion(version);
    lrs_puts(" - System configuration");
    lrs_puts("Use numbers [0-9] to edit parameters");
    lrs_puts("[S] save settings to EEPROM and exit menu");
    lrs_puts("[X] revert changes and exit menu");
    lrs_puts("[I] reinitialize settings to sketch defaults");
    lrs_puts("[R] calculate random key and hop list");
    lrs_puts("[F] display actual frequencies used");
    lrs_puts("[Z] enter receiver configuration utility\n");
    bindPrint();
    break;
  case 1:
    lrs_puts("Set base frequency (in Hz): ");
    break;
  case 2:
    lrs_puts("Set RF magic (hex) e.g. 0xDEADF00D: ");
    break;
  case 3:
    lrs_puts("Set RF power (0-7): ");
    break;
  case 4:
    lrs_puts("Set channel spacing (x10kHz): ");
    break;
  case 5:
    lrs_puts("Set Hop channels (max 24, separated by commas) valid values 1-255: ");
    break;
  case 6:
    lrs_puts("Set Datarate (0-2): ");
    break;
  case 7:
    lrs_puts("Set Channel config: ");
    lrs_puts("Valid choices: 1 - 4+4 / 2 - 8 / 3 - 8+4 / 4 - 12 / 5 - 12+4 / 6 - 16");
    break;
  case 9:
    lrs_puts("Set serial baudrate: ");
    break;
  case 11:
    lrs_puts("Set telemetry packet size: ");
    break;
  }

  // Flush input
  delay(10);
  while (lrs_inputPending(Serial)) {
    lrs_getc();
  }
}

void RX_menu_headers(void)
{
  uint8_t ch;
  switch (CLI_menu) {
  case -1:
    lrs_printf("\n\nopenLRSng ");
    printVersion(version);
    lrs_printf(" - receiver configurator, rx sw ");
    printVersion(rxcVersion);
    lrs_puts("\r\nUse numbers [1-D] to edit ports [E-Q] for settings");
    lrs_puts("[R] revert RX settings to defaults");
    lrs_puts("[S] save settings to RX");
    lrs_puts("[X] abort changes and exit RX config\n");
    rxPrint();
    break;
  default:
    if ((CLI_menu > 0) && (CLI_menu <= rxcNumberOfOutputs)) {
      lrs_printf("Set output for port %hd\r\n", CLI_menu);
      lrs_printf("Valid choices are: [1]-[16] (channel 1-16)");
      ch=20;
      for (uint8_t i = 0; i < rxcSpecialPinCount; i++) {
        if (rxcSpecialPins[i].output == CLI_menu - 1) {
          lrs_printf(", [%hu] (%s)", ch, SPECIALSTR(rxcSpecialPins[i].type));
          ch++;
        }
      }
      lrs_putc('\r');
      lrs_putc('\n');
    }
    break;
  }
}

void showFrequencies()
{
  for (uint8_t ch = 0; (ch < MAXHOPS) && (bind_data.hopchannel[ch] != 0) ; ch++) {
    lrs_printf("Hop channel %hu @ %lu\r\n",
               ch, bind_data.rf_frequency + 10000L * bind_data.hopchannel[ch] * bind_data.rf_channel_spacing);
  }
}

void CLI_buffer_reset(void)
{
  // Empty buffer and reset position
  CLI_buffer_position = 0;
  memset(CLI_buffer, 0, sizeof(CLI_buffer));
}

uint8_t CLI_inline_edit(char c)
{
  if (c == 0x7F || c == 0x08) { // Delete or Backspace
    if (CLI_buffer_position > 0) {
      // Remove last char from the buffer
      CLI_buffer_position--;
      CLI_buffer[CLI_buffer_position] = 0;

      // Redraw the output with last character erased
      lrs_putc('\r');
      for (uint8_t i = 0; i < CLI_buffer_position; i++) {
        lrs_putc(CLI_buffer[i]);
      }
      lrs_putc(' ');
      lrs_putc('\r');
      for (uint8_t i = 0; i < CLI_buffer_position; i++) {
        lrs_putc(CLI_buffer[i]);
      }
    } else {
      lrs_putc('\007'); // bell
    }
  } else if (c == 0x1B) { // ESC
    CLI_buffer_reset();
    return 1; // signal editing done
  } else if (c == 0x0D || c == 0x0A) { // Enter/Newline
    return 1; // signal editing done
  } else {
    if (CLI_buffer_position < EDIT_BUFFER_SIZE) {
      lrs_putc(c);
      CLI_buffer[CLI_buffer_position++] = c; // Store char in the buffer
    } else {
      lrs_putc('\007'); // bell
    }
  }
  return 0;
}

void handleRXmenu(char c)
{
  uint8_t ch;
  if (CLI_menu == -1) {
    switch (c) {
    case '!':
#ifdef HEXGET
      hexDump(&rx_config, sizeof(rx_config));
#else
      lrs_puts("NOT ENABLED");
#endif
      break;
    case '\n':
    case '\r':
      RX_menu_headers();
      break;
    case '@':
#ifdef HEXGET
      hexGet(&rx_config, sizeof(rx_config));
#else
      lrs_puts("NOT ENABLED");
#endif
      RX_menu_headers();
      break;
    case 's':
    case 'S': {
      lrs_puts("Sending settings to RX\n");
      uint8_t tx_buf[1 + sizeof(rx_config)];
      tx_buf[0] = 'u';
      memcpy(tx_buf + 1, &rx_config, sizeof(rx_config));
      tx_packet(tx_buf, sizeof(rx_config) + 1);
      rx_reset();
      RF_Mode = Receive;
      delay(200);
      if (RF_Mode == Received) {
        spiSendAddress(0x7f);   // Send the package read command
        tx_buf[0] = spiReadData();
        if (tx_buf[0] == 'U') {
          lrs_puts("*****************************");
          lrs_puts("RX Acked - update successful!");
          lrs_puts("*****************************");
        }
      }
    }
    break;
    case 'r':
    case 'R': {
      lrs_puts("Resetting settings on RX\n");
      uint8_t tx_buf[1 + sizeof(rx_config)];
      tx_buf[0] = 'i';
      tx_packet(tx_buf, 1);
      rx_reset();
      RF_Mode = Receive;
      delay(200);
      if (RF_Mode == Received) {
        spiSendAddress(0x7f); // Send the package read command
        tx_buf[0] = spiReadData();
        for (uint8_t i = 0; i < sizeof(rx_config); i++) {
          tx_buf[i + 1] = spiReadData();
        }
        memcpy(&rx_config, tx_buf + 1, sizeof(rx_config));
        if (tx_buf[0]=='I') {
          lrs_puts("*****************************");
          lrs_puts("RX Acked - revert successful!");
          lrs_puts("*****************************");
        }
      }
    }
    break;
    case 'x':
    case 'X':
    case 0x1b: //ESC
      // restore settings from EEPROM
      lrs_puts("Aborted edits");
      // leave CLI
      CLI_menu = -2;
      break;
    case 'a':
    case 'b':
    case 'c':
    case 'd':
      c -= 'a' - 'A';
      // Fallthru
    case 'A':
    case 'B':
    case 'C':
    case 'D':
      c -= 'A' - 10 - '0';
      // Fallthru
    case '9':
    case '8':
    case '7':
    case '6':
    case '5':
    case '4':
    case '3':
    case '2':
    case '1':
      c -= '0';
      if ( c > rxcNumberOfOutputs) {
        lrs_puts("invalid selection");
        break;
      }
      CLI_menu = c;
      RX_menu_headers();
      break;
    case 'f':
    case 'F':
      CLI_menu = 20;
      lrs_puts("Set failsafe delay (0 disabled, 1-255 == 0.1s - 50min)");
      break;
    case 'g':
    case 'G':
      CLI_menu = 21;
      lrs_puts("Set PPM stop delay (0 disabled, 1-255 == 0.1s - 50min)");
      break;
    case 'h':
    case 'H':
      CLI_menu = 22;
      lrs_puts("Set PWM stop delay (0 disabled, 1-255 == 0.1s - 50min)");
      break;
    case 'i':
    case 'I':
      CLI_menu = 23;
      lrs_puts("Set beacon frequency in Hz: 0=disable, Px=PMR channel x, Fx=FRS channel x");
      break;
    case 'j':
    case 'J':
      CLI_menu = 24;
      lrs_puts("Set beacon delay");
      break;
    case 'k':
    case 'K':
      CLI_menu = 25;
      lrs_puts("Set beacon interval");
      break;
    case 'l':
    case 'L':
      CLI_menu = 26;
      lrs_puts("Set PPM minimum sync");
      break;
    case 'm':
    case 'M':
      CLI_menu = 27;
      lrs_puts("Set RSSI injection channel (0==disable)");
      break;
    case 'n':
    case 'N':
      lrs_puts("Toggled PPM channel limit");
      rx_config.flags ^= PPM_MAX_8CH;
      CLI_menu = -1;
      RX_menu_headers();
      break;
    case 'o':
    case 'O':
      lrs_puts("Toggled 'always bind'");
      rx_config.flags ^= ALWAYS_BIND;
      CLI_menu = -1;
      RX_menu_headers();
      break;
    case 'p':
    case 'P':
      lrs_puts("Toggled 'slave mode'");
      rx_config.flags ^= SLAVE_MODE;
      CLI_menu = -1;
      RX_menu_headers();
      break;
    case 'q':
    case 'Q':
      lrs_puts("Toggled 'immediate output'");
      rx_config.flags ^= IMMEDIATE_OUTPUT;
      CLI_menu = -1;
      RX_menu_headers();
      break;
    }
    while (lrs_inputPending(Serial)) {
      lrs_getc();
    }
  } else { // we are inside the menu
    if (CLI_inline_edit(c)) {
      if (CLI_buffer_position == 0) { // no input - abort
        CLI_menu = -1;
        RX_menu_headers();
      } else {
        uint32_t value = strtoul(CLI_buffer, NULL, 0);
        bool valid_input = 0;
        switch (CLI_menu) {
        case 13:
        case 12:
        case 11:
        case 10:
        case 9:
        case 8:
        case 7:
        case 6:
        case 5:
        case 4:
        case 3:
        case 2:
        case 1:
          if (CLI_menu > rxcNumberOfOutputs) {
            break;
          }
          if ((value > 0) && (value <= 16)) {
            rx_config.pinMapping[CLI_menu - 1] = value - 1;
            valid_input = 1;
          } else {
            ch=20;
            for (uint8_t i = 0; i < rxcSpecialPinCount; i++) {
              if (rxcSpecialPins[i].output != (CLI_menu - 1)) {
                continue;
              }
              if (ch == value) {
                rx_config.pinMapping[CLI_menu - 1] = rxcSpecialPins[i].type;
                valid_input = 1;
              }
              ch++;
            }
          }
          break;
        case 20:
          if (value <= 255) {
            rx_config.failsafeDelay = value;
            valid_input = 1;
          }
          break;
        case 21:
          if (value <= 255) {
            rx_config.ppmStopDelay = value;
            valid_input = 1;
          }
          break;
        case 22:
          if (value <= 255) {
            rx_config.pwmStopDelay = value;
            valid_input = 1;
          }
          break;
        case 23:
          if ((CLI_buffer[0] | 0x20) == 'p') {
            value = strtoul(CLI_buffer + 1, NULL, 0);
            if ((value >= 1) && (value <= 8)) {
              value=EU_PMR_CH(value);
            } else {
              value = 1; //invalid
            }
          } else if ((CLI_buffer[0] | 0x20) == 'f') {
            value = strtoul(CLI_buffer + 1, NULL, 0);
            if ((value >= 1) && (value <= 7)) {
              value = US_FRS_CH(value);
            } else {
              value = 1; //invalid
            }
          }
          if ((value == 0) || ((value >= MIN_RFM_FREQUENCY) && (value <= MAX_RFM_FREQUENCY))) {
            rx_config.beacon_frequency = value;
            valid_input = 1;
          }
          break;
        case 24:
          if ((value >= MIN_DEADTIME) && (value <= MAX_DEADTIME)) {
            rx_config.beacon_deadtime = value;
            valid_input = 1;
          }
          break;
        case 25:
          if ((value >= MIN_INTERVAL) && (value <= MAX_INTERVAL)) {
            rx_config.beacon_interval = value;
            valid_input = 1;
          }
          break;
        case 26:
          if ((value >= 2500) && (value <= 10000)) {
            rx_config.minsync = value;
            valid_input = 1;
          }
        case 27:
          if (value == 0) {
            rx_config.RSSIpwm = 255;
            valid_input = 1;
          } else if (value <= 16) {
            rx_config.RSSIpwm = value - 1;
            valid_input = 1;
          }
          break;
        }
        if (valid_input) {
          if (CLI_magic_set == 0) {
            bind_data.rf_magic++;
          }
        } else {
          lrs_puts("\r\nInvalid input - discarded!\007");
        }
        CLI_buffer_reset();
        // Leave the editing submenu
        CLI_menu = -1;
        lrs_putc('\r');
        lrs_putc('\n');
        RX_menu_headers();
      }
    }
  }
}

uint8_t rxcConnect()
{
  uint8_t tx_buf[1 + sizeof(rx_config)];
  uint32_t last_time = micros();

  init_rfm(1);
  do {
    tx_buf[0] = 't';
    tx_packet(tx_buf, 1);
    RF_Mode = Receive;
    rx_reset();
    delay(250);
  } while ((RF_Mode == Receive) && (!lrs_inputPending(Serial)) && ((micros() - last_time) < 30000000));

  if (RF_Mode == Receive) {
    return 2;
  }

  spiSendAddress(0x7f);   // Send the package read command
  tx_buf[0] = spiReadData();
  if (tx_buf[0] != 'T') {
    return 3;
  }

  rxcVersion = (uint16_t)spiReadData() * 256;
  rxcVersion += spiReadData();

  rxcNumberOfOutputs = spiReadData();
  rxcSpecialPinCount = spiReadData();
  if (rxcSpecialPinCount > RXC_MAX_SPECIAL_PINS) {
    return 3;
  }

  for (uint8_t i = 0; i < sizeof(struct rxSpecialPinMap) * rxcSpecialPinCount; i++) {
    *(((uint8_t*)&rxcSpecialPins) + i) = spiReadData();
  }

  tx_buf[0] = 'p'; // ask for config dump
  tx_packet(tx_buf, 1);
  RF_Mode = Receive;
  rx_reset();
  delay(50);

  if (RF_Mode == Receive) {
    return 2;
  }
  spiSendAddress(0x7f);   // Send the package read command
  tx_buf[0] = spiReadData();
  if (tx_buf[0] != 'P') {
    return 3;
  }

  for (uint8_t i = 0; i < sizeof(rx_config); i++) {
    *(((uint8_t*)&rx_config) + i) = spiReadData();
  }
  return 1;
}

void CLI_RX_config()
{
  lrs_puts("Connecting to RX, power up the RX (with bind plug if not using always bind)");
  lrs_puts("Press any key to cancel");
  if (lrs_inputPending(Serial)) {
    handleRXmenu(lrs_getc());
  }

  switch (rxcConnect()) {
  case 2:
    lrs_puts("Timeout when connecting to RX");
    return;
  case 3:
    lrs_puts("Protocol error with RX");
    return;
  }

  CLI_menu = -1;
  CLI_magic_set = 0;
  RX_menu_headers();
  while (CLI_menu != -2) { // LOCK user here until settings are saved or abandonded
    if (lrs_inputPending(Serial)) {
      handleRXmenu(lrs_getc());
    }
  }
}

void handleCLImenu(char c)
{
  if (CLI_menu == -1) {
    switch (c) {
    case '!':
#ifdef HEXGET
      hexDump(&bind_data, sizeof(bind_data));
#else
      lrs_puts("NOT ENABLED");
#endif
      break;
    case '\n':
    case '\r':
      CLI_menu_headers();
      break;
    case '@':
#ifdef HEXGET
      hexGet(&bind_data, sizeof(bind_data));
#else
      lrs_puts("NOT ENABLED");
#endif
      CLI_menu_headers();
      break;
    case 's':
    case 'S':
      // save settings to EEPROM
      bindWriteEeprom();
      lrs_puts("Settings saved to EEPROM");
      // leave CLI
      CLI_menu = -2;
      break;
    case 'x':
    case 'X':
    case 0x1b: //ESC
      // restore settings from EEPROM
      bindReadEeprom();
      lrs_puts("Reverted settings from EEPROM");
      // leave CLI
      CLI_menu = -2;
      break;
    case 'i':
    case 'I':
      // restore factory settings
      bindInitDefaults();
      lrs_puts("Loaded factory defaults");

      CLI_menu_headers();
      break;
    case 'r':
    case 'R':
      // randomize channels and key
      bindRandomize();
      lrs_puts("Key and channels randomized");

      CLI_menu_headers();
      break;
    case 'f':
    case 'F':
      showFrequencies();
      //CLI_menu_headers();
      break;
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
      CLI_menu = c - '0';
      CLI_menu_headers();
      break;
    case '8':
      lrs_puts("Toggled telemetry!");
      {
        uint8_t newf = (bind_data.flags + TELEMETRY_PASSTHRU) & TELEMETRY_MASK;
        bind_data.flags &= ~TELEMETRY_MASK;
        bind_data.flags |= newf;
      }
      CLI_menu = -1;
      CLI_menu_headers();
      break;
    case '9':
      CLI_menu = 9;
      CLI_menu_headers();
      break;
    case '0':
      lrs_puts("Toggled TX muting!");
      bind_data.flags ^= MUTE_TX;
      CLI_menu = -1;
      CLI_menu_headers();
      break;
    case 'a':
    case 'A':
      lrs_puts("Toggled inverted PPM!");
      bind_data.flags ^= INVERTED_PPMIN;
      CLI_menu = -1;
      CLI_menu_headers();
      break;
    case 'b':
    case 'B':
      lrs_puts("Toggled microPPM");
      bind_data.flags ^= MICROPPM;
      CLI_menu = -1;
      CLI_menu_headers();
      break;
    case 'c':
    case 'C':
      lrs_puts("Toggled MAVlink framing");
      bind_data.flags ^= MAVLINK_FRAMING;
      CLI_menu = -1;
      CLI_menu_headers();
      break;
    case 'd':
    case 'D':
      lrs_puts("Toggled Reverse PPM RSSI");
      bind_data.flags ^= REVERSE_PPM_RSSI;
      CLI_menu = -1;
      CLI_menu_headers();
      break;
    case 'e':
    case 'E':
      CLI_menu = 11;
      CLI_menu_headers();
      break;
    case 'z':
    case 'Z':
      CLI_RX_config();
      CLI_menu = -1;
      CLI_menu_headers();
      break;
    }
  } else { // we are inside the menu
    if (CLI_inline_edit(c)) {
      if (CLI_buffer_position == 0) { // no input - abort
        CLI_menu = -1;
        CLI_menu_headers();
      } else {
        uint32_t value = strtoul(CLI_buffer, NULL, 0);
        bool valid_input = 0;
        switch (CLI_menu) {
        case 1:
          if ((value > MIN_RFM_FREQUENCY) && (value < MAX_RFM_FREQUENCY)) {
            bind_data.rf_frequency = value;
            valid_input = 1;
          }
          break;
        case 2:
          bind_data.rf_magic = value;
          CLI_magic_set = 1; // user wants specific magic, do not auto update
          valid_input = 1;
          break;
        case 3:
          if (value < 8) {
            bind_data.rf_power = value;
            valid_input = 1;
          }
          break;
        case 4:
          if ((value > 0) && (value < 11)) {
            bind_data.rf_channel_spacing = value;
            valid_input = 1;
          }
          break;
        case 5: {
          char* slice = strtok(CLI_buffer, ",");
          uint8_t channel = 0;
          while ((slice != NULL) && (atoi(slice) != 0)) {
            if (channel < MAXHOPS) {
              bind_data.hopchannel[channel++] = atoi(slice);
            }
            slice = strtok(NULL, ",");
          }
          valid_input = 1;
          while (channel < MAXHOPS) {
            bind_data.hopchannel[channel++] = 0;
          }
        }
        break;
        case 6:
          if (value < DATARATE_COUNT) {
            bind_data.modem_params = value;
            valid_input = 1;
          }
          break;
        case 7:
          if ((value >= 1) && (value <= 6)) {
            bind_data.flags &= 0xf8;
            bind_data.flags |= value;
            valid_input = 1;
          }
          break;
        case 9:
          if ((value >= 1200) && (value <= 115200)) {
            bind_data.serial_baudrate = value;
            valid_input = 1;
          }
          break;
        case 11:
          if ((value >= 1) && (value <= 63)) {
            bind_data.serial_downlink = value;
            valid_input = 1;
          }
          break;
        }
        if (valid_input) {
          if (CLI_magic_set == 0) {
            bind_data.rf_magic++;
          }
        } else {
          lrs_puts("\r\nInvalid input - discarded!\007");
        }
        CLI_buffer_reset();
        // Leave the editing submenu
        CLI_menu = -1;
        lrs_putc('\r');
        lrs_putc('\n');
        CLI_menu_headers();
      }
    }
  }
}

void handleCLI()
{
  CLI_menu = -1;
  CLI_magic_set = 0;
  CLI_menu_headers();
  while (CLI_menu != -2) { // LOCK user here until settings are saved
    if (lrs_inputPending(Serial)) {
      handleCLImenu(lrs_getc());
    }
  }

  // Clear buffer
  delay(10);
  while (lrs_inputPending(Serial)) {
    lrs_getc();
  }
}

void binaryMode()
{
  // Just entered binary mode, flip the bool
  binary_mode_active = true;

  PSP_init(&binary_com);
  while (binary_mode_active == true) { // LOCK user here until exit command is received
    if (SerialAvailable(Serial)) {
      PSP_read_packet(&binary_com);
    }
  }
}

