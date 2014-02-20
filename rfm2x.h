//
// RFM2X info
//

//
// Some things are just better left to macros
//
// Convert a value and offset to a bit pattern
//
#define RFM2X_MAKE_BITS(x, v) \
    (((v) << (RFM2X_##x##_SHIFT)) & RFM2X_##x##_MASK)
#define RFM2X_TAKE_BITS(x, v) \
    (((v) & RFM2X_##x##_MASK) >> (RFM2X_##x##_SHIFT))

// Register table: Section 10
#define RFM2X_REG_DEV_TYPE                0x00U
#define RFM2X_REG_DEV_VER                 0x01U
#define RFM2X_REG_DEV_STATUS              0x02U
#define RFM2X_REG_INT_STAT_1              0x03U
#define RFM2X_REG_INT_STAT_2              0x04U
#define RFM2X_REG_INT_EN_1                0x05U
#define RFM2X_REG_INT_EN_2                0x06U
#define RFM2X_REG_OP_FUNC_CTRL_1          0x07U
#define RFM2X_REG_OP_FUNC_CTRL_2          0x08U
#define RFM2X_REG_OSC_LOAD_CAP            0x09U
#define RFM2X_REG_MIC_OUTPUT_CLK          0x0AU
#define RFM2X_REG_GPIO0_CFG               0x0BU
#define RFM2X_REG_GPIO1_CFG               0x0CU
#define RFM2X_REG_GPIO2_CFG               0x0DU
#define RFM2X_REG_IO_PORT_CFG             0x0EU
#define RFM2X_REG_ADC_CFG                 0x0FU

#define RFM2X_REG_ADC_SENS_AMP_OFF        0x10U
#define RFM2X_REG_ADC_VAL                 0x11U
#define RFM2X_REG_TEMP_SENS_CTRL          0x12U
#define RFM2X_REG_TEMP_VAL_OFF            0x13U
#define RFM2X_REG_WAKE_TIMER_PER_1        0x14U
#define RFM2X_REG_WAKE_TIMER_PER_2        0x15U
#define RFM2X_REG_WAKE_TIMER_PER_3        0x16U
#define RFM2X_REG_WAKE_TIMER_VAL_1        0x17U
#define RFM2X_REG_WAKE_TIMER_VAL_2        0x18U
#define RFM2X_REG_LOW_DUTY_CYC            0x19U
#define RFM2X_REG_LOW_BATT_DET            0x1AU
#define RFM2X_REG_BATT_VOLT_LVL           0x1BU
#define RFM2X_REG_IF_FILT_BW              0x1CU
#define RFM2X_REG_AFC_LOOP_GEAR           0x1DU
#define RFM2X_REG_AFC_TIME_CTRL           0x1EU
#define RFM2X_REG_CLK_REC_GEAR            0x1FU

#define RFM2X_REG_CLK_RECOV_OVER_RAT      0x20U
#define RFM2X_REG_CLK_RECOV_OFF_2         0x21U
#define RFM2X_REG_CLK_RECOV_OFF_1         0x22U
#define RFM2X_REG_CLK_RECOV_OFF_0         0x23U
#define RFM2X_REG_CLK_RECOV_TLG_1         0x24U
#define RFM2X_REG_CLK_RECOV_TLG_0         0x25U
#define RFM2X_REG_RSSI                    0x26U
#define RFM2X_REG_RSSI_TCCI               0x27U
#define RFM2X_REG_ANT_DIV_1               0x28U
#define RFM2X_REG_ANT_DIV_2               0x29U
#define RFM2X_REG_AFC_LIMITER             0x2AU
#define RFM2X_REG_AFC_CORR_READ           0x2BU
#define RFM2X_REG_OOK_CTR_VAL_1           0x2CU
#define RFM2X_REG_OOK_CTR_VAL_2           0x2DU
#define RFM2X_REG_SLICER_PEAK_HOLD        0x2EU
    // 0x2FU - reserved

#define RFM2X_REG_DATA_ACC_CTRL           0x30U
#define RFM2X_REG_EZMAC_STAT              0x31U
#define RFM2X_REG_HEADER_CTRL_1           0x32U
#define RFM2X_REG_HEADER_CTRL_2           0x33U
#define RFM2X_REG_PREAMBLE_LEN            0x34U
#define RFM2X_REG_PREAMBLE_DET_CTRL       0x35U
#define RFM2X_REG_SYNC_WORD_3             0x36U
#define RFM2X_REG_SYNC_WORD_2             0x37U
#define RFM2X_REG_SYNC_WORD_1             0x38U
#define RFM2X_REG_SYNC_WORD_0             0x39U
#define RFM2X_REG_TRANSMIT_HDR_3          0x3AU
#define RFM2X_REG_TRANSMIT_HDR_2          0x3BU
#define RFM2X_REG_TRANSMIT_HDR_1          0x3CU
#define RFM2X_REG_TRANSMIT_HDR_0          0x3DU
#define RFM2X_REG_TRANSMIT_PKT_LEN        0x3EU
#define RFM2X_REG_CHECK_HDR_3             0x3FU

#define RFM2X_REG_CHECK_HDR_2             0x40U
#define RFM2X_REG_CHECK_HDR_1             0x41U
#define RFM2X_REG_CHECK_HDR_0             0x42U
#define RFM2X_REG_HDR_EN_3                0x43U
#define RFM2X_REG_HDR_EN_2                0x44U
#define RFM2X_REG_HDR_EN_1                0x45U
#define RFM2X_REG_HDR_EN_0                0x46U
#define RFM2X_REG_RCVD_HDR_3              0x47U
#define RFM2X_REG_RCVD_HDR_2              0x48U
#define RFM2X_REG_RCVD_HDR_1              0x49U
#define RFM2X_REG_RCVD_HDR_0              0x4AU
#define RFM2X_REG_RCVD_PKT_LEN            0x4BU
    // 0x4CU - 0x4EU reserved
#define RFM2X_REG_ADC8_CTRL               0x4FU

    // 0x50U - 0x5FU reserved

#define RFM2X_REG_CHAN_FILT_COEF          0x60U
    // 0x61U reserved
#define RFM2X_REG_CRYS_OSC_CTRL           0x62U
#define RFM2X_REG_AGC_OVER                0x63U
    // 0x64U - 0x6CU reserved
#define RFM2X_REG_TX_PWR                  0x6DU
#define RFM2X_REG_TX_DATA_RATE_1          0x6EU
#define RFM2X_REG_TX_DATA_RATE_2          0x6FU

#define RFM2X_REG_MOD_MODE_CTRL_1         0x70U
#define RFM2X_REG_MOD_MODE_CTRL_2         0x71U
#define RFM2X_REG_FREQ_DEV                0x72U
#define RFM2X_REG_FREQ_OFF_1              0x73U
#define RFM2X_REG_FREQ_OFF_2              0x74U
#define RFM2X_REG_FREQ_BAND_SEL           0x75U
#define RFM2X_REG_NOM_CAR_FREQ_1          0x76U
#define RFM2X_REG_NOM_CAR_FREQ_2          0x77U
    // 0x78U reserved
#define RFM2X_REG_FREQ_HOP_CH_SEL         0x79U
#define RFM2X_REG_FREQ_HOP_ST_SZ          0x7AU
    // 0x7BU reserved
#define RFM2X_REG_TX_FIFO_CTRL_1          0x7CU
#define RFM2X_REG_TX_FIFO_CTRL_2          0x7DU
#define RFM2X_REG_RX_FIFO_CTRL            0x7EU
#define RFM2X_REG_FIFO_ACCESS             0x7FU

// Register 00 - Device type
#define RFM2X_DEV_TYPE_7                      0x07U
// Register 01 - Device version
#define RFM2X_DEV_VER_6                       0x06U
// Register 02 - Device status
#define RFM2X_DEV_ST_FFOVFL                   0x80U
#define RFM2X_DEV_ST_FFUNFL                   0x40U
#define RFM2X_DEV_ST_RXFFEM                   0x20U
#define RFM2X_DEV_ST_HEADERR                  0x10U
#define RFM2X_DEV_ST_CPS_MASK                 0x03U
#define RFM2X_DEV_ST_CPS_SHIFT                0
// Register 03 - Interrupt register 1
#define RFM2X_INT_FFERR                       0x80U
#define RFM2X_INT_TXFFAFULL                   0x40U
#define RFM2X_INT_TXFFAEM                     0x20U
#define RFM2X_INT_RXFFAFULL                   0x10U
#define RFM2X_INT_EXT                         0x08U
#define RFM2X_INT_PKSENT                      0x04U
#define RFM2X_INT_PKVALID                     0x02U
#define RFM2X_INT_CRCERROR                    0x01U
// Register 04 - Interrupt register 2
#define RFM2X_INT_SWDET                       0x80U
#define RFM2X_INT_PREAVAL                     0x40U
#define RFM2X_INT_PREAINVAL                   0x20U
#define RFM2X_INT_RSSI                        0x10U
#define RFM2X_INT_WUT                         0x08U
#define RFM2X_INT_LBD                         0x04U
#define RFM2X_INT_CHIPRDY                     0x02U
#define RFM2X_INT_POR                         0x01U
// Register 05 - Interrupt enable register 1
// Register 06 - Interrupt enable register 2
// Register 07 - Operating and function control 1
#define RFM2X_OFC_1_SWRES                     0x80U
#define RFM2X_OFC_1_ENLBD                     0x40U
#define RFM2X_OFC_1_ENWT                      0x20U
#define RFM2X_OFC_1_X32KSEL                   0x10U
#define RFM2X_OFC_1_TXON                      0x08U
#define RFM2X_OFC_1_RXON                      0x04U
#define RFM2X_OFC_1_PLLON                     0x02U
#define RFM2X_OFC_1_XTON                      0x01U
// Register 08 - Operating and function control 2
#define RFM2X_OFC_2_ANTDIV_MASK               0xE0U
#define RFM2X_OFC_2_ANTDIV_SHIFT              5
#define RFM2X_OFC_2_RXMPK                     0x10U
#define RFM2X_OFC_2_AUTOTX                    0x08U
#define RFM2X_OFC_2_ENLDM                     0x04U
#define RFM2X_OFC_2_FFCLRRX                   0x02U
#define RFM2X_OFC_2_FFCLRTX                   0x01U
// Register 09 Crystal oscillator load capacitance
#define RFM2X_OLC_XTALSHFT                    0x80U
#define RFM2X_OLC_XLC_MASK                    0x7FU
#define RFM2X_OLC_XLC_SHIFT                   0
// Register 0A Microcontroller output clock
#define RFM2X_MOC_CLKT_MASK                   0x30U
#define RFM2X_MOC_CLKT_SHIFT                  4
#define RFM2X_MOC_ENLFC                       0x08U
#define RFM2X_MOC_MCLK_MASK                   0x07U
#define RFM2X_MOC_MCLK_SHIFT                  0
// Register 0B GPIO 0 configuration
#define RFM2X_GPIO_DRV_MASK                   0xC0U
#define RFM2X_GPIO_DRV_SHIFT                  6
#define RFM2X_GPIO_PUP                        0x20U
#define RFM2X_GPIO_MASK                       0x1FU
#define RFM2X_GPIO_SHIFT                      0
// Register 0C GPIO 1 configuration
// Register 0D GPIO 2 configuration
// Register 0E I/O port configuration
#define RFM2X_IOPC_EXTITST_MASK               0x70U
#define RFM2X_IOPC_EXTITST_SHIFT              4
#define RFM2X_IOPC_ITSD0                      0x08U
#define RFM2X_IOPC_DIO2                       0x04U
#define RFM2X_IOPC_DIO1                       0x02U
#define RFM2X_IOPC_DIO0                       0x01U
// Register 0F ADC configuration
#define RFM2X_AC_START                        0x80U
#define RFM2X_AC_ADCSEL_MASK                  0x70U
#define RFM2X_AC_ADCSEL_SHIFT                 4
#define RFM2X_AC_ADCREF_MASK                  0x0CU
#define RFM2X_AC_ADCREF_SHIFT                 2
#define RFM2X_AC_ADCAGAIN_MASK                0x03U
#define RFM2X_AC_ADCAGAIN_SHIFT               0

// Register 10 ADC sensor amplifier offset
#define RFM2X_ASAO_ADC_OFFS_MASK              0x0FU
#define RFM2X_ASAO_ADC_OFFS_SHIFT             0
// Register 11 ADC value - WHOLE REGISTER
// Register 12 Temperature sensor control
#define RFM2X_TSC_RANGE_MASK                  0xC0U
#define RFM2X_TSC_RANGE_SHIFT                 6
#define RFM2X_TSC_ENTSOFFS                    0x20U
#define RFM2X_TSC_ENTSTRIM                    0x10U
#define RFM2X_TSC_TSTRIM_MASK                 0x0FU
#define RFM2X_TSC_TSTRIM_SHIFT                0
// Register 13 Temperature value offset - WHOLE REGISTER
// Register 14 Wake-up timer period 1
#define RFM2X_WTR__MASK                       0x1FU
#define RFM2X_WTR_SHIFT                       0
// Register 15 Wake-up timer period 2 - WHOLE REGISTER
// Register 16 Wake-up timer period 3 - WHOLE REGISTER
// Register 17 Wake-up timer value 1 - WHOLE REGISTER
// Register 18 Wake-up timer value 2 - WHOLE REGISTER
// Register 19 Low duty cycle mode duration - WHOLE REGISTER
// Register 1A Low battery detector threshold
#define RFM2X_LBDT_MASK                       0x1FU
#define RFM2X_LBDT_SHIFT                      0
// Register 1B Battery voltage level
#define RFM2X_VBAT_MASK                       0x1FU
#define RFM2X_VBAT_SHIFT                      0
// Register 1C IF filter bandwidth
#define RFM2X_IFB_DWN3_BYPASS                 0x80U
#define RFM2X_IFB_NDEC_MASK                   0x70U
#define RFM2X_IFB_NDEC_SHIFT                  4
#define RFM2X_IFB_FILSET_MASK                 0x0FU
#define RFM2X_IFB_FILSET_SHIFT                0
// Register 1D AFC loop gearshift override
#define RFM2X_ALG_AFCBD                       0x80U
#define RFM2X_ALG_ENAFC                       0x40U
#define RFM2X_ALG_AFCGEARH_MASK               0x38U
#define RFM2X_ALG_AFCGEARH_SHIFT              3
#define RFM2X_ALG_1P5BYPASS                   0x04U
#define RFM2X_ALG_MATAP                       0x02U
#define RFM2X_ALG_PH0SIZE                     0x01U
// Register 1E AFC timing control
#define RFM2X_ATC_SWAIT_TIMER_MASK            0xC0U
#define RFM2X_ATC_SWAIT_TIMER_SHIFT           6
#define RFM2X_ATC_SHWAIT_MASK                 0x38U
#define RFM2X_ATC_SHWAIT_SHIFT                3
#define RFM2X_ATC_ANWAIT_MASK                 0x07U
#define RFM2X_ATC_ANWAIT_SHIFT                0
// Register 1F Clock recovery gearshift override
#define RFM2X_CRG_CRFAST_MASK                 0x38U
#define RFM2X_CRG_CRFAST_SHIFT                3
#define RFM2X_CRG_CRSLOW_MASK                 0x07U
#define RFM2X_CRG_CRSLOW_SHIFT                0

// Register 20 Clock recovery oversampling ratio - WHOLE REGISTER
// Register 21 Clock recovery offset 2
#define RFM2X_CRO_RXOSR_MASK                  0xE0U
#define RFM2X_CR0_RXOSR_SHIFT                 5
#define RFM2X_CR0_STALLCTRL                   0x10U
#define RFM2X_CR0_NCOFF_MASK                  0x70U
#define RFM2X_CR0_NCOFF_SHIFT                 0
// Register 22 Clock recovery offset 1 - WHOLE REGISTER
// Register 23 Clock recovery offset 0 - WHOLE REGISTER
// Register 24 Clock recovery timing loop gain 1
#define RFM2X_CRTLG_RXNCOCOMP                 0x10U
#define RFM2X_CRTLG_CRGAIN2X                  0x08U
#define RFM2X_CRTLG_CRGAIN_MASK               0x07U
#define RFM2X_CRTLG_CRGAIN_SHIFT              0
// Register 25 Clock recovery timing loop gain 0 - WHOLE REGISTER
// Register 26 Received signal strength indicator - WHOLE REGISTER
// Register 27 RSSI threshold for clear channel indicator - WHOLE REGISTER
// Register 28 Antenna diversity 1 - WHOLE REGISTER
// Register 29 Antenna diversity 2 - WHOLE REGISTER
// Register 2A AFC limiter - WHOLE REGISTER
// Register 2B AFC correction read - WHOLE REGISTER
// Register 2C OOK counter value 1
#define RFM2X_OCV1_AFC_CORR_MASK              0xC0U
#define RFM2X_OCV1_AFC_CORR_SHIFT             6
#define RFM2X_OCV1_OOKFRZEN                   0x20U
#define RFM2X_OCV1_PEAKDETEN                  0x10U
#define RFM2X_OCV1_MADETEN                    0x08U
#define RFM2X_OCV1_OOKCNT_MASK                0x07U
#define RFM2X_OCV1_OOKCNT_SHIFT               0
// Register 2D OOK counter value 2 - WHOLE REGISTER
// Register 2E Slicer peak hold
#define RFM2X_SPH_ATACK_MASK                  0x70U
#define RFM2X_SPH_ATTACH_SHIFT                4
#define RFM2X_SPH_DECAY_MASK                  0x0FU
#define RFM2X_SPH_DECAY_SHIFT                 0

// Register 30 Data access control
#define RFM2X_DAC_ENPACRX                     0x80U
#define RFM2X_DAC_LSBFRST                     0x40U
#define RFM2X_DAC_CRCDONLY                    0x20U
#define RFM2X_DAC_SKIP2PH                     0x10U
#define RFM2X_DAC_ENPACTX                     0x08U
#define RFM2X_DAC_ENCRC                       0x04U
#define RFM2X_DAC_CRC_MASK                    0x03U
#define RFM2X_DAC_CRC_SHIFT                   0
// Register 31 EzMAC status
#define RFM2X_EZMACS_RXCRC1                   0x40U
#define RFM2X_EZMACS_PKSRCH                   0x20U
#define RFM2X_EZMACS_PKRX                     0x10U
#define RFM2X_EZMACS_PKVALID                  0x08U
#define RFM2X_EZMACS_CRCERROR                 0x04U
#define RFM2X_EZMACS_PKTX                     0x02U
#define RFM2X_EZMACS_PKSENT                   0x01U
// Register 32 Header control 1
#define RFM2X_HC1_BCEN_MASK                   0xF0U
#define RFM2X_HC1_BCEN_SHIFT                  4
#define RFM2X_HC1_HDCH_MASK                   0x0FU
#define RFM2X_HC1_HDCH_SHIFT                  0
// Register 33 Header control 2
#define RFM2X_HC2_SKIPSYN                     0x80U
#define RFM2X_HC2_HDLEN_MASK                  0x70U
#define RFM2X_HC2_HDLEN_SHIFT                 4
#define RFM2X_HC2_FIXPKLEN                    0x08U
#define RFM2X_HC2_SYNCLEN_MASK                0x07U
#define RFM2X_HC2_SYNCLEN_SHIFT               0
// Register 34 Preamble length - WHOLE REGISTER
// Register 35 Preamble detection control
#define RFM2X_PDC_PREATH_MASK                 0xF8U
#define RFM2X_PDC_PREATH_SHIFT                3
#define RFM2X_PDC_RSSI_OFF_MASK               0x07U
#define RFM2X_PDC_RSSI_OFF_SHIFT              0
// Register 36 Sync word 3 - WHOLE REGISTER
// Register 37 Sync word 2 - WHOLE REGISTER
// Register 38 Sync word 1 - WHOLE REGISTER
// Register 39 Sync word 0 - WHOLE REGISTER
// Register 3A Transmit header 3 - WHOLE REGISTER
// Register 3B Transmit header 2 - WHOLE REGISTER
// Register 3C Transmit header 1 - WHOLE REGISTER
// Register 3D Transmit header 0 - WHOLE REGISTER
// Register 3E Transmit packet length - WHOLE REGISTER
// Register 3F Check header 3 - WHOLE REGISTER

// Register 40 Check header 2 - WHOLE REGISTER
// Register 41 Check header 1 - WHOLE REGISTER
// Register 42 Check header 0 - WHOLE REGISTER
// Register 43 Header enable 3 - WHOLE REGISTER
// Register 44 Header enable 2 - WHOLE REGISTER
// Register 45 Header enable 1 - WHOLE REGISTER
// Register 46 Header enable 0 - WHOLE REGISTER
// Register 47 Received header 3 - WHOLE REGISTER
// Register 48 Received header 2 - WHOLE REGISTER
// Register 49 Received header 1 - WHOLE REGISTER
// Register 4A Received header 0 - WHOLE REGISTER
// Register 4B Received packet length
#define RFM2X_AC_ADC8_MASK                    0x3FU
#define RFM2X_AC_ADC8_SHIFT                   0

// Register 60 Channel filter coefficient address
#define RFM2X_CFC_LNV_PRE_TH_MASK             0xF0U
#define RFM2X_CFC_LNV_PRE_TH_SHIFT            4
#define RFM2X_CFC_CHFILADD_MASK               0x0FU
#define RFM2X_CFC_CHFILADD_SHIFT              0
// Register 62 Crystal oscillator control test
#define RFM2X_COCT_PWST_MASK                  0xE0U
#define RFM2X_COCT_PWST_SHIFT                 5
#define RFM2X_COCT_CLKHYST                    0x10U
#define RFM2X_COCT_ENBIAS2X                   0x08U
#define RFM2X_COCT_ENAMP2X                    0x04U
#define RFM2X_COCT_BUFOVR                     0x02U
#define RFM2X_COCT_ENBUF                      0x01U
// Register 63 AGC override 1
#define RFM2X_AO_SGI                          0x40U
#define RFM2X_AO_AGCEN                        0x20U
#define RFM2X_AO_LNAGAIN                      0x10U
#define RFM2X_AO_PGA3                         0x08U
#define RFM2X_AO_PGA2                         0x04U
#define RFM2X_AO_PGA1                         0x02U
#define RFM2X_AO_PGA0                         0x01U
// Register 6D TX power
#define RFM2X_TP_PAPEAKVAL                    0x80U
#define RFM2X_TP_PAPEAKEN                     0x40U
#define RFM2X_TP_PAPEAKLVL_MASK               0x30U
#define RFM2X_TP_PAPEAKLVL_SHIFT              4
#define RFM2X_TP_LNA_SW                       0x08U
#define RFM2X_TP_TXPOW_MASK                   0x07U
#define RFM2X_TP_TXPOW_SHIFT                  0
// Register 6E TX data rate 1 - WHOLE REGISTER
// Register 6F TX data rate 2 - WHOLE REGISTER

// Register 70 Modulation mode control 1
#define RFM2X_MMC1_TXDLRTSCALE                0x20U
#define RFM2X_MMC1_ENPHPWDN                   0x10U
#define RFM2X_MMC1_MANPPOL                    0x08U
#define RFM2X_MMC1_ENMANINV                   0x04U
#define RFM2X_MMC1_ENMANCH                    0x02U
#define RFM2X_MMC1_ENWHITE                    0x01U
// Register 71 Modulation mode control 2
#define RFM2X_MMC2_TRCLK_MASK                 0xC0U
#define RFM2X_MMC2_TRCLK_SHIFT                6
#define RFM2X_MMC2_DTMOD_MASK                 0x30U
#define RFM2X_MMC2_DTMOD_SHIFT                4
#define RFM2X_MMC2_ENINV                      0x08U
#define RFM2X_MMC2_FD8                        0x04U
#define RFM2X_MMC2_MODTYP_MASK                0x03U
#define RFM2X_MMC2_MODTYP_SHIFT               0
// Register 72 Frequency deviation - WHOLE REGISTER
// Register 73 Frequency offset 1 - WHOLE REGISTER
// Register 74 Frequency offset 2
#define RFM2X_FO2_FO_MASK                     0x03U
#define RFM2X_FO2_FO_SHIFT                    0
// Register 75 Frequency band select
#define RFM2X_FBS_SBSEL                       0x40U
#define RFM2X_FBS_HBSEL                       0x20U
#define RFM2X_FBS_FB_MASK                     0x1FU
#define RFM2X_FBS_FB_SHIFT                    0
// Register 76 Nominal carrier frequency 1 - WHOLE REGISTER
// Register 77 Nominal carrier frequency 2 - WHOLE REGISTER
// Register 79 Frequency hopping channel select - WHOLE REGISTER
// Register 7A Frequency hopping step size
// Register 7c TX FIFO control 1
#define RFM2X_TFC1_TXAFTHR_MASK               0x3FU
#define RFM2X_TFC1_TXAFTHR_SHIFT              0
// Register 7D TX FIFO control 2
#define RFM2X_TFC2_TXAETHR_MASK               0x3FU
#define RFM2X_TFC2_TXAETHR_SHIFT              0
// Register 7E RX FIFO control
#define RFM2X_RFC_RXAFTHR_MASK                0x3FU
#define RFM2X_RFC_RXAFTHR_SHIFT               0
// Register 7F FIFO access - WHOLE REGISTER
