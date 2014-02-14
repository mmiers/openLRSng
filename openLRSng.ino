// **********************************************************
// ************************ openLRSng ***********************
// **********************************************************
// ** by Kari Hautio - kha @ AeroQuad/RCGroups/IRC(Freenode)
// ** other commits by cTn-dev, rlboyd, DTFUHF, pwarren
//
// Developer chat at IRC: #openLRS @ freenode
//
// This code is based on original OpenLRS and thUndeadMod
//
// Donations for development tools and utilities (beer) here
// https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=DSWGKGKPRX5CS
//
// Please note that for basic usage there is no need to use this
// code in source form. Instead use configurator program available
// freely in Google Chrome store.
// http://goo.gl/iX7dJx
//
// **********************************************************
// ************ based on: OpenLRS thUndeadMod ***************
// Mihai Andrian - thUndead http://www.fpvuk.org/forum/index.php?topic=3642.0
//
// **********************************************************
// *************** based on: OpenLRS Code *******************
// ***  OpenLRS Designed by Melih Karakelle on 2010-2011  ***
// **  an Arudino based RC Rx/Tx system with extra futures **
// **       This Source code licensed under GPL            **
// **********************************************************

// **********************************************************
// **************** original OpenLRS DEVELOPERS *************
// Mihai Andrian - thUndead http://www.fpvuk.org/forum/index.php?topic=3642.0
// Melih Karakelle (http://www.flytron.com) (forum nick name: Flytron)
// Jan-Dirk Schuitemaker (http://www.schuitemaker.org/) (forum nick name: CrashingDutchman)
// Etienne Saint-Paul (http://www.gameseed.fr) (forum nick name: Etienne)

//################################
//### HW CONFIGURATION SECTION ###
//################################

// NOTE: All settings are made via the CLI or configurator interface at runtime

// To compile with Arduino select TX/RX and BOARD_TYPE setting as needed below

//####### COMPILATION TARGET #######
// Enable to compile transmitter code, default is RX
//#define COMPILE_TX

//####### TX BOARD TYPE #######
// 0 = Flytron OpenLRS M1 Tx Board (not verified)
// 1 = Flytron OpenLRS M1 Rx Board as TX (not verified)
// 2 = Flytron OpenLRS M2/M3 Tx Board / OrangeRx UHF TX
// 3 = Flytron OpenLRS Rx v2 Board / OrangeRx UHF RX / HawkEye UHF RX workking as TX
// 4 = OpenLRSngTX / HawkEye UHF TX
// 5 = OpenLRSngRX-4ch (DTF UHF) as TX
// 6 = DTF UHF DeluxeTX (Atmega32u4)
//#define BOARD_TYPE 6

//####### RX BOARD TYPE #######
// 3 = Flytron OpenLRS Rx v2 / OrangeRx UHF RX / HawkEye UHF RX
// 5 = OpenLRSngRX-4ch (DTF UHF)
//#define BOARD_TYPE 3

//### Module type selection (only for modified HW)
//#define RFMXX_868
//#define RFMXX_915

//####################
//### CODE SECTION ###
//####################

#include <Arduino.h>

#include "version.h"
#include "rfm2x.h"
#include "hardware.h"
#include "binding.h"
#include "common.h"

#ifdef COMPILE_TX
#include "binary_com.h"
#include "dialog.h"
#include "frskytx.h"
#include "TX.h"
#else // COMPILE_RX
#include "I2C.h"
#include "serialPPM.h"
#include "RX.h"
#endif
