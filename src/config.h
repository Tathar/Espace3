#ifndef CONFIG_H
#define CONFIG_H

#include <pinout.h>
// SPI :
// pin 48: Digital Pot (logique inversé)
// pin 50: MISO
// pin 51: MOSI
// pin 52: SCLK
// pin 53 Can Bus (logique inversé)
//#define PIN_SPI_CS_DP 48
//#define PIN_SPI_CS_CAN 53

//Can Bus
// Pin 2: interuption (données disponible)

#define PIN_CB_DATA PIN_CAN_INTERUPT
#define CB_ID_FP 0x766
#define CB_MASK_MA 0x00
#define CB_ADDRESS_FP 0x00
#define CB_ID_BAT 0x00
#define CB_ADDRESS_BAT 0x00

//octet 0 porte 0x8 AvG 0x10 AVD 0x80 coffre 0x20 arG 0x40 ar D
#define CB_PORTE_AVG 0x08
#define CB_PORTE_AVD 0x10
#define CB_PORTE_ARG 0x20
#define CB_PORTE_ARD 0x40
#define CB_PORTE_COFFRE 0x80

// Commande au volant
// pin 48: A1
// pin 47: A2
// pin 46: A3
// pin 45: B1
// pin 44: B2
// pin 43: B3

// B1 => A3 Bouton Bas
// B1 => B3 Vol -
// B1 => A1 Vol +
// B2 => A3 Btn haut droit
// B2 => B3 Btn haut gauche
// A2 => A3 Molette Cran 1  A2 = blanc
// A2 => B3 Molette Cran 2
// A2 => A1 Molette Cran 3

#define CV_A1 PIN_CV_A1 // input = Vol + / Molette 3
#define CV_A2 PIN_CV_A2 // Output =  Molette 1 2 3
#define CV_A3 PIN_CV_A3 // input = Btn bas / Btn Haut Droite / molette 1
#define CV_B1 PIN_CV_B1 // Output = Btn bas / vol + / Vol +
#define CV_B2 PIN_CV_B2 // Output = Btn Haut droite / Btn haut gauche
#define CV_B3 PIN_CV_B3 // input = Vol + / Molette 3

#define NONE 0
#define CV_BTN_DOWN 1
#define CV_VOL_DOWN 2
#define CV_VOL_UP 4
#define CV_BTN_UP_RGT 8
#define CV_BTN_UP_LFT 16
#define CV_MOL_1 64
#define CV_MOL_2 128
#define CV_MOL_3 256
#define CV_VOL_UP_DOWN 6
#define CV_BTN_UP_LFT_BTN_DOWN 9
#define CV_BTN_UP_LFT_RGT 24
#define CV_MASK_CURSOR 1 + 2 + 4
#define CV_MASK_MOL 64 + 128 + 256
#define CV_MASK_ALL 0xFFFF

//todo
// commande autoradio
// pin 26: relais Tip
// pin 28: relais Ring

#define AR_TIP PIN_PCB_RELAY_POT_2 //logique inversé
#define AR_RING PIN_PCB_RELAY_1    //logique inversé pin26

// source / 2 sec = off => 1.2
// MUTE => 3.5
// display => 5.75
// next => 8
// prev => 11.25
// Vol up => 16
// Vol down => 24
// band => 62.7

#define AR_NEED_RING 256

#define NONE 0
#define AR_SOURCE 3
#define AR_MUTE 9
#define AR_DISPLAY 14
#define AR_NEXT 20
#define AR_PREV 28
#define AR_VOL_UP 41
#define AR_VOL_DOWN 61
#define AR_BAND 149 //160
#define AR_PRESET_UP 20 + AR_NEED_RING
#define AR_PRESET_DOWN 28 + AR_NEED_RING

#define POT0_SEL 0x11
#define POT1_SEL 0x12
#define BOTH_POT_SEL 0x13

#define POT0_SHUTDOWN 0x21
#define POT1_SHUTDOWN 0x22
#define BOTH_POT_SHUTDOWN 0x23

// pin 32: relais frein de parking
// pin 34: relais marche arriere

#define PIN_AR_MA PIN_PCB_RELAY_2 //logique inversé
#define PIN_AR_FP PIN_PCB_RELAY_3 //logique inversé

//retroviseur
// pin 38: relais ouverture retro
// pin 37: relais fermeture retro

#define PIN_RV_OPEN 23
#define PIN_RV_CLOSE 25
#define PIN_RELAI_3 27 //inutilisé
#define PIN_RELAI_4 29 //inutilisé

#define RV_TIME 3000 //temp d ouverture et fermeture du retroviseur en ms

//Trape autoradio
// pin 3 (PWM)
// pin 30 relais alime servo

#define PIN_TRAP_PWM PIN_PWM_SRV_1  //3
#define PIN_TRAP_ALIM PIN_PWR_SRV_1 //30

#define TRAP_STOP 48
#define TRAP_SPEED 12
#define TRAP_OPEN_TIME 3500
#define TRAP_CLOSE_TIME 2500

#define TRAP_UP TRAP_STOP - 15
#define TRAP_DOWN TRAP_STOP + TRAP_SPEED

//resistance chauffante
// pin 38 : relais
// pin 40 : relais
// pin 42 : relais

#define PIN_RC_O_1 PIN_PCB_RELAY_4  //logique inversé 42
#define PIN_RC_O_23 PIN_PCB_RELAY_5 //logique inversé 40
#define PIN_RC_O_4 PIN_PCB_RELAY_6  //logique inversé 38

//entree resistance chauffante
/*
#define PIN_RC_I_1 7  //logique inversé Beige
#define PIN_RC_I_23 6 //logique inversé Noir
#define PIN_RC_I_4 5  //logique inversé Violet
*/
#define RC_OFF 0
#define RC_1 1
#define RC_23 2
#define RC_4 4

#define RC_123 3
#define RC_14 5
#define RC_234 6

#define RC_1234 7
#define RC_ALL RC_1234

#define DEBOUNCE 20
#define LONG_DEBOUNCE 80

#endif