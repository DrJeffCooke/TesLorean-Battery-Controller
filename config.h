#ifndef __CONFIG_H__
#define __CONFIG_H__

// Cell Voltage warning levels, unit 1/100th Volt
#define CELL_LOW_LOW_VOLTAGE 250
#define CELL_LOW_VOLTAGE 260
#define CELL_HIGH_VOLTAGE 350 
#define CELL_HIGH_HIGH_VOLTAGE 360

// Pack Voltage warning levels, unit volts
#define PACK_LOW_LOW_VOLTAGE 280
#define PACK_LOW_VOLTAGE 292
#define PACK_HIGH_VOLTAGE  393
#define PACK_HIGH_HIGH_VOLTAGE 403

// Battery warning temps in oC
#define PACK_LOW_LOW_TEMP 0x58    // 4C or 40F
#define PACK_LOW_TEMP 0x70        // 16C or 60F    
#define PACK_HIGH_TEMP 0x86       // 27C or 81F
#define PACK_HIGH_HIGH_TEMP 0x9C  // 38C or 100F

// Battery warning temps in oF
//#define PACK_LOW_LOW_TEMPF 40     // 4C or 40F
//#define PACK_LOW_TEMPF 60         // 16C or 60F 
//#define PACK_HIGH_TEMPF 80        // 27C or 81F
//#define PACK_HIGH_HIGH_TEMPF 100  // 38C or 100F
    
// Relay Statuses
#define RELAY_ON 0
#define RELAY_OFF 1

// CAN board Chip Select (CS) pins
#define TESLOREAN_CAN_CS 9
#define TESLOREAN_CAN_INT 3
#define BATTERY_CAN_CS 10
#define BATTERY_CAN_INT 2

// Relay Connections - Connector X358 (contactors control)
#define Relay_X358_10  12  // X358-10 K2: Main +VE contactor [F]
#define Relay_X358_7  4  // X358-7 K3: Main -VE [G]
#define Relay_X358_8  5  // X358-8 Multifunction contactor [E]
#define Relay_X358_9  6  // X358-9 K1: Precharge contactor for Main [L]

// Relay Connections - Connector X357 (data connection)
#define Relay_X357_4  7  // X357-3 to 12v accessory wake up
#define Relay_X357_5  8  // X357-14 to 12v HV energy mgt communication enable

// High Voltage Contactors
#define CONTACTORSOPEN 0        // All open -VE, MultiFunction, Pre-charge, and +VE
#define CONTACTORSNEGMUL 1      // Negative and MultiFunction closed, all others open
#define CONTACTORSPRECRG 2      // Pre-charger closed, Neg and MultiF also closed, others open
#define CONTACTORSPREPOS 3      // Pre-charger and +VE closed, Neg and MultiF also closed
#define CONTACTORSCLOSED 4      // Pre-charger opened, +VE, Neg, and MultiF also closed
#define CONTACTORSECONIM 5      // Switch -VE and +VE contactors to PWM (Economizer mode) - REQUIRES Solidstate switching

// Battery Data Communications
#define BMSCOMMSOFF 0          // No accessories or communications from the BMS
#define BMSCOMMSON 1           // Enable battery accessories and communications from the BMS

//********* DELAY SETTINGS ******************
#define DELAY_PRECHARGE 2000    // Max time (in ms) that the Pre-charger contactor is closed, and +VE open
#define DELAY_INTERRELAY 500    // Time between the contactor actions
#define DELAY_STATUSMSG 2000    // Time between battery status messages
#define DELAY_MODFMSG 5000    // Time between battery module temp messages
#define DELAY_PCKFMSG 8000    // Time between battery pack temp messages

#endif
