#ifndef __CONFIG_H__
#define __CONFIG_H__

// Cell Voltage warning levels, unit 1/100th Volt
#define CELL_LOW_LOW_VOLTAGE 280
#define CELL_LOW_VOLTAGE 300
#define CELL_HIGH_VOLTAGE 345 
#define CELL_HIGH_HIGH_VOLTAGE 365

// Pack Voltage warning levels, unit volts
#define PACK_LOW_LOW_VOLTAGE 314
#define PACK_LOW_VOLTAGE 320
#define PACK_HIGH_VOLTAGE  400
#define PACK_HIGH_HIGH_VOLTAGE 408

// Battery warning temps
#define PACK_LOW_LOW_TEMP 40
#define PACK_LOW_TEMP 60
#define PACK_HIGH_TEMP 80
#define PACK_HIGH_HIGH_TEMP 100
    
// Relay Statuses
#define RELAY_ON 0
#define RELAY_OFF 1

// Relay Connections - Connector X358 (contactors control)
#define Relay_X358_7  3  // X358-7 K3: Main -VE [G]
#define Relay_X358_8  4  // X358-8 Multifunction contactor [E]
#define Relay_X358_9  5  // X358-9 K1: Precharge contactor for Main [L]
#define Relay_X358_10  6  // X358-10 K2: Main +VE contactor [F]

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
#define DELAY_STATUSMSG 500    // Time between battery status messages
#define DELAY_MODFMSG 10000    // Time between battery module temp messages
#define DELAY_PCKFMSG 30000    // Time between battery pack temp messages

#endif
