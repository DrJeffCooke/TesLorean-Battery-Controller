/*
  TesLorean Battery Control Module
  2018
  Jeff Cooke

  Based on Gen2 Charger Control Program by T de Bree, D.Maguire 2017-2018
  Additional work by C. Kidder
  
  Coding ToDos
  - DONE Converted moduletemps array to be single byte values for internal C, i.e. 0.5C offset -40
  - DONE Convert packtemps to single byte internal representation

  Coding Checks
  - Code that pulls module temps is occassionally pulling Module 7 wrong, suspect
    bits check as signalling a data point that isn't there

  - Suspect that Pack temps are only produced by battery when the accessories are switched on
    Reporting 0's consistently

  - 
  
  Coding Notes
  - Temps are reported in F or C?

  // check
*/

//****** DEBUG VARIABLES
#define debug_setup 1
#define debug_info 1
//#define debug 1

struct can_frame {
    uint16_t  can_id;
    uint8_t   can_dlc;
    uint8_t   data[8];
};

// Pulls on millis() less often
unsigned long curtimemillis;

//#include <mcp2515.h>
#include <mcp_can.h>
#include <SPI.h>
#include "config.h"
#include <avr/wdt.h>

// Variables for the CAN traffic
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

// Variables for 
#define StoredFrames 100

// Array for received CAN frames
can_frame CANFrames_can0[StoredFrames];
can_frame CANFrames_can1[StoredFrames];

// Counters tracking the add point and read point in the circular array
volatile uint8_t addPointFrames_can0 = 0;
volatile uint8_t readPointFrames_can0 = 0;
volatile uint8_t netAddReadCount_can0 = 0;   // Adds increment, Reads decrement, only Read if <> 0
volatile uint8_t canError_can0 = 0;
volatile uint8_t addPointFrames_can1 = 0;
volatile uint8_t readPointFrames_can1 = 0;
volatile uint8_t netAddReadCount_can1 = 0;   // Adds increment, Reads decrement, only Read if <> 0
volatile uint8_t canError_can1 = 0;

// Create the CAN objects, specific the 'Chip Select' line
MCP_CAN can0(BATTERY_CAN_CS);
MCP_CAN can1(TESLOREAN_CAN_CS);

//********* TIME SINCE VARIABLES *************
unsigned long timeContactorStatus = 0;
unsigned long timeBatteryStatus = 0;
unsigned long timeModTempsStatus = 0;
unsigned long timePckTempsStatus = 0;
unsigned long timeSincePackVChange = 0;

//********* STATE VARIABLES ******************
uint8_t contactorsstate;      // Current state of the contactors
uint8_t contactorsreqdstate;  // Requested state of the contactors
uint8_t bmsstate;             // Current state of the BMS communications
uint8_t bmsreqdstate;         // Requested state of the BMS communications
uint8_t cellvoltmodule;       // 1-8 : Module to send volts status for 0x150
uint8_t cellvolttriplet;      // 1-14 : Cell triplet to send volts status for 0x150

//********* GENERAL VARIABLES ******************
uint8_t incomingByte = 0;
unsigned long timeLastStatusMsg = 0;

//********* CAN IDs ******************
uint16_t BattConnFrameID = 0x132;
uint16_t BMSConnFrameID = 0x142;
uint16_t BattStatFrameID = 0x141;
uint16_t BattModFFrameID = 0x133;
uint16_t BattPckFFrameID = 0x131;
uint16_t BattVoltFrameID = 0x150;
uint16_t BattVoltReqFrameID = 0x151;
uint16_t BattWarnVFrameID = 0x0110;      // Voltage warning
uint16_t BattWarnTFrameID = 0x0111;      // Temperature warning
uint16_t BattWarnIFrameID = 0x0112;      // Isolation warning

//********* HELPER ARRAYS ************
uint8_t group4mods[8][3] = {{0, 0, 1}, {1, 2, 2}, {3, 3, 4}, {4, 5, 5}, {6, 6, 7}, {7, 9, 9}, {9, 9, 9}, {9, 9, 9}}; // Modules for Group 4 voltages
uint8_t startcellnum[5] = {0, 3, 6, 9, 12}; // cell voltage frames, starting triple voltage cell num

//********* DATA FROM BMS ******************
uint8_t moduletemps[8] = {0x78, 0x78, 0x78, 0x78, 0x78, 0x78, 0x78, 0x78};       // 1 = 0.5C, default to 20C which is 
uint16_t totalpackvolts = 0;                              // 1 = 1V
uint16_t priorpackvolts = 0;                             // Total voltage at prior reporting
uint8_t packtemps[2] = {0x78, 0x78};                           // 1 = 1F
uint16_t cellvolts[14][8];                                // 14 cells in each of 8 modules, 1 = 1/100th V
uint16_t mincellvolts = 0;                                // lowest individual cell voltage, 1 = 1/100th V
uint16_t maxcellvolts = 0;                                // highest individual cell voltage, 1 = 1/100th V
uint8_t frame260[5][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
uint8_t frame262[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
uint8_t frame270[8][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
uint8_t frame272[8][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
uint8_t frame274[2][3] = {{0, 0, 0}, {0, 0, 0}};

struct can_frame frame;

// Mimicks the CAN routine from the prior library
void sendMessage(MCP_CAN canx, can_frame & frame)
{
  canx.sendMsgBuf(frame.can_id, 0, frame.can_dlc, frame.data);
}

void setup()
{
  #ifdef debug_setup 
    Serial.println("Initializing Digital Pins for Relay control");
  #endif

  //// SETUP OUTPUT LINES
  pinMode(Relay_X358_7, OUTPUT);
  pinMode(Relay_X358_8, OUTPUT);
  pinMode(Relay_X358_9, OUTPUT);
  pinMode(Relay_X358_10, OUTPUT);
  pinMode(Relay_X357_4, OUTPUT);
  pinMode(Relay_X357_5, OUTPUT);
    
  //// INITIALIZE RELAY PINS
  digitalWrite(Relay_X358_7, RELAY_OFF); // X358-7 K3: Main -VE [G]
  digitalWrite(Relay_X358_8, RELAY_OFF); // X358-8 Multifunction contactor [E]
  digitalWrite(Relay_X358_9, RELAY_OFF); // X358-9 K1: Precharge contactor for Main [L]
  digitalWrite(Relay_X358_10, RELAY_OFF); // X358-10 K2: Main +VE contactor [F]
  digitalWrite(Relay_X357_4, RELAY_OFF); // X357-3 to 12v accessory wake up
  digitalWrite(Relay_X357_5, RELAY_OFF); // X357-14 to 12v HV energy mgt communication enable

  //// SERIAL PORT
  #ifdef debug_setup 
    Serial.println("Initializing serial port (115200 baud)");
  #endif
  // init the serial port - for status/debug
  while (!Serial);
  Serial.begin(115200);

  // init the SPI communications
  SPI.begin();

  // Initialize the array pointers
  addPointFrames_can0 = 0;
  readPointFrames_can0 = 0;
  netAddReadCount_can0 = 0;
  addPointFrames_can1 = 0;
  readPointFrames_can1 = 0;
  netAddReadCount_can1 = 0;

  // Initialize timers
  // Starts with CAN arrival
  //timeContactorStatus = millis();
  //timeBatteryStatus = millis();
  // Starts with Module power-up
  curtimemillis = millis();
  timeLastStatusMsg = curtimemillis;
  timeModTempsStatus = curtimemillis;
  timePckTempsStatus = curtimemillis;
  timeSincePackVChange = curtimemillis;
  
  // Prepare the interrupt pins
  pinMode(BATTERY_CAN_INT, INPUT);
  pinMode(TESLOREAN_CAN_INT, INPUT);

  // Startup CAN Battery bus
  can0.begin(CAN_1000KBPS);
  #ifdef debug_setup
    Serial.println("Battery CANbus initialized (1000 kbps)");
  #endif

  // Startup CAN  TesLorean bus
  can1.begin(CAN_500KBPS);
  #ifdef debug_setup
    Serial.println("TesLorean CANbus initialized (500 kbps)");
  #endif

  // Connect the interrupt to the CAN frame trigger
  // attachInterrupt(digitalPinToInterrupt(BATTERY_CAN_INT), irqBATHandler, LOW);
  // attachInterrupt(digitalPinToInterrupt(TESLOREAN_CAN_INT), irqTESHandler, LOW);

  // Set the watchdog - 8 second to reset
  wdt_enable(WDTO_8S);

  // Output the instructions for frames
  outputInstructions();
}

// Print out the instructions
void outputInstructions()
{
  Serial.println("Instructions");  
  Serial.println("'?' to reshow the instructions");
  Serial.println("'a' activate the BMS");
  Serial.println("'d' deactivate the BMS");
  Serial.println("'c' close the contactors - enable power");
  Serial.println("'o' open the contactors - shutdown power");
  Serial.println("'r' report all cell voltages");
}

void loop()
{
  // Loop logic - preference reading in new CAN frames, if one is read in skip the processing
  // Only run a process cycle when no arriving frames were waiting
  bool readSuccess = false;

  // Check CAN0 for Data - Battery CAN
  if(!digitalRead(BATTERY_CAN_INT))   // pin low if data on can0
  {
    can0.readMsgBuf(&CANFrames_can0[addPointFrames_can0].can_dlc, CANFrames_can0[addPointFrames_can0].data);
    CANFrames_can0[addPointFrames_can0].can_id = can0.getCanId();
    readSuccess = true;
    // Bump readPoint forward if overwritten
    if(netAddReadCount_can0 > 0 && readPointFrames_can0 == addPointFrames_can0)
    {
        readPointFrames_can0 = (readPointFrames_can0 + 1) % StoredFrames;
        netAddReadCount_can0--;
    }
    // Bump the addPoint forward
    addPointFrames_can0 = (addPointFrames_can0 + 1) % StoredFrames;
    if (netAddReadCount_can0 < StoredFrames){netAddReadCount_can0++;}    
  }

  // Check CAN1 for Data - TesLorean CAN
  if(!digitalRead(TESLOREAN_CAN_INT))   // pin low if data on can1
  {
    can1.readMsgBuf(&CANFrames_can1[addPointFrames_can1].can_dlc, CANFrames_can1[addPointFrames_can1].data);
    CANFrames_can1[addPointFrames_can1].can_id = can1.getCanId();
    readSuccess = true;
    // Bump readPoint forward if overwritten
    if(netAddReadCount_can1 > 0 && readPointFrames_can1 == addPointFrames_can1)
    {
        readPointFrames_can1 = (readPointFrames_can1 + 1) % StoredFrames;
        netAddReadCount_can1--;
    }
    // Bump the addPoint forward
    addPointFrames_can1 = (addPointFrames_can1 + 1) % StoredFrames;
    if (netAddReadCount_can1 < StoredFrames){netAddReadCount_can1++;}  
  }
  
  // Define CAN Frame variable structure
  struct can_frame incoming;

  // If a frame was just read, skip processing until there is a break
  if(!readSuccess)
  {
    // Check for a frame ready to read from BATTCAN
    can_frame frame0;
    if (netAddReadCount_can0 > 0)
    {
      #ifdef debug 
        Serial.println("CAN frame available on Battery bus - decoding");
      #endif
      // Copy data into temp store
      frame0.can_id = CANFrames_can0[readPointFrames_can0].can_id;
      frame0.can_dlc = CANFrames_can0[readPointFrames_can0].can_dlc;
      for (int i = 0; i < frame0.can_dlc; i++)
      {
        frame0.data[i] = CANFrames_can0[readPointFrames_can0].data[i];
      }
      readPointFrames_can0 = (readPointFrames_can0 + 1) % StoredFrames;
      if(netAddReadCount_can0 > 0){netAddReadCount_can0--;}
    
      // Process frame
      canBATTERYdecode(frame0);
    }
  
    // Check for a frame ready to read from TESLCAN
    can_frame frame1;
    if (netAddReadCount_can1 > 0)
    {
    // Process frame
      #ifdef debug 
        Serial.println("CAN frame available on TesLorean bus - decoding");
      #endif
      // Copy data into temp store
      frame1.can_id = CANFrames_can1[readPointFrames_can1].can_id;
      frame1.can_dlc = CANFrames_can1[readPointFrames_can1].can_dlc;
      for (int i = 0; i < frame1.can_dlc; i++)
      {
        frame1.data[i] = CANFrames_can1[readPointFrames_can1].data[i];
      }
      readPointFrames_can1 = (readPointFrames_can1 + 1) % StoredFrames;
      if(netAddReadCount_can1 > 0){netAddReadCount_can1--;}
      
      // Process the frame
      canTESLOREANdecode(frame1);
    }

  } // end !readSuccess (only process on non-receive cycles)

  // Check the Serial Bus for commands
  if (Serial.available())
  {
    incomingByte = Serial.read(); // read the incoming byte:

    switch (incomingByte)
    {
      case '?': //d for debug
        // Output the instructions for instructions
        outputInstructions();
        break;

      case 'a': //a for activate BMS
        // Deactivate the battery data communications
        bmsreqdstate = BMSCOMMSON;
        break;
        
      case 'd': //d for deactivate BMS
        // Activate the battery data communications
        bmsreqdstate = BMSCOMMSOFF;
        break;

      case 'c': //c for contactors closed
        // Set the contactors closed - power enabled
        contactorsreqdstate = CONTACTORSCLOSED;
        break;
        
      case 'o': //d for contactors open
        // Set the contactors open - power disabled
        contactorsreqdstate = CONTACTORSOPEN;
        break;
        
      case 'r': //r for report cell voltages
        // Set the counters that signal transmission underway
        cellvoltmodule = 1;
        cellvolttriplet = 1;
        break;
              
      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }  // end of Serial input check

  // Check for Request to change to a different Battery Contactors Status
  if (contactorsreqdstate != contactorsstate)
  {
    // Request to CLOSE (activate) the battery contactors
    if (contactorsreqdstate == CONTACTORSCLOSED)
    {
      // Catch current time
      curtimemillis = millis();

      switch (contactorsstate)
      {
        case CONTACTORSOPEN:

          #ifdef debug_info
            Serial.println(F("Prior status: CONTACTORSOPEN; New status: CONTACTORSNEGMUL"));
          #endif

          // Switch on the -VE and Multifunction Contactors
          digitalWrite(Relay_X358_7, RELAY_ON);  // Enable Main -VE Contactor
          digitalWrite(Relay_X358_8, RELAY_ON);  // Enable Multifunction Contactor

          // Immediately change to state CONTACTORNEGMUL
          contactorsstate = CONTACTORSNEGMUL;

          // Start the contactorsstatetimer
          timeContactorStatus = curtimemillis;

          break;

        case CONTACTORSNEGMUL:
          // Check that the require time has passed in this state
          if ((curtimemillis - timeContactorStatus) > DELAY_INTERRELAY)
          {
            #ifdef debug_info
              Serial.println(F("Prior status: CONTACTORSNEGMUL; New status: CONTACTORSPRECRG"));
            #endif

            // Switch on the PreCharge Contactor
            digitalWrite(Relay_X358_9, RELAY_ON);  // Enable Pre-charger +VE Contactor

            // Change to state CONTACTORSPRECRG
            contactorsstate = CONTACTORSPRECRG;

            // Restart the contactorsstatetimer
            timeContactorStatus = curtimemillis;
          }
          break;

        case CONTACTORSPRECRG:
          // Check that the require time has passed in this state
          if ((curtimemillis - timeContactorStatus) > DELAY_PRECHARGE)
          {
            #ifdef debug_info
              Serial.println(F("Prior status: CONTACTORSPRECRG; New status: CONTACTORSPREPOS"));
            #endif

            // Switch on the Positive Contactor
            digitalWrite(Relay_X358_10, RELAY_ON);  // Enable Main +VE Contactor

            // Change to state CONTACTORSPREPOS
            contactorsstate = CONTACTORSPREPOS;

            // Restart the contactorsstatetimer
            timeContactorStatus = curtimemillis;
          }
          break;

        case CONTACTORSPREPOS:
          // Check that the require time has passed in this state
          if ((curtimemillis - timeContactorStatus) > DELAY_INTERRELAY)
          {
            #ifdef debug_info
              Serial.println(F("Prior status: CONTACTORSPREPOS; New status: CONTACTORSCLOSED"));
            #endif

            // Switch off the PreCharge Contactor
            digitalWrite(Relay_X358_9, RELAY_OFF);  // Disable Pre-charger +VE Contactor

            // Change to state CONTACTORSCLOSED
            contactorsstate = CONTACTORSCLOSED;

            // Restart the contactorsstatetimer
            timeContactorStatus = curtimemillis;
          }
          break;

        default:
          // if nothing else matches, do the default
          break;
      }
    }

    // Request to OPEN (deactivate) the battery contactors
    if (contactorsreqdstate == CONTACTORSOPEN)
    {
      #ifdef debug_info
        Serial.println(F("New status: CONTACTORSOPEN"));
      #endif

      // Immediately deactivate all the contactors (open)
      digitalWrite(Relay_X358_10, RELAY_OFF);  // Disable Main +VE Contactor
      digitalWrite(Relay_X358_7, RELAY_OFF);  // Disable Main -VE Contactor
      digitalWrite(Relay_X358_8, RELAY_OFF);  // Disable Multifunction Contactor

      // Change to state CONTACTORSOPEN
      contactorsstate = CONTACTORSOPEN;
    }

  }  // Required state != Current state

  // Check for Request to change to a different BMS Communication Status
  if (bmsreqdstate != bmsstate)
  {

    // Request to turn on the BMS communications
    if (bmsreqdstate == BMSCOMMSON)
    {
      #ifdef debug_info
        Serial.println(F("BMS communications: ON"));
      #endif
      
      // Enable the signal lines
      digitalWrite(Relay_X357_4, RELAY_ON); // X357-3 Accessory wake-up
      digitalWrite(Relay_X357_5, RELAY_ON); // X357-14 HV communications enable

      // Update the status
      bmsstate = BMSCOMMSON;
    }

    // Request to turn off the BMS communications
    if (bmsreqdstate == BMSCOMMSOFF)
    {
      #ifdef debug_info 
        Serial.println(F("BMS communications: OFF"));
      #endif

      // Disable the signal lines
      digitalWrite(Relay_X357_5, RELAY_OFF); // X357-14 HV communications enable
      digitalWrite(Relay_X357_4, RELAY_OFF); // X357-3 Accessory wake-up

      // Update the status
      bmsstate = BMSCOMMSOFF;
    }
  }

  // catch the current time
  curtimemillis = millis();

  // Output a debug status message on regular frequency
  if ((curtimemillis - timeLastStatusMsg) > DELAY_STATUSMSG)
  {
    #ifdef debug_info
      Serial.println();
      Serial.print("Time [");
      Serial.print(millis());
      Serial.print("] State: ");
      Serial.println(contactorsstate);
    #endif

    timeLastStatusMsg = curtimemillis;
  }

  // CONTROLLER CAN MESSAGES 
  struct can_frame outframe;

//  // catch the current time
//  curtimemillis = millis();

  // Send out a Battery Contactor/BMS status message every 1 sec  
  if ((curtimemillis - timeBatteryStatus)> DELAY_STATUSMSG)
  {
    // reset the timer
    timeBatteryStatus = curtimemillis;

    #ifdef debug 
      Serial.println("Output: Contactor states and battery voltages");
    #endif

    // Test if voltage warning message needed
    bool voltwarn = false;
    uint8_t voltcode = 0;
    uint16_t warnvolt = 0;

    // Send Battery Status Update message
    outframe.can_id = BattStatFrameID;
    outframe.can_dlc = 8;            // Data payload 8 bytes
    outframe.data[0] = contactorsstate;   // Status of the HV contactors
    outframe.data[1] = bmsstate;          // Status of the BMS communications
    outframe.data[2] = lowByte(totalpackvolts);
    outframe.data[3] = highByte(totalpackvolts);
    outframe.data[4] = lowByte(mincellvolts);
    outframe.data[5] = highByte(mincellvolts);
    outframe.data[6] = lowByte(maxcellvolts);
    outframe.data[7] = highByte(maxcellvolts);
    sendMessage(can1, outframe);

    // Test Total Pack Voltage
    voltwarn = false;
    if (totalpackvolts != 0)
    {
      if (totalpackvolts < PACK_LOW_VOLTAGE){voltcode = 0x0F;warnvolt=totalpackvolts;voltwarn=true;}
      if (totalpackvolts < PACK_LOW_LOW_VOLTAGE){voltcode = 0x00;warnvolt=totalpackvolts;voltwarn=true;}
      if (totalpackvolts > PACK_HIGH_VOLTAGE){voltcode = 0xF0;warnvolt=totalpackvolts;voltwarn=true;}
      if (totalpackvolts > PACK_HIGH_HIGH_VOLTAGE){voltcode = 0xFF;warnvolt=totalpackvolts;voltwarn=true;}
    }

    if (voltwarn)
    {
      // Send Battery Pack Voltage WARNING message
      outframe.can_id = BattWarnVFrameID;
      outframe.can_dlc = 8;            // Data payload 8 bytes
      outframe.data[0] = voltcode;    // FF = HH, F0 = H, 0F = L, 00 = LL
      outframe.data[1] = lowByte(warnvolt);
      outframe.data[2] = highByte(warnvolt);
      outframe.data[3] = 1;       // 1 = Pack
      outframe.data[4] = 0;
      outframe.data[5] = 0;
      outframe.data[6] = 0;
      outframe.data[7] = 0;
      sendMessage(can1, outframe);
    }

    // Test Min and Max cell voltages
    voltwarn = false;
    if (totalpackvolts != 0)
    {
      if (mincellvolts < CELL_LOW_VOLTAGE){voltcode = 0x0F;warnvolt=mincellvolts;voltwarn=true;}
      if (mincellvolts < CELL_LOW_LOW_VOLTAGE){voltcode = 0x00;warnvolt=mincellvolts;voltwarn=true;}
      if (maxcellvolts > CELL_HIGH_VOLTAGE){voltcode = 0xF0;warnvolt=maxcellvolts;voltwarn=true;}
      if (maxcellvolts > CELL_HIGH_HIGH_VOLTAGE){voltcode = 0xFF;warnvolt=maxcellvolts;voltwarn=true;}
    }

    if (voltwarn)
    {
      // Send Battery Pack Voltage WARNING message
      outframe.can_id = BattWarnVFrameID;
      outframe.can_dlc = 8;            // Data payload 8 bytes
      outframe.data[0] = voltcode;    // FF = HH, F0 = H, 0F = L, 00 = LL
      outframe.data[1] = lowByte(warnvolt);
      outframe.data[2] = highByte(warnvolt);
      outframe.data[3] = 0;       // 0 = Cell
      outframe.data[4] = 0;
      outframe.data[5] = 0;
      outframe.data[6] = 0;
      outframe.data[7] = 0;
      sendMessage(can1, outframe);
    }
  }

//  // catch the current time
//  curtimemillis = millis();

  // Send out a Battery module temperature message
  if ((curtimemillis - timeModTempsStatus) > DELAY_MODFMSG)
  {
    // reset the timer
    timeModTempsStatus = curtimemillis;

    #ifdef debug 
      Serial.println("Output: Temperature status");
    #endif
    
    // Send Battery Module Temps message, units are 1=1 degrees
    outframe.can_id = BattModFFrameID;
    outframe.can_dlc = 8;            // Data payload 8 bytes
    outframe.data[0] = moduletemps[0];
    outframe.data[1] = moduletemps[1];
    outframe.data[2] = moduletemps[2];
    outframe.data[3] = moduletemps[3];
    outframe.data[4] = moduletemps[4];
    outframe.data[5] = moduletemps[5];
    outframe.data[6] = moduletemps[6];
    outframe.data[7] = moduletemps[7];
    sendMessage(can1, outframe);
    
    // Test if temperature warning message needed
    bool tempwarn = false;
    uint8_t warncode = 0;
    uint8_t warntemp = 0;
    for (int j = 0; j < 8; j++)
    {
      tempwarn = false;
      if ((moduletemps[j]) != 0)
      {
        if (moduletemps[j]<PACK_LOW_TEMP){warncode = 0x0F;warntemp=moduletemps[j];tempwarn=true;}
        if (moduletemps[j]<PACK_LOW_LOW_TEMP){warncode = 0x00;warntemp=moduletemps[j];tempwarn=true;}
        if (moduletemps[j]>PACK_HIGH_TEMP){warncode = 0xF0;warntemp=moduletemps[j];tempwarn=true;}
        if (moduletemps[j]>PACK_HIGH_HIGH_TEMP){warncode = 0xFF;warntemp=moduletemps[j];tempwarn=true;}
      }

      if (tempwarn)
      {
        // Send Battery Pack Temps WARNING message
        outframe.can_id = BattWarnTFrameID;
        outframe.can_dlc = 8;            // Data payload 8 bytes
        outframe.data[0] = warncode;    // FF = HH, F0 = H, 0F = L, 00 = LL
        outframe.data[1] = warntemp;
        outframe.data[2] = 0;
        outframe.data[3] = 1;       // Modules
        outframe.data[4] = j;
        outframe.data[5] = 0;
        outframe.data[6] = 0;
        outframe.data[7] = 0;
        sendMessage(can1, outframe);
      }
    }
  }

//  // catch the current time
//  curtimemillis = millis();

  // Send out a Battery pack temperature message
  if ((curtimemillis - timePckTempsStatus) > DELAY_PCKFMSG)
  {
    // reset the timer
    timePckTempsStatus = curtimemillis;

    #ifdef debug 
      Serial.println("Output: Battery pack temperatures and voltage trend");
    #endif
    
    // Calculate the rate of voltage decline
    unsigned long timeSinceVoltageChange;
    if (totalpackvolts != priorpackvolts)
    {
      // Get the time slice
      timeSinceVoltageChange = curtimemillis - timeSincePackVChange;
      // Update the timestamp for the voltage change
      timeSincePackVChange = millis();
    }
    float minsSinceVoltageChange;
    float voltchangerate;
    minsSinceVoltageChange = timeSinceVoltageChange / 600;  // 100ths of a minute
    voltchangerate = (totalpackvolts - priorpackvolts) / minsSinceVoltageChange;  // Volts per 100th of a minute
    priorpackvolts = totalpackvolts;
    int16_t voltchangerate2 = voltchangerate;
        
    // Send Battery Pack Temps message
    outframe.can_id = BattPckFFrameID;
    outframe.can_dlc = 8;            // Data payload 8 bytes
    outframe.data[0] = 0;
    if (packtemps[0] > 0x0F){outframe.data[0] = packtemps[0];} //lowByte(packtemps[0]);
    outframe.data[1] = 0; // highByte(packtemps[0]);
    outframe.data[2] = 0;
    if (packtemps[1] > 0x0F){outframe.data[2] = packtemps[1];} //lowByte(packtemps[1]);
    outframe.data[3] = 0; //highByte(packtemps[1]);
    outframe.data[4] = lowByte(voltchangerate2);
    outframe.data[5] = highByte(voltchangerate2);
    outframe.data[6] = 0;
    outframe.data[7] = 0;
    sendMessage(can1, outframe);
    
    /*
     * Pack Temperatures are not being reported from the BMS as they were in early runs
     * no idea why - may be a 12v/5v issue on the enable lines
     * 
    // Test if a pack temp warning message warranted
    if (packtemps[0] != 0 || packtemps[1] != 0)
    {
    
      if ((min(packtemps[0],packtemps[1]) < PACK_LOW_TEMP) || (max(packtemps[0],packtemps[1]) > PACK_HIGH_TEMP))
      {
          byte warncode = 0;
          byte packhalf = 0;
          uint8_t warntemp = 0;
          if (packtemps[0] < PACK_LOW_TEMP){warncode = 0x0F;packhalf = 0;warntemp=packtemps[0];}
          if (packtemps[1] < PACK_LOW_TEMP){warncode = 0x0F;packhalf = 1;warntemp=packtemps[1];}
          if (packtemps[0] < PACK_LOW_LOW_TEMP){warncode = 0x00;packhalf = 0;warntemp=packtemps[0];}
          if (packtemps[1] < PACK_LOW_LOW_TEMP){warncode = 0x00;packhalf = 1;warntemp=packtemps[1];}
          if (packtemps[0] > PACK_HIGH_TEMP){warncode = 0xF0;packhalf = 0;warntemp=packtemps[0];}
          if (packtemps[1] > PACK_HIGH_TEMP){warncode = 0xF0;packhalf = 1;warntemp=packtemps[1];}
          if (packtemps[0] > PACK_HIGH_HIGH_TEMP){warncode = 0xFF;packhalf = 0;warntemp=packtemps[0];}
          if (packtemps[1] > PACK_HIGH_HIGH_TEMP){warncode = 0xFF;packhalf = 1;warntemp=packtemps[1];}
  
          // Send Battery Pack Temps WARNING message
          outframe.can_id = BattWarnTFrameID;
          outframe.can_dlc = 8;            // Data payload 8 bytes
          outframe.data[0] = warncode;    // FF, F0, 0F, FF
          outframe.data[1] = warntemp; //lowByte(packtemps[0]);
          outframe.data[2] = 0; //highByte(packtemps[0]);
          outframe.data[3] = 0;       // Pack
          outframe.data[4] = packhalf;
          outframe.data[5] = 0;
          outframe.data[6] = 0;
          outframe.data[7] = 0;
          sendMessage(can1, outframe);
      }
    }
    */
  }

  // Check if individual voltages are being transmitted (two per cycle) and if so send the next one
  if (cellvoltmodule != 0 && cellvolttriplet !=0)
  {  
    #ifdef debug
      //if (cellvoltmodule == 1 && cellvolttriplet == 1)
      //{
      //  Serial.println("Output: Battery cell voltages");
      //}
    #endif
    
    // Send Battery Individual Cell Voltage
    outframe.can_id = BattVoltFrameID;
    outframe.can_dlc = 8;            // Data payload 8 bytes
    outframe.data[0] = cellvoltmodule;
    outframe.data[1] = cellvolttriplet;
    outframe.data[2] = lowByte(cellvolts[cellvolttriplet-1][cellvoltmodule-1]);
    outframe.data[3] = highByte(cellvolts[cellvolttriplet-1][cellvoltmodule-1]);
    outframe.data[4] = lowByte(cellvolts[cellvolttriplet][cellvoltmodule-1]);
    outframe.data[5] = highByte(cellvolts[cellvolttriplet][cellvoltmodule-1]);
    outframe.data[6] = 0;
    outframe.data[7] = 0;
    sendMessage(can1, outframe);

    // Progress to next one OR finish transmission
    cellvolttriplet = cellvolttriplet + 2;
    if (cellvolttriplet > 12)
    {    
      cellvolttriplet = 1;
      cellvoltmodule++;
      if (cellvoltmodule > 8)
      {
        cellvoltmodule = 0;
        cellvolttriplet = 0; 
      }
    }
  }

  // Reset the watchdog
  wdt_reset();
}

// Check for Instructions on the TesLorean CAN bus
void canTESLOREANdecode(can_frame & frame)
{
  switch (frame.can_id)
  {
    case 0x132: // BattConnFrameID : Battery HV Activation/Deactivation

      #ifdef debug 
        Serial.println("TesL: 0x132 - Battery HV Activate/Deactivate");
      #endif
    
      // 1 = Connect, 0 = Disconnect
      if (frame.data[0] == 0)
      {
        // Set the contactors required state to deactivated
        contactorsreqdstate = CONTACTORSOPEN;
      }
      if (frame.data[0] == 1)
      {
        // Set the contactors required state to activated
        contactorsreqdstate = CONTACTORSCLOSED;
      }
      break;

    case 0x142: // BMSConnFrameID : BMS Activation/Deactivation

      #ifdef debug 
        Serial.println("TesL: 0x142 - BMS Activate/Deactivate");
      #endif
      
      // 1 = Start, 0 = Stop
      if (frame.data[0] == 0)
      {
        // Activate the battery data communications
        bmsreqdstate = BMSCOMMSOFF;
      }
      if (frame.data[0] == 1)
      {
        // Deactivate the battery data communications
        bmsreqdstate = BMSCOMMSON;
      }
      break;

    case 0x151: // BattVoltReqFrameID : Start transmmission of individual voltages

      #ifdef debug 
        Serial.println("TesL: 0x151 - Start transmission of cell voltages");
      #endif
    
      // Set the counters that signal transmission underway
      cellvoltmodule = 1;
      cellvolttriplet = 1;
      break;

    default:
      // if nothing else matches, do the default
      #ifdef debug 
        Serial.print("TesL: ");
        Serial.print(frame.can_id, HEX);
        Serial.println(" - frame received");
      #endif

      break;

   }
}

// Output a number of bytes from the frame
void outputframe(can_frame & frame, uint8_t fnums)
{
  for (uint8_t f = 0; f < fnums; f++)
  {
    if (f != 0){
      Serial.print(",");
    }
    Serial.print(frame.data[(f)],HEX);
  }
}

// Decode messages from the Chevy Spark BMS
void canBATTERYdecode(can_frame & frame)
{
  // Temp variables
  uint8_t by0, by1, by2, by3, by4, by6, by7;

  switch (frame.can_id)
  {
    case 0x0302: // Bank temperatures (2 banks per module, 4 modules in pack)

      #ifdef debug 
        Serial.print("Batt: 0x302 - Bank temperatures : ");
        outputframe(frame, 8);
        Serial.println();
      #endif
    
      // Get data bytes
      by0 = (frame.data[0] & 0x01);             // bit 0 from by0
      by7 = ((frame.data[7] & 0x20) >> 5);      // bit 5 from by7 -> shifted 5 times

      // Test that the frame contains data
      if (by0 == 1)
      {
        // by7 indicates if this frame contains 2 or 6 data bytes
        for (int framenum = 1; framenum < (by7 == 0 ? 7 : 3) ; framenum++)
        {
          // by7 indicates which bank is reporting, which other bytes to reference, also temp is 0.5oC, offset 40C
          moduletemps[(by7 == 0 ? (framenum - 1) : (framenum + 5))] = frame.data[framenum];
        }
        
      } // frame contains data

      break;

    case 0x0216: // Total pack voltage

      #ifdef debug 
        Serial.print("Batt: 0x216 - Total pack voltage : ");
        outputframe(frame, 8);  // by1 & 2
        Serial.println();
      #endif
      
      // Get bytes
      by1 = (frame.data[1] & 0x1F);     // lower 5 bits of by1
      by2 = (frame.data[2]);            // all bits of by2
      totalpackvolts = ((((uint16_t)by1 << 8) + by2) >> 3);      // Value is MSB by1 + LSB by2 / 8

      #ifdef debug 
        Serial.print("Total Pack Volts : ");
        Serial.println(totalpackvolts, DEC);
      #endif

      break;

    case 0x0460: // Pack halves temperatures

      #ifdef debug 
        Serial.print("Batt: 0x460 - Pack half temperatures : ");
        outputframe(frame, 8);
        Serial.println();
      #endif
      
      // Get bytes ; tempF = value / 2 - 40
      // Changed to -40 to match with ambient temps
      by1 = frame.data[1];    //((frame.data[1] >> 1) - 40);        // by1, div by 2, subtract 40
      by3 = frame.data[3];    //((frame.data[3] >> 1) - 40);        // by3, div by 2, subtract 40
      packtemps[0] = by1;
      packtemps[1] = by3;

      #ifdef debug 
        Serial.print("Pack Temp 1 : ");
        Serial.println(packtemps[0], HEX);
        Serial.print("Pack Temp 2 : ");
        Serial.println(packtemps[1], HEX);
      #endif

      break;

    default:
      // do nothing
      break;
  }

  // Check for the set of FrameIDs that carry cell voltages
  if (frame.can_id == 0x0200 || frame.can_id == 0x0202 || frame.can_id == 0x0204 || frame.can_id == 0x0206 || frame.can_id == 0x0208)
  {

    #ifdef debug 
      Serial.print("Batt: ");
      Serial.print(frame.can_id,HEX);
      Serial.print(" - Cell voltages : ");
      outputframe(frame, 8);
      Serial.println();
    #endif

    // Get the group number
    byte gpnum = 0;
    if (frame.can_id == 0x0200) {
      gpnum = 0;
    }
    if (frame.can_id == 0x0202) {
      gpnum = 1;
    }
    if (frame.can_id == 0x0204) {
      gpnum = 2;
    }
    if (frame.can_id == 0x0206) {
      gpnum = 3;
    }
    if (frame.can_id == 0x0208) {
      gpnum = 4;
    }
    
    // :)
    // Alternative 1 liner to extract the group number from the frame id
    // gpnum = (((uint16_t)frame.can_id & 0x000F) >> 1)

    // Get the module number
    byte mdnum = 0;
    mdnum = ((frame.data[6] & 0xE0) >> 5);       // get top 3 bits of by6

    // Get the voltages
    uint16_t vt1 = (uint16_t)((frame.data[0] & 0x1F) << 8) + (frame.data[1]);   //  MSB last 5 bits of 0, LSB all 8 bits of 1
    uint16_t vt2 = (uint16_t)((frame.data[2] & 0x1F) << 8) + (frame.data[3]);   //  MSB last 5 bits of 2, LSB all 8 bits of 3
    uint16_t vt3 = (((uint16_t)(frame.data[4] << 8) + (frame.data[5] & 0xF8)) >> 3);    //  MSB all 8 bits of 4, LSB first 5 bits of 5
    
    // Convert to 100th volt (divide by 16)         ???? Are voltages reported to 1600th of a volt ????
    vt1 = (vt1 >> 4);
    vt2 = (vt2 >> 4);
    vt3 = (vt3 >> 4);

    // Check for a new min cell voltage
    uint16_t mintriplet = 0;
    mintriplet = min(vt1, min(vt2, vt3));       // get lowest voltage of the triplet
    // check if new lowest cell voltage found
    if (mincellvolts == 0)
    {
      mincellvolts = mintriplet;
    }
    else
    {
      if (mintriplet < mincellvolts) {mincellvolts = mintriplet;}
    }

    // Check for a new max cell voltage
    uint16_t maxtriplet = 0;
    maxtriplet = max(vt1, max(vt2, vt3));       // get highest voltage of the triplet
    // check if new highest cell voltage found
    if (maxcellvolts == 0)
    {
      maxcellvolts = maxtriplet;
    }
    else
    {
      if (maxtriplet > maxcellvolts) {maxcellvolts = maxtriplet;}
    }

    byte c1;
    byte c2;
    byte c3;

    if (gpnum != 4) {

      // Get the cell numbers from the help pre-defined array
      byte c1 = startcellnum[gpnum];
      byte c2 = c1 + 1;
      byte c3 = c2 + 1;

      // Save the voltages
      cellvolts[c1][mdnum] = vt1;
      cellvolts[c2][mdnum] = vt2;
      cellvolts[c3][mdnum] = vt3;
      
      #ifdef debug 
        Serial.print("Batt: Module ");
        Serial.print(mdnum,DEC);
        Serial.print(" [Cell ");
        Serial.print(c1,DEC);
        Serial.print("] ");
        Serial.print(vt1,DEC);
        Serial.print(" [Cell ");
        Serial.print(c2,DEC);
        Serial.print("] ");
        Serial.print(vt2,DEC);
        Serial.print(" [Cell ");
        Serial.print(c3,DEC);
        Serial.print("] ");
        Serial.print(vt3,DEC);
        Serial.println(" V");
      #endif
    }
    else            // gpnum = 4 // Code for the odd frame covering the rest of the voltages
    {
      // Get the exception module numbers from the pre-defined helper array
      byte m1 = group4mods[mdnum][0];
      byte m2 = group4mods[mdnum][1];
      byte m3 = group4mods[mdnum][2];

      // Get the cell numbers
      byte cs = startcellnum[gpnum];

      // Test the first result
      if (m1 != 9)
      {
        if (m1 != m2)
        {
          cs += 1;
        }
        cellvolts[cs][m1] = vt1;
        c1 = cs;
        cs += 1;
      }
      // Test the second result
      if (m2 != 9)
      {
        if (m2 != m1)
        {
          cs = startcellnum[gpnum];
        }
        cellvolts[cs][m2] = vt2;
        c2 = cs;
        cs += 1;
      }
      // Test the third result
      if (m3 != 9)
      {
        if (m3 != m2)
        {
          cs = startcellnum[gpnum];
        }
        cellvolts[cs][m3] = vt3;
        c3 = cs;
      }

      #ifdef debug 
        Serial.print("Batt: ");
        if (m1 != 9)
        {
          Serial.print(" Module ");
          Serial.print(m1,DEC);
          Serial.print(" [Cell ");
          Serial.print(c1,DEC);
          Serial.print("] ");
          Serial.print(vt1,DEC);
        }
        if (m2 != 9)
        {
          Serial.print(" Module ");
          Serial.print(m2,DEC);
          Serial.print(" [Cell ");
          Serial.print(c2,DEC);
          Serial.print("] ");
          Serial.print(vt2,DEC);
        }
        if (m2 != 9)
        {
          Serial.print(" Module ");
          Serial.print(m3,DEC);
          Serial.print(" [Cell ");
          Serial.print(c3,DEC);
          Serial.print("] ");
          Serial.print(vt3,DEC);
          Serial.println(" V");
        }
      #endif

    }
  }     // end of voltage frame check

  // Check for the set of FrameIDs that carry the unknown data
  if (frame.can_id == 0x0260 || frame.can_id == 0x0262 || frame.can_id == 0x0270 || frame.can_id == 0x0272 || frame.can_id == 0x0274)
  {

    #ifdef debug 
      Serial.println("Batt: 0x260/262/270/272/274 - Unknown data");
      Serial.print("Batt: ");
      Serial.print(frame.can_id,HEX);
      Serial.print(" - Unknown : ");
      outputframe(frame, 8);
      Serial.println();
    #endif

    // Get the counter - top 5 bits of by1
    byte ctr = ((frame.data[0] & 0xF8) >> 3);

    // Get the number of datasets to extract
    byte datacnt = 0;
    if (frame.can_id == 0x0260) {
      datacnt = 5;
    }
    if (frame.can_id == 0x0262) {
      datacnt = 3;
    }
    if (frame.can_id == 0x0270) {
      datacnt = 8;
    }
    if (frame.can_id == 0x0272) {
      datacnt = 8;
    }
    if (frame.can_id == 0x0274) {
      datacnt = 2;
    }

    // Cycle through the count and extract 3 bits from each
    for (int datainx = 0; datainx < datacnt; datainx++)
    {
      // Extract the bits
      byte b1 = (frame.data[datainx] & 0x04) >> 2;
      byte b2 = (frame.data[datainx] & 0x02) >> 1;
      byte b3 = (frame.data[datainx] & 0x01);

      // Assign to the appropriate maxtrix
      if (frame.can_id == 0x0260) {
        frame260[datainx][0] = b1;
        frame260[datainx][1] = b2;
        frame260[datainx][2] = b3;
      }
      if (frame.can_id == 0x0262) {
        frame262[datainx][0] = b1;
        frame262[datainx][1] = b2;
        frame262[datainx][2] = b3;
      }
      if (frame.can_id == 0x0270) {
        frame270[datainx][0] = b1;
        frame270[datainx][1] = b2;
        frame270[datainx][2] = b3;
      }
      if (frame.can_id == 0x0272) {
        frame272[datainx][0] = b1;
        frame272[datainx][1] = b2;
        frame272[datainx][2] = b3;
      }
      if (frame.can_id == 0x0274) {
        frame274[datainx][0] = b1;
        frame274[datainx][1] = b2;
        frame274[datainx][2] = b3;
      }
    }
  }   // end of can1decode for CAN frames from the battery
  
}
