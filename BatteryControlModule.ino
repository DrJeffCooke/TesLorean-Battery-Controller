/*
  TesLorean Battery Control Module
  2018
  Jeff Cooke

  Based on Gen2 Charger Control Program by T de Bree, D.Maguire 2017-2018
  Additional work by C. Kidder
  
  Coding ToDos
  
  Coding Notes
  - Temps are reported in F or C?
  
*/

#include <mcp2515.h>
#include <SPI.h>
#include "config.h"

// Create the CAN objects, specific the 'Chip Select' line
MCP2515 can0(10);   // TesLorean CAN bus - CS on line 10
MCP2515 can1(9);    // Battery CAN Bus - CS on line 9

//****** DEBUG VARIABLES
#define debug 1

//********* TIME SINCE VARIABLES *************
unsigned long timeContactorStatus = 0;
unsigned long timeBatteryStatus = 0;
unsigned long timeModTempsStatus = 0;
unsigned long timePckTempsStatus = 0;

//********* STATE VARIABLES ******************
int contactorsstate;      // Current state of the contactors
int contactorsreqdstate;  // Requested state of the contactors
int bmsstate;             // Current state of the BMS communications
int bmsreqdstate;         // Requested state of the BMS communications
int cellvoltmodule;       // 1-8 : Module to send volts status for 0x150
int cellvolttriplet;      // 1-14 : Cell triplet to send volts status for 0x150

//********* GENERAL VARIABLES ******************
int incomingByte = 0;
unsigned long timeLastStatusMsg = 0;

//********* CAN IDs ******************
int BattConnFrameID = 0x140;
int BMSConnFrameID = 0x142;
int BattStatFrameID = 0x141;
int BattModFFrameID = 0x123;
int BattPckFFrameID = 0x120;
int BattVoltFrameID = 0x150;
int BattVoltReqFrameID = 0x151;
int BattWarnVFrameID = 0x0110;      // Voltage warning
int BattWarnTFrameID = 0x0111;      // Temperature warning
int BattWarnIFrameID = 0x0112;      // Isolation warning

//********* HELPER ARRAYS ************
byte group4mods[8][3] = {{0, 0, 1}, {1, 2, 2}, {3, 3, 4}, {4, 5, 5}, {6, 6, 7}, {7, 9, 9}, {9, 9, 9}, {9, 9, 9}}; // Modules for Group 4 voltages
byte startcellnum[5] = {0, 3, 6, 9, 12}; // cell voltage frames, starting triple voltage cell num

//********* DATA FROM BMS ******************
uint16_t moduletemps[8] = {0, 0, 0, 0, 0, 0, 0, 0};       // 1 = 0.5F
uint16_t totalpackvolts = 0;                              // 1 = 1V
uint16_t priorpackvolts = 0;                             // Total voltage at prior reporting
uint16_t packtemps[2] = {0, 0};                           // 1 = 1F
uint16_t cellvolts[14][8];                                // 14 cells in each of 8 modules, 1 = 1/100th V
uint16_t mincellvolts = 0;                                // lowest individual cell voltage, 1 = 1/100th V
uint16_t maxcellvolts = 0;                                // highest individual cell voltage, 1 = 1/100th V
byte frame260[5][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
byte frame262[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
byte frame270[8][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
byte frame272[8][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
byte frame274[2][3] = {{0, 0, 0}, {0, 0, 0}};

void setup()
{
  #ifdef debug 
    Serial.println("Initializing Digital Pins for Relay control");
  #endif
  
  //// INITIALIZE RELAY PINS
  digitalWrite(Relay_X358_7, RELAY_OFF); // X358-7 K3: Main -VE [G]
  digitalWrite(Relay_X358_8, RELAY_OFF); // X358-8 Multifunction contactor [E]
  digitalWrite(Relay_X358_9, RELAY_OFF); // X358-9 K1: Precharge contactor for Main [L]
  digitalWrite(Relay_X358_10, RELAY_OFF); // X358-10 K2: Main +VE contactor [F]
  digitalWrite(Relay_X357_4, RELAY_OFF); // X357-3 to 12v accessory wake up
  digitalWrite(Relay_X357_5, RELAY_OFF); // X357-14 to 12v HV energy mgt communication enable

  //// SETUP OUTPUT LINES
  pinMode(Relay_X358_7, OUTPUT);
  pinMode(Relay_X358_8, OUTPUT);
  pinMode(Relay_X358_9, OUTPUT);
  pinMode(Relay_X358_10, OUTPUT);
  pinMode(Relay_X357_4, OUTPUT);
  pinMode(Relay_X357_5, OUTPUT);

  //// SERIAL PORT
  #ifdef debug 
    Serial.println("Initializing serial port (115200 baud)");
  #endif
  // init the serial port - for status/debug
  while (!Serial);
  Serial.begin(115200);

  // init the SPI communications
  SPI.begin();

  //// TIMERS AND INTERRUPTS
  //  Timer3.attachInterrupt(Controller_CAN_Messages).start(90000); // charger messages every 100ms

  // Startup CAN  TesLorean bus
  can0.reset();
  can0.setBitrate(CAN_500KBPS);
  can0.setNormalMode();
  #ifdef debug
    Serial.println("TesLorean CANbus initialized (500 kbps)");
  #endif

  // Startup CAN  Battery bus
  can1.reset();
  can1.setBitrate(CAN_500KBPS);
  can1.setNormalMode();
  #ifdef debug
    Serial.println("Battery CANbus initialized (500 kbps)");
  #endif

}

void loop()
{
  // Define CAN Frame variable structure
  struct can_frame incoming;

  // Check for instructions on the TesLorean CAN bus
  if (can0.readMessage(&incoming) == MCP2515::ERROR_OK)
  {
    #ifdef debug 
      Serial.println("CAN frame available on TesLorean bus - decoding");
    #endif
    can0decode(incoming);
  }

  // Check for data from the Chevy Spark Battery CAN bus
  if (can1.readMessage(&incoming) == MCP2515::ERROR_OK)
  {
    #ifdef debug 
      Serial.println("CAN frame available on Battery bus - decoding");
    #endif
    can1decode(incoming);
  }

  // Selectively send out status and informational messages
  Controller_CAN_Messages;

  // Check the Serial Bus for commands
  if (Serial.available())
  {
    incomingByte = Serial.read(); // read the incoming byte:

    switch (incomingByte)
    {
      case 'd'://d for display
        Serial.println();
        Serial.print("Status : ");
        Serial.println(incomingByte);
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
      switch (contactorsstate)
      {
        case CONTACTORSOPEN:

          #ifdef debug 
            Serial.println("Prior status: CONTACTORSOPEN; New status: CONTACTORSNEGMUL");
          #endif

          // Switch on the -VE and Multifunction Contactors
          digitalWrite(Relay_X358_7, RELAY_ON);  // Enable Main -VE Contactor
          digitalWrite(Relay_X358_8, RELAY_ON);  // Enable Multifunction Contactor

          // Immediately change to state CONTACTORNEGMUL
          contactorsstate = CONTACTORSNEGMUL;

          // Start the contactorsstatetimer
          timeContactorStatus = millis();

          break;

        case CONTACTORSNEGMUL:
          // Check that the require time has passed in this state
          if (timeContactorStatus <  (millis() - DELAY_INTERRELAY))
          {
            #ifdef debug 
              Serial.println("Prior status: CONTACTORSNEGMUL; New status: CONTACTORSPRECRG");
            #endif

            // Switch on the PreCharge Contactor
            digitalWrite(Relay_X358_9, RELAY_ON);  // Enable Pre-charger +VE Contactor

            // Change to state CONTACTORSPRECRG
            contactorsstate = CONTACTORSPRECRG;

            // Restart the contactorsstatetimer
            timeContactorStatus = millis();
          }
          break;

        case CONTACTORSPRECRG:
          // Check that the require time has passed in this state
          if (timeContactorStatus <  (millis() - DELAY_PRECHARGE))
          {
            #ifdef debug 
              Serial.println("Prior status: CONTACTORSPRECRG; New status: CONTACTORSPREPOS");
            #endif

            // Switch on the Positive Contactor
            digitalWrite(Relay_X358_10, RELAY_ON);  // Enable Main +VE Contactor

            // Change to state CONTACTORSPREPOS
            contactorsstate = CONTACTORSPREPOS;

            // Restart the contactorsstatetimer
            timeContactorStatus = millis();
          }
          break;

        case CONTACTORSPREPOS:
          // Check that the require time has passed in this state
          if (timeContactorStatus <  (millis() - DELAY_INTERRELAY))
          {
            #ifdef debug 
              Serial.println("Prior status: CONTACTORSPREPOS; New status: CONTACTORSCLOSED");
            #endif

            // Switch off the PreCharge Contactor
            digitalWrite(Relay_X358_9, RELAY_OFF);  // Disable Pre-charger +VE Contactor

            // Change to state CONTACTORSCLOSED
            contactorsstate = CONTACTORSCLOSED;

            // Restart the contactorsstatetimer
            timeContactorStatus = millis();
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
      #ifdef debug 
        Serial.println("New status: CONTACTORSOPEN");
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
      #ifdef debug 
        Serial.println("BMS communications: ON");
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
      #ifdef debug 
        Serial.println("BMS communications: OFF");
      #endif

      // Disable the signal lines
      digitalWrite(Relay_X357_5, RELAY_OFF); // X357-14 HV communications enable
      digitalWrite(Relay_X357_4, RELAY_OFF); // X357-3 Accessory wake-up

      // Update the status
      bmsstate = BMSCOMMSOFF;
    }
  }

  // Output a debug status message on regular frequency
  #ifdef debug 
  if (timeLastStatusMsg <  (millis() - DELAY_STATUSMSG))
  {
    timeLastStatusMsg = millis();

    Serial.println();
    Serial.print("Time [");
    Serial.print(millis());
    Serial.print("] Contactor State: ");
    Serial.println(contactorsstate);
  }
  #endif
}

// Check for Instructions on the TesLorean CAN bus
void can0decode(can_frame & frame)
{
  switch (frame.can_id)
  {
    case 0x140: // BattConnFrameID : Battery HV Activation/Deactivation

      #ifdef debug 
        Serial.println("TesL: 0x140 - Battery HV Activate/Deactivate");
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
        bmsreqdstate = BMSCOMMSON;
      }
      if (frame.data[0] == 1)
      {
        // Deactivate the battery data communications
        bmsreqdstate = BMSCOMMSOFF;
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
      break;
  }
}

// Output selected bytes from the CANbus frame data (used in debugging)
// if fnums = 1, outputs data byte 0 (of 0..7)
// if fnums = 5, outputs data byte 0 and 3 (of 0..7)
void outframe(can_frame & frame, uint8_t fnums)
{
  for (uint8_t f = 1; f < 129; f=f*2)
  {
    if ((fnums & f) > 0)
    {
      Serial.print("f");
      Serial.print((f-1),DEC);
      Serial.print("=");
      Serial.print(frame.data[(f-1)],HEX);
      Serial.print(" ");
    }
  }
}

// Decode messages from the Chevy Spark BMS
void can1decode(can_frame & frame)
{
  // Temp variables
  byte by0, by1, by2, by3, by4, by6, by7;

  switch (frame.can_id)
  {
    case 0x0302: // Bank temperatures (2 banks per module, 4 modules in pack)

      #ifdef debug 
        Serial.print("Batt: 0x302 - Bank temperatures : ");
        outframe(frame, 0x0F);
        Serial.println();
      #endif
    
      // Get data bytes
      by0 = ((frame.data[0] & 0x80) >> 7);      // b7 bit of by0, shift to b0 position
      by7 = ((frame.data[7] & 0x04) >> 2);      // b4 bit if by7, shift to b0 position 

      // Test that the frame contains data
      if (by0 == 1)
      {
        // by7 indicates if this frame contains 4 or 7 data bytes
        for (int framenum = 1; framenum < ((by7 == 1 ? 3 : 7) + 1) ; framenum++)
        {
          // by7 indicates which bank is reporting, which other bytes to reference, also temp is 0.5o
          moduletemps[(by7 == 0 ? (framenum - 1) : (framenum + 5))] = (frame.data[framenum] >> 1); // This temp is /2
        }

        #ifdef debug 
          Serial.print("Module Temps : ");
          for (int m = 0; m < 8; m++)
          {
            Serial.print(moduletemps[m],DEC);
            Serial.print(" ");
          }
          Serial.println();
        #endif
        
      }
      break;

    case 0x0216: // Total pack voltage

      #ifdef debug 
        Serial.print("Batt: 0x216 - Total pack voltage : ");
        outframe(frame, 0x06);  // by1 & 2
        Serial.println();
      #endif
      
      // Get bytes
      by1 = (frame.data[1] & 0x1F);     // lower 5 bits of by1
      by2 = (frame.data[2]);            // all bits of by2
      totalpackvolts = ((uint16_t)by1 << 8) + by2;      // Value is MSB by1 + LSB by2

      #ifdef debug 
        Serial.print("Total Pack Volts : ");
        Serial.println(totalpackvolts, DEC);
      #endif
      
      break;

    case 0x0460: // Pack halves temperatures

      #ifdef debug 
        Serial.print("Batt: 0x460 - Pack half temperatures : ");
        outframe(frame, (8+2));
        Serial.println();
      #endif
      
      // Get bytes ; tempF = value / 2 - 20
      by1 = ((frame.data[1] >> 1) - 20);        // by1, div by 2, subtract 20
      by3 = ((frame.data[3] >> 1) - 20);        // by3, div by 2, subtract 20
      packtemps[0] = by1;
      packtemps[1] = by3;

      #ifdef debug 
        Serial.print("Pack Temp 1 : ");
        Serial.println(packtemps[0], DEC);
        Serial.print("Pack Temp 2 : ");
        Serial.println(packtemps[1], DEC);
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
      outframe(frame, 0xFF);
      Serial.println();
    #endif

    // Get the group number
    byte gpnum = 0;
    if (frame.can_id = 0x0200) {
      gpnum = 0;
    }
    if (frame.can_id = 0x0202) {
      gpnum = 1;
    }
    if (frame.can_id = 0x0204) {
      gpnum = 2;
    }
    if (frame.can_id = 0x0206) {
      gpnum = 3;
    }
    if (frame.can_id = 0x0208) {
      gpnum = 4;
    }
    
    // :)
    // Alternative 1 liner to extract the group number from the frame id
    // gpnum = (((uint16_t)frame.can_id & 0x000F) >> 1)

    // Get the module number
    byte mdnum = 0;
    mdnum = frame.data[6] & 0x07;       // get lower 3 bits of by6

    // Get the voltages
    uint16_t vt1 = (uint16_t)((frame.data[0] & 0x1F) << 8) + (frame.data[1]);   //  MSB last 5 bits of 0, LSB all 8 bits of 1
    uint16_t vt2 = (uint16_t)((frame.data[2] & 0x1F) << 8) + (frame.data[3]);   //  MSB last 5 bits of 2, LSB all 8 bits of 3
    uint16_t vt3 = (uint16_t)(frame.data[4] >> 3) + ((frame.data[5] & 0xF8) >> 3);    //  MSB all 8 bits of 4, LSB first 5 bits of 5

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
        Serial.print(" Module ");
        Serial.print(m1,DEC);
        Serial.print(" [Cell ");
        Serial.print(c1,DEC);
        Serial.print("] ");
        Serial.print(vt1,DEC);
        Serial.print(" Module ");
        Serial.print(m2,DEC);
        Serial.print(" [Cell ");
        Serial.print(c2,DEC);
        Serial.print("] ");
        Serial.print(vt2,DEC);
        Serial.print(" Module ");
        Serial.print(m3,DEC);
        Serial.print(" [Cell ");
        Serial.print(c3,DEC);
        Serial.print("] ");
        Serial.print(vt3,DEC);
        Serial.println(" V");
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
      Serial.print(" - Cell voltages : ");
      outframe(frame, 0xFF);
      Serial.println();
    #endif

    // Get the counter - last 5 bits of by1
    byte ctr = frame.data[0] & 0x1F;

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
      byte b1 = (frame.data[datainx] & 0x10) >> 4;
      byte b2 = (frame.data[datainx] & 0x20) >> 5;
      byte b3 = (frame.data[datainx] & 0x40) >> 6;

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
    // end of can1decode for CAN frames from the battery
  }
}

void Controller_CAN_Messages()
{
  struct can_frame outframe;

  // Send out a Battery Contactor/BMS status message every 1sec
  if (timeBatteryStatus <  (millis() - DELAY_STATUSMSG))
  {
    #ifdef debug 
      Serial.println("Output: Contactor states and battery voltages");
    #endif
    
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
    can0.sendMessage(&outframe);
    
    // reset the timer
    timeBatteryStatus = millis();
  }

  // Send out a Battery module temperature message
  if (timeModTempsStatus <  (millis() - DELAY_MODFMSG))
  {
    #ifdef debug 
      Serial.println("Output: ???");
    #endif
    
    // Send Battery Module Temps message, units are 1=1 degrees
    outframe.can_id = BattModFFrameID;
    outframe.can_dlc = 8;            // Data payload 8 bytes
    outframe.data[0] = (uint8_t)(moduletemps[0] >> 1);
    outframe.data[1] = (uint8_t)(moduletemps[1] >> 1);
    outframe.data[2] = (uint8_t)(moduletemps[2] >> 1);
    outframe.data[3] = (uint8_t)(moduletemps[3] >> 1);
    outframe.data[4] = (uint8_t)(moduletemps[4] >> 1);
    outframe.data[5] = (uint8_t)(moduletemps[5] >> 1);
    outframe.data[6] = (uint8_t)(moduletemps[6] >> 1);
    outframe.data[7] = (uint8_t)(moduletemps[7] >> 1);
    can0.sendMessage(&outframe);
    
    // Test if temperature warning message needed
    bool tempwarn = false;
    byte warncode = 0;
    uint16_t warntemp = 0;
    for (int j = 0; j < 8; j++)
    {
      tempwarn = false;
      if ((moduletemps[j]>>1)<PACK_LOW_TEMP){warncode = 0x0F;warntemp=(moduletemps[j]>>1);tempwarn=true;}
      if ((moduletemps[j]>>1)<PACK_LOW_LOW_TEMP){warncode = 0x00;warntemp=(moduletemps[j]>>1);tempwarn=true;}
      if ((moduletemps[j]>>1)>PACK_HIGH_TEMP){warncode = 0xF0;warntemp=(moduletemps[j]>>1);tempwarn=true;}
      if ((moduletemps[j]>>1)<PACK_HIGH_HIGH_TEMP){warncode = 0xFF;warntemp=(moduletemps[j]>>1);tempwarn=true;}

      if (tempwarn)
      {
        // Send Battery Pack Temps WARNING message
        outframe.can_id = BattWarnTFrameID;
        outframe.can_dlc = 8;            // Data payload 8 bytes
        outframe.data[0] = warncode;    // FF = HH, F0 = H, 0F = L, 00 = LL
        outframe.data[1] = lowByte(warntemp);
        outframe.data[2] = highByte(warntemp);
        outframe.data[3] = 1;       // Modules
        outframe.data[4] = j;
        outframe.data[5] = 0;
        outframe.data[6] = 0;
        outframe.data[7] = 0;
        can0.sendMessage(&outframe);
      }
    }

    // reset the timer
    timeModTempsStatus = millis();
  }

  // Send out a Battery pack temperature message
  if (timePckTempsStatus <  (millis() - DELAY_PCKFMSG))
  {
    #ifdef debug 
      Serial.println("Output: Battery pack temperatures and voltage trend");
    #endif
    
    // Calculate the rate of voltage decline
    uint16_t deltatotalvolts = (totalpackvolts - priorpackvolts) * (60000/DELAY_PCKFMSG);
    
    // Send Battery Pack Temps message
    outframe.can_id = BattPckFFrameID;
    outframe.can_dlc = 8;            // Data payload 8 bytes
    outframe.data[0] = lowByte(packtemps[0]);
    outframe.data[1] = highByte(packtemps[0]);
    outframe.data[2] = lowByte(packtemps[1]);
    outframe.data[3] = highByte(packtemps[1]);
    outframe.data[4] = lowByte(deltatotalvolts);
    outframe.data[5] = highByte(deltatotalvolts);
    outframe.data[6] = 0;
    outframe.data[7] = 0;
    can0.sendMessage(&outframe);
    
    // Test if a pack temp warning message warranted
    if ((min(packtemps[0],packtemps[1]) < PACK_LOW_TEMP) || (max(packtemps[0],packtemps[1]) > PACK_HIGH_TEMP))
    {
        byte warncode = 0;
        byte packhalf = 0;
        uint16_t warntemp = 0;
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
        outframe.data[1] = lowByte(packtemps[0]);
        outframe.data[2] = highByte(packtemps[0]);
        outframe.data[3] = 0;       // Pack
        outframe.data[4] = packhalf;
        outframe.data[5] = 0;
        outframe.data[6] = 0;
        outframe.data[7] = 0;
        can0.sendMessage(&outframe);
    }

    // reset the timer
    timePckTempsStatus = millis();
  }

  // Check if individual voltages are being transmitted (two per cycle) and if so send the next one
  if (cellvoltmodule != 0 && cellvolttriplet !=0){
    
    #ifdef debug 
      Serial.println("Output: Battery cell triplet voltages");
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
    can0.sendMessage(&outframe);

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

}  // end of loop
