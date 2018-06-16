/*
 * Chevy Spark Battery Contactor Control
 * Main line & Charger line
 * note: main line is pre-charger protected
 * note: Charger line is not pre-charger connected (presumably the charger controller handled pre-charge spikes)
 * 
 * Pins on the X358 connector
 * 1 = Ground : HV Low interlock : Black
 * 2 = 12v : HV Hi Interlock : Purple
 *  Contactors work without the Interlock lines being powered, must be monitored outwith the battery module
 * 5 = Ground : for all Contactors : Black
 * 7 = Main -VE contactor  : Purple
 * 8 = Multifunction contactor : Grey Blue
 *    (linked to Pre-charge, Charger Batt+ve, and enables power to the battery heater
 * 9 = Pre-charge contactor (on main line) : White Pink
 * 10 = Main +VE contactor : Brown Green
 * 11 = Charger -VE contactor : White Purple
 * 12 = Changer +VE contactor : Yellow Purple
 * 14 = Aux heater : Purple
 * 
 * Main line On Sequence
 * 7 ON : Main -VE contactor enabled
 * 8 ON : Multifunction contactor enabled
 * 9 ON : Pre-charge concactor enabled
 * WAIT (until voltages on either side of pre-charger are roughly equivalent)
 * 10 ON : Main +VE contactor enabled
 * 9 OFF : Pre-charger contactor disabled
 * 
 * Main line Off Sequence
 * 10 OFF : Main +VE contactor disabled
 * 7 OFF : Main -VE contactor disabled
 * 8 OFF : Multifunction contactor disabled
 * 
 * Charger line On Sequence
 * 11 ON : Battery -VE contactor enabled
 * 8 ON : Multifunction contactor enabled
 * 12 ON : Battery +VE contactor enabled
 * 
 * Charger line Off Sequence
 * 12 OFF : Battery +VE contactor disabled
 * 8 OFF : Multifunction contcactor disabled
 * 11 OFF : Battery -VE contactor disabled
 * 
*/

/*-----( Import needed libraries )-----*/
/*-----( Declare Constants )-----*/
#define RELAY_ON 0
#define RELAY_OFF 1

#define DELAY_PRECHARGE 2000
#define DELAY_INTERRELAY 250

/*-----( Declare objects )-----*/
/*-----( Declare Variables )-----*/
#define Relay_X358_5  2  // X358-5 to Ground : Acts as ground for all contactors
#define Relay_X358_7  3  // X358-7 K3: Main -VE [G]
#define Relay_X358_8  4  // X358-8 Multifunction contactor [E]
#define Relay_X358_9  5  // X358-9 K1: Precharge contactor for Main [L]
#define Relay_X358_10  6  // X358-10 K2: Main +VE contactor [F]
#define Relay_X358_11  7  // X358-11 Charger -VE contactor [H]
#define Relay_X358_12  8  // X358-12 Charger +VE contactor [D]
#define Relay_Spare  9  // Not used for charging test

// Variables needed for serial function
unsigned long serialdata;
int inbyte;
unsigned int ison;

void setup()   /****** SETUP: RUNS ONCE ******/
{
//-------( Initialize Pins so relays are inactive at reset)----
  digitalWrite(Relay_X358_5, RELAY_OFF);
  digitalWrite(Relay_X358_7, RELAY_OFF);
  digitalWrite(Relay_X358_8, RELAY_OFF);
  digitalWrite(Relay_X358_9, RELAY_OFF);  
  digitalWrite(Relay_X358_10, RELAY_OFF);  
  digitalWrite(Relay_X358_11, RELAY_OFF);  
  digitalWrite(Relay_X358_12, RELAY_OFF);  
  digitalWrite(Relay_Spare, RELAY_OFF);  
  
//---( THEN set pins as outputs ) ---
  pinMode(Relay_X358_5, OUTPUT);   
  pinMode(Relay_X358_7, OUTPUT);  
  pinMode(Relay_X358_8, OUTPUT);  
  pinMode(Relay_X358_9, OUTPUT);    
  pinMode(Relay_X358_10, OUTPUT);    
  pinMode(Relay_X358_11, OUTPUT);    
  pinMode(Relay_X358_12, OUTPUT);    
  pinMode(Relay_Spare, OUTPUT);    

  // Set up the serial port to get commands from the laptop
  Serial.begin(9600);

  // Set the cycle off to start with
  ison = 0;

  // Display instructions  
  Serial.println("Instructions...");
  Serial.println("9 = Total shutdown");
  Serial.println("1 = Main Line Enable");
  Serial.println("2 = Main Line Disable");
  Serial.println("3 = Charger Line Enable");
  Serial.println("4 = Charger Line Disable");
  Serial.println("");

}//--(end setup )---

void loop()
{
  // Get a input signal from the serial port (computer USB)
  getSerial();

  // Request to shutdown all the relays
  if (serialdata==9)
  {
    Serial.println("Total Shutdown...");
    digitalWrite(Relay_X358_5, RELAY_OFF);
    digitalWrite(Relay_X358_7, RELAY_OFF);
    digitalWrite(Relay_X358_8, RELAY_OFF);
    digitalWrite(Relay_X358_9, RELAY_OFF);
    digitalWrite(Relay_X358_10, RELAY_OFF);
    digitalWrite(Relay_X358_11, RELAY_OFF);
    digitalWrite(Relay_X358_12, RELAY_OFF);
    digitalWrite(Relay_Spare, RELAY_OFF);
    Serial.println("Total shutdown completed");
    ison = 0;
  }

  // Enabled the Main Line Power (pre-charger sequence)
  if (serialdata==1)
  {
      if (ison!=0)
      {
        Serial.println("Warning : Unexpected starting state");
      }
      Serial.println("Main Line Enabling...");
      digitalWrite(Relay_X358_5, RELAY_ON);  // Ground for all Contactors
      delay(DELAY_INTERRELAY);
      digitalWrite(Relay_X358_7, RELAY_ON);  // Enable Main -VE Contactor
      digitalWrite(Relay_X358_8, RELAY_ON);  // Enable Multifunction Contactor
      delay(DELAY_INTERRELAY);
      digitalWrite(Relay_X358_9, RELAY_ON);  // Enable Pre-charger +VE Contactor
      delay(DELAY_PRECHARGE);                      // Wait 2 seconds  // Note: This is a guess at how long the spike will last
      digitalWrite(Relay_X358_10, RELAY_ON);  // Enable Main +VE Contactor
      delay(DELAY_INTERRELAY);                      // Wait 1/4 second  // Note: This is a guess at how long Main +VE contactor takes to fully connect
      digitalWrite(Relay_X358_9, RELAY_OFF);  // Disable Pre-charger +VE Contactor
      ison = 1;
      Serial.println("Main Line Enabled");
  }

  // Disable the Main Line Power
  if (serialdata==2)
  {
      if (ison!=1)
      {
        Serial.println("Warning : Select 1 first to enable Main Line");
      }
      Serial.println("Main Line Disabling");
      digitalWrite(Relay_X358_10, RELAY_OFF);  // Disable Main +VE Contactor
      delay(DELAY_INTERRELAY);
      digitalWrite(Relay_X358_7, RELAY_OFF);  // Disable Main -VE Contactor
      digitalWrite(Relay_X358_8, RELAY_OFF);  // Disable Multifunction Contactor
      delay(DELAY_INTERRELAY);
      digitalWrite(Relay_X358_5, RELAY_OFF);  // Ground for all Contactors
      ison = 0;
      Serial.println("Main Line Disabled");
  }

  // Enable the Charger Line Power (not pre-charge protected)
  if (serialdata==3)
  {
      if (ison!=0)
      {
        Serial.println("Warning : Unexpected starting state");
      }
      Serial.println("Charger Line Enabling...");
      digitalWrite(Relay_X358_5, RELAY_ON);  // Ground for all Contactors
      delay(DELAY_INTERRELAY);
      digitalWrite(Relay_X358_11, RELAY_ON);  // Enable Charger -VE Contactor
      digitalWrite(Relay_X358_8, RELAY_ON);  // Enable Multifunction Contactor
      delay(DELAY_INTERRELAY);
      digitalWrite(Relay_X358_12, RELAY_ON);  // Enable Main +VE Contactor
      ison = 3;
      Serial.println("Charger Line Enabled");
  }

  // Disable the Charger Line Power
  if (serialdata==4)
  {
      if (ison!=3)
      {
        Serial.println("Warning : Select 3 first to enable Charger Line");
      }
      Serial.println("Charger Line Disabling");
      digitalWrite(Relay_X358_12, RELAY_OFF);  // Disable Main +VE Contactor
      delay(DELAY_INTERRELAY);
      digitalWrite(Relay_X358_11, RELAY_OFF);  // Disable Main -VE Contactor
      digitalWrite(Relay_X358_8, RELAY_OFF);  // Disable Multifunction Contactor
      delay(DELAY_INTERRELAY);
      digitalWrite(Relay_X358_5, RELAY_OFF);  // Ground for all Contactors
      ison = 0;
      Serial.println("Charger Line Disabled");
  }

}

long getSerial()
{
  serialdata = 0;

  // send data only when you receive data:
  if (Serial.available() > 0) {

    // read the incoming byte:
    inbyte = Serial.read();

    // Calc serialdata
    serialdata = serialdata * 10 + inbyte - '0';
  }

  return serialdata;
}

//*********( THE END )***********

