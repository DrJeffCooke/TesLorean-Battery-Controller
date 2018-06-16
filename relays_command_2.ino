/* YourDuino Example: Relay Control 1.10
  Handles "Relay is active-low" to assure
  no relay activation from reset until
  application is ready.
   terry@yourduino.com */

/*-----( Import needed libraries )-----*/
/*-----( Declare Constants )-----*/
#define RELAY_ON 0
#define RELAY_OFF 1
/*-----( Declare objects )-----*/
/*-----( Declare Variables )-----*/
#define Relay_1  2  // X357-1 to +12v HOT at all times 10A fuse
#define Relay_2  3  // X358-2 to Ground
#define Relay_3  4  // X358-4 to +12v on with Ignition 5A fuse
#define Relay_4  5  // X358-3 to 12v accessory wake up
#define Relay_5  6  // X358-14 to 12v HV energy mgt communication enable
#define Relay_6  7
#define Relay_7  8
#define Relay_8  9

// Variables needed for serial function
unsigned long serialdata;
int inbyte;
unsigned int ison;

void setup()   /****** SETUP: RUNS ONCE ******/
{
//-------( Initialize Pins so relays are inactive at reset)----
  digitalWrite(Relay_1, RELAY_OFF);
  digitalWrite(Relay_2, RELAY_OFF);
  digitalWrite(Relay_3, RELAY_OFF);
  digitalWrite(Relay_4, RELAY_OFF);  
  digitalWrite(Relay_5, RELAY_OFF);  
  digitalWrite(Relay_6, RELAY_OFF);  
  digitalWrite(Relay_7, RELAY_OFF);  
  digitalWrite(Relay_8, RELAY_OFF);  
  
//---( THEN set pins as outputs ) ---
  pinMode(Relay_1, OUTPUT);   
  pinMode(Relay_2, OUTPUT);  
  pinMode(Relay_3, OUTPUT);  
  pinMode(Relay_4, OUTPUT);    
  pinMode(Relay_5, OUTPUT);    
  pinMode(Relay_6, OUTPUT);    
  pinMode(Relay_7, OUTPUT);    
  pinMode(Relay_8, OUTPUT);    

  // Set up the serial port to get commands from the laptop
  Serial.begin(9600);

  // Set the cycle off to start with
  ison = 0;

}//--(end setup )---

void loop()
{
  // Get a input signal from the serial port (computer USB)
  getSerial();

  // 1 = Power-up (X358-1on,2on,5on)
  // 2 = Precharger activate for Battery to Charger connection (X358-9on,11on,8on,pause,12on,9off)
  // 3 = Disconnect charger  (X358-12off,11off,8off) 
  // 4 = Shut down (X358-5off,2off,1off)
  // 5 = Emergency shutdown
  
  if (serialdata==1)
  {
      if (ison==0)
      {
        Serial.println("Power-Up");
        digitalWrite(Relay_1, RELAY_ON);  // X357-1 12v HOT at all times 10A
        delay(1000);
        digitalWrite(Relay_2, RELAY_ON);  // X357-2 Ground
        delay(1000);
        digitalWrite(Relay_3, RELAY_ON);  // X357-4 12v with Ignition 5A
        ison = 1;
      }
  }
  if (serialdata==2)
  {
      if (ison!=0)
      {
        Serial.println("Select 1 first");
      }
      if (ison==1)
      {
        Serial.println("Wake up communications");
        digitalWrite(Relay_4, RELAY_ON); // X357-3 Accessory wake-up
        delay(1000);
        digitalWrite(Relay_5, RELAY_ON); // X358-14 HV communications enable
        delay(1000);
        ison = 2;
        Serial.println("Communications ready");
      }
  }
  if (serialdata==3)
  {
      if (ison!=2)
      {
        Serial.println("Select 1 and 2 first");
      }
      if (ison==2)
      {
        Serial.println("Stop Communications");
        digitalWrite(Relay_5, RELAY_OFF); // X358-14 HV communications enable
        delay(1000);
        digitalWrite(Relay_4, RELAY_OFF); // X357-3 Accessory wake-up
        Serial.println("Communications stopped");
        ison = 3;
      }
  }
  if (serialdata==4)
  {
      if (ison!=3)
      {
        Serial.println("Select 1,2,3 first before disconnecting");
      }
      if (ison==3)
      {
        Serial.println("Power-Down");
        digitalWrite(Relay_3, RELAY_OFF);  // X357-4 Ignition 12v
        delay(1000);
        digitalWrite(Relay_2, RELAY_OFF);  // X357-2 to Ground
        delay(1000);
        digitalWrite(Relay_1, RELAY_OFF);  // X357-1 12v Always HOT
        ison = 0;
        Serial.println("Power-Down Complete");
      }
  }
  if (serialdata==5)
  {
    Serial.println("Safety Power-Down");
    digitalWrite(Relay_8, RELAY_OFF);
    digitalWrite(Relay_7, RELAY_OFF);
    digitalWrite(Relay_6, RELAY_OFF);
    digitalWrite(Relay_5, RELAY_OFF);
    digitalWrite(Relay_4, RELAY_OFF);
    digitalWrite(Relay_3, RELAY_OFF);
    digitalWrite(Relay_2, RELAY_OFF);
    digitalWrite(Relay_1, RELAY_OFF);
    Serial.println("Safety Power-Down Completed");
    ison = 0;
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

