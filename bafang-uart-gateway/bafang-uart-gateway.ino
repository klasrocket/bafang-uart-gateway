/*
   BAFANG BBSxx change settings on the fly (aka 'cop button') and BBSxx communication protocol
   Refer to this topic on https://endless-sphere.com for more information:
   https://endless-sphere.com/forums/viewtopic.php?f=2&t=94850&p=1389269
*/

//Commands for BBS controller
const byte Hspeed[] = {0x16, 0x53, 0x0B, 0x03, 0xFF, 0xFF, 0x14, 0x04, 0x04, 0xFF, 0x14, 0x08, 0x00, 0x3C, 0xD2}; //program pedal settings speed:'by displays command'
const byte Lspeed[] = {0x16, 0x53, 0x0B, 0x03, 0xFF, 0x1B, 0x14, 0x04, 0x04, 0xFF, 0x14, 0x08, 0x00, 0x3C, 0xEE}; //program pedal settings speed:27kph
const byte readPed[] = {0x11, 0x53}; // read pedal settings
const byte PAS0[] = {0x16, 0x0B, 0x00, 0x21, 0x00}; //command that display sends when turn off button is pressed


//Hall sensor variables
const int hallPin = 2;      // hall effect sensor pin
int hallState = 0;          // 0=UnlimitedSpeed(with magnet) 1=LimitedSpeed(without magnet), assume UnlimitedSpeed
bool firstHallValue = true; // Ignore first hall sensor reading, otherwise the BBS is programmed on each start-up

//variables for parsing display/controller data
byte DData[2];              //array to hold incomming display data
byte CData[3];              //array to hold incomming controller data
byte controllerResponse[20];//array to hold responses from controller

const byte DSpeed[] = {0x11, 0x20};               //Command that display sends to controller to receive speed data
const byte DAmps[] = {0x11, 0x0A};                //Command that display sends to controller to receive amp data
const byte Dmoving[] = {0x11, 0x31};              //Command that display sends to controller to receive info weather bike is stationarry or not
const byte DBatPerc[] = {0x11, 0x11};             //UNSURE: battery percentage?
const byte Dunknwn[] = {0x11, 0x08};              //UNKNOWN
const byte PASsuccessResp[] = {0x53, 0x0B, 0x5E}; // response from controller when pedal settings were successfully written

// First two bytes of setting PAS level
const byte DPAS[] = {0x16, 0x0B};
// Next two bytes PAS level
const byte DPAS0[] = {0x00, 0x21}; // PAS 0 16 0B 00 21
const byte DPAS1[] = {0x01, 0x22}; // PAS 1 16 0B 01 22
const byte DPAS2[] = {0x0B, 0x2C}; // PAS 2 16 0B 0B 2C
const byte DPAS3[] = {0x0C, 0x2D}; // PAS 3 16 0B 0C 2D
const byte DPAS4[] = {0x0D, 0x2E}; // PAS 4 16 0B 0D 2E
const byte DPAS5[] = {0x02, 0x23}; // PAS 5 16 0B 02 23
const byte DPAS6[] = {0x15, 0x36}; // PAS 6 16 0B 15 36
const byte DPAS7[] = {0x16, 0x37}; // PAS 7 16 0B 16 37
const byte DPAS8[] = {0x17, 0x38}; // PAS 8 16 0B 17 38
const byte DPAS9[] = {0x03, 0x24}; // PAS 9 16 0B 03 24

// Variables for parsing PAS level
bool PASExp = false;                          //True when PAS 'mask'(0x16, 0x0B) received
int PASbyte = 0;                              //counter that keeps track of how many PAS bytes were received (after 0x16, 0x0B)
int PASLevel = -1;                            //int holding PAS level
byte PASreInit[] = {0x16, 0x0B, 0x01, 0x22};  //command for re-initialising PAS level after speed limit change

//Variables for parsing Display commands and controller response
int DataExp = -1;                   //What data is expected from controller: -1=nothing 0=speed, 1=amps, 2=moving, 3=DBatPerc, 4=unknown
int nExpBytes = 0;                  //number of byte expected from controller
int byteNo = 0;                     //number of bytes received from controller
unsigned long  FirstByteTimestamp;  //time when first byte from controller was received
const float wheelsize_m = 2.2 ;     //wheelsize in meters

bool logging = false;   //enable/disable logging over HC05
int whosTalking = 0;    // keep track of who is talking; 0=display, 1=controller

void setup()
{
  Serial.begin(9600);  //HC-05 module @TX-18 RX19
  Serial2.begin(1200);    //Display DP-C18 @TX14 RX15
  Serial3.begin(1200);    //BBS02 controller @TX16 RX17

  pinMode(hallPin, INPUT); // The hall effect sensor pin as an input
}

void loop()
{
  checkhall();
  readDisplay();
  readController();
  readTerminal();
}

void checkhall() {
  // reading the state of the hall effect sensor pin
  int hallStatetmp = digitalRead(hallPin);

  // Ignore first hall sensor reading, do not program BBS02
  if (firstHallValue) {
    hallState = hallStatetmp;
    firstHallValue = false;
    return; // do nothing
  }

  //Only program controller when hall sensor state changes (not every reading)
  if (hallState != hallStatetmp) {
    hallState = hallStatetmp;

    switch (hallState) {
      case 1: //no magnet
        changeSpeedLimitation(false); //Max 27kph
        break;

      case 0: //magnet present
        changeSpeedLimitation(true); //Unlimited (By displays command)
        break;
    }
  }
}

void changeSpeedLimitation(bool HighSpeed) {
  //to avoid programming controller in de middle of a display command
  //wait until controller starts responding
  while (whosTalking == 0) {
    readDisplay();
    readController();
  }

  //clear messages send by controller
  readControllerResponse(false);


  bool PASsucces = false;

  //program pedal settings, stop while loop when controller confirms success
  while (!PASsucces) {
    switch (HighSpeed) {
      case true: //
        programController(Hspeed, sizeof(Hspeed), true);
        break;
      case false: //
        programController(Lspeed, sizeof(Lspeed), true);
        break;
    }
    //check if controller gave expected response
    if (memcmp ( PASsuccessResp, controllerResponse, 3 ) == 0) {
      PASsucces = true;
    }
  }

  //Re-init current PAS-level
  programController(PAS0, sizeof(PAS0), false);
  programController(PASreInit, sizeof(PASreInit), false);

  //clear messages send by display in the mean time
  flushDisplay();
}

void programController(byte *command, int commandsize, bool echoResponse) {
  //write command to controller
  Serial3.write(command, commandsize);

  //Echo to HC05 (for debugging
  Serial.print("writing: ");
  for (int i = 0; i < commandsize; i++)
  {
    Serial.print(command[i], HEX);
  }
  Serial.println("");

  //read controller response
  readControllerResponse(echoResponse);
}

void readControllerResponse(bool echo) {

  // To capture the variable length response from controller, wait max 'maxWaittime' for a next byte to be received
  // When maxWaittime is exceeded, asume controller finished sending response

  int maxWaittime = 200; //If nothing is received after  200 miliseconds, controller finished sending response
  bool exceeded_maxWaittime = false;
  unsigned long waitUntill;
  int byteNo = 0;

  waitUntill = millis() + maxWaittime;

  //monitor messages from controller until maxwaittime is exceeded
  while (!exceeded_maxWaittime)  {
    if (Serial3.available() > 0) {
      byte cByte = Serial3.read();

      //place received byte in array
      controllerResponse[byteNo] = cByte;

      if (byteNo < 18) {
        byteNo = byteNo + 1;
      }

      waitUntill = millis() + maxWaittime; //Adjust waitUntill
    }

    if (millis() > waitUntill) {
      exceeded_maxWaittime = true;
    }
  }

  //echo to HC05
  if (echo) {
    for (int i = 0; i < byteNo; i++) {
      Serial.print(controllerResponse[i], HEX);
    }
    Serial.println("");
  }
}


void flushDisplay() {
  //Read data from Display and do nothing, just clear the buffer
  while (Serial2.available() > 0) {
    byte dummy = Serial2.read();
  }
}

void readController() {
  // Read byte from the controller and send to Display
  if (Serial3.available())
  {
    parseController(Serial3.peek(), millis());
    Serial2.write(Serial3.read());
    whosTalking = 1;
      //  Serial.write(Serial3.read());

  }
}

void readDisplay()  {

  // Read byte from the Display and send to controller
  if (Serial2.available())
  {
    parseDisplay(Serial2.peek());
    Serial3.write(Serial2.read());
    whosTalking = 0;
    //Serial.write(Serial2.read());
  }
}


void parseDisplay(byte Dincomming) {

  //shift bytes
  DData[0] = DData[1];
  DData[1] = Dincomming;

  //check if first two bytes of setting PAS level are received, next two bytes will determine PAS level
  if (memcmp ( DData, DPAS, sizeof(DData) ) == 0) {
    DataExp = -1;
    PASbyte = -1;
    PASExp = true; // next two bytes for setting PAS level are expected
  }


  //Keep track of how many PAS-level bytes are received
  if (PASExp) {
    PASbyte = PASbyte + 1;
  }

  //When two PASlevel bytes are received (after the 0x16, 0x0B mask), parse two PAS-level bytes
  if (PASbyte == 2) {
    PASExp = false; //no longer expecting PAS-level
    PASbyte = -1;//reset
    parsePAS();
  }

  //when logging turned off, skip rest of parsing
  if (!logging) {
    return;
  }


  //compare byte array 'DData' with known commands from display
  if (memcmp ( DData, DSpeed, sizeof(DData) ) == 0) {
    DataExp = 0;
    nExpBytes = 3;
  }
  else if (memcmp ( DData, DAmps, sizeof(DData) ) == 0) {
    DataExp = 1;
    nExpBytes = 2;
  }
  else if (memcmp ( DData, Dmoving, sizeof(DData) ) == 0) {
    DataExp = 2;
    nExpBytes = 2;
  }
  else if (memcmp ( DData, DBatPerc, sizeof(DData) ) == 0) {
    DataExp = 3;
    nExpBytes = 2;
  }
  else if (memcmp ( DData, Dunknwn, sizeof(DData) ) == 0) {
    DataExp = 4;
    nExpBytes = 1;
  }
  else {
    DataExp = -1;
    nExpBytes = 0;
  }

}

void parsePAS() {

  int oldPAS = PASLevel;

  //compare byte array with PAS levels
  if (memcmp ( DData, DPAS0, sizeof(DData) ) == 0) {
    PASLevel = 0;
  }
  else if (memcmp ( DData, DPAS2, sizeof(DData) ) == 0) {
    PASLevel = 1;
  }
  else if (memcmp ( DData, DPAS4, sizeof(DData) ) == 0) {
    PASLevel = 2;
  }
  else if (memcmp ( DData, DPAS6, sizeof(DData) ) == 0) {
    PASLevel = 3;
  }
  
  else if (memcmp ( DData, DPAS8, sizeof(DData) ) == 0) {
    PASLevel = 4;
  }
  else if (memcmp ( DData, DPAS9, sizeof(DData) ) == 0) {
    PASLevel = 5;
  }


  // Only do if PAS level actually changed
  if (PASLevel != oldPAS) {
    // set command for re-initialising current PAS level (in case speed limit needs to be changed)
    PASreInit[2] = DData[0];
    PASreInit[3] = DData[1];

    //Echo to HC05
    Serial.print("*P");
    Serial.print(PASLevel);
    //Serial.print(",");
    //Serial.print(millis());
    Serial.print("*");
  }
}


void parseController(byte Cincomming, unsigned long timestamp) {
  //when logging turned off, skip procedure
  if (!logging) {
    return;
  }

  //skip procedure if no data is expected
  if (DataExp == -1) {
    byteNo = 0;
    return;
  }

  //timestamp when first byte was received
  if (byteNo == 0) {
    FirstByteTimestamp = timestamp;
  }

  //if bytes are expected, place byte in array and increment byteNo
  if (byteNo < nExpBytes) {
    CData[byteNo] = Cincomming;
    byteNo = byteNo + 1;
  }

  //when the number of expected bytes for a specific display command have been received, parse data
  if (nExpBytes == byteNo) {

    switch (DataExp) {
      case 0: //speed/
        Serial.print("*Spd");
        Serial.print((CData[1] + CData[0] * 256) * wheelsize_m * 60 / 1000); //CData[1]=rpm when rpm>255 CData[0]=1 then 256 should be added to CData[1] for correct rpm
        //Serial.print(",");
        //Serial.print(FirstByteTimestamp);
        Serial.println("*");
        break;
      case 1: //amps
        Serial.print("*Amp");
        Serial.print(CData[0] / 2); // divided by two this seems to be the Amps value
        //Serial.print(",");
        //Serial.print(FirstByteTimestamp);
        Serial.println("*");
        break;
      case 2: //moving
        Serial.print("*Moving");
        Serial.print(CData[0]); //30=stationary, 31=bike is moving
        //Serial.print(",");
        //Serial.print(FirstByteTimestamp);
        Serial.println("*");
        break;
      case 3: //BatteryPercentage (?)
        Serial.print("*Bat");
        Serial.print(CData[0]);
        //  Serial.print(",");
        // Serial.print(FirstByteTimestamp);
        Serial.println("*");
        break;
      case 4: //UNKNOWN
        Serial.print("*U");
        Serial.print(CData[0]);
        // Serial.print(",");
        //Serial.print(FirstByteTimestamp);
        Serial.println("*");
        break;
    }
    byteNo = 0;//reset
    DataExp = -1;//reset
  }
}

void readTerminal()  {
  // read commands received from terminal
  if (Serial.available())
  {
    char terminal =  Serial.read();

    switch (terminal) {
      case 'l'://switch to limited speed
        changeSpeedLimitation(false);
        Serial.println("changeSpeedLimitation(false);");
        break;
      case 'h'://switch to unlimited speed
        Serial.println("changeSpeedLimitation(true);");
        break;
      case '0':     // turn logging off
        logging = false;
        Serial.println("logging = false;");
        break;
      case '1':      // turn logging on
        logging = true;
        Serial.println("logging = true;");
        break;
    }
  }
}







