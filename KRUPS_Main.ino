#include <Arduino.h>
#include <IridiumSBD.h>
#include <elapsedMillis.h>
#include <brieflzCompress.h>
#include <PriorityQueue.h>
#include <SD.h>
#include <Adafruit_FONA.h>
#include "KRUPS_Alt_Radio.h"

#include "Packet.h"
#include "Config.h"
#include "Control.h"

#if DEBUG_SENSORS
#include "Debug.h"
#else
#include "KRUPS_TC.h"
#include "KRUPS_SENSORS.h"
#endif

File storedData;
volatile bool launched = false, ejected = true, splash_down = false, GPS_Mode = false, GPSFail = false;

//Radio object initialization
altRadio Radio;
const int REN = 29; // CC1120 and amplifier power pin NOTE: drive low when not transmitting

//GPS Initialization 
#define FONA_RST 11
// this is a large buffer for replies
char replybuffer[255];
//set pins for the the FONA to access the teensy, 
//pins 7(RX) and 8(TX)
HardwareSerial *fonaSerial = &Serial3;
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

/*
   Variables associated with creating and tracking priority packets
*/
int Pnumber = 0;  // Number of data packets that have been created
uint8_t priority_buf[BUFFER_SIZE]; // holds readings to be stored in priority packets
size_t ploc = 0; //location in the buffer
uint8_t numPriority = 0; //number of packets of this time made
double avgCompressP = 0; //avg compression value for this type of packet
elapsedMillis timeSinceP = 0; //time since this packet has been made last (ms)
double avgTimeSinceP; //avg time to make a priority packet (s)

/*
   Variables associated with creating and tracking regular packets
*/
uint8_t regular_buf[BUFFER_SIZE];
size_t rloc = 0; //location in regular buffer
uint8_t numRegular = 0; //number of packets of this time made
double avgCompressR = 0; //avg compression value for this type of packet
elapsedMillis timeSinceR = 0; //time since this packet has been made last (ms)
elapsedMillis TimeSinceLastComm = 0;
double avgTimeSinceR  = 0; //avg time to make a regular packet (s)

/*
   variables for compression
*/
uint8_t compressedData[COMPRESS_BUFF_SIZE]; //storage for compressed data
uint8_t workspace[WORKSPACESIZE]; //workspace required by the compression library

/*
   Stat tracking variables
*/
uint16_t bytesMade = 0; //total data generated to this point
//uint16_t bytesSent = 0; //bytes sent by the modem
uint8_t sendAttemptErrors = 0; // number of times the iridium modem throws an error while attempting to send
int16_t measure_reads = 0; //tells which cycle read we are on (0-3) to determine where to put data

#if USE_MODEM
IridiumSBD isbd(Serial4);  //the object for the iridium modem
#endif

/*
   Message Queue
*/
//priority queue to hold all generated packets, sorts using packetPriorityCalc (defined in Control.h)
PriorityQueue<Packet> message_queue = PriorityQueue<Packet>(packetPriorityCalc);

bool inCallBack = false; //used for debug output


#if true
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}
#endif
void GPS_Test_Mode()
{
  GPS_Mode = true; //set flag to disable nonGPS actions in call back
  blinkLed(2, 250);

#if USE_MODEM
  isbd.sendSBDText("GPS Mode Activated"); //send an indicator message
#endif

  //fill dummy packet
  size_t loc = 0;
  int16_t num = 0;
  while (loc < 1960)
  {
    append(compressedData, loc, num);
  }

  //send until power off (also checks for power off in callback)
  while (true)
  {
#if USE_MODEM
    isbd.sendSBDBinary(compressedData, 1960);
#endif
    checkPowerOffSignal();
    delay(GPS_MODE_FREQ * 1000);
  }
}

/*
   Protocol to send a message from the top the priority queue, with error handling
*/
void startSendingMessage()
{
  //grab a packet
  Packet packet = message_queue.peek();
  uint16_t loc  = packet.getLength();
  packStats(packet);

  Serial.println(loc);
  //attempt to send
#if USE_MODEM
  int response =  isbd.sendSBDBinary(packet.getArrayBase(), loc-13);
#else
  int response = 0;
#endif

  //if it returns 0 message is sent
  if (response == 0)
  {
    message_queue.pop(); //remove message from the list
#if OUTPUT_PACKETS
    printPacket(packet);
#endif
  }
  else //if not we have an error
  {
    //print the error
    printMessage("Error while sending packet: ");
    printMessageln(response);

    sendAttemptErrors++;
    //don't remove from the queue
  }
  TimeSinceLastComm = 0;
}

/*
   Method to handle reading in data from onboard sensors into buff at loc
   Checking to see how compressed the data is, and if its over the limit pushing
   a packet into the message queue
*/
void readInData(uint8_t* buff, size_t& loc)
{
  unsigned long compressLen = 0;
  size_t priorloc;
  priorloc = loc;

  //read in sensor data, only if corresponding flag is set
#if USE_TIME
  save_time(buff, loc);        // 3 bytes
#endif

#if USE_GYRO
  Read_gyro(buff, loc);        // 6 bytes
#endif

#if USE_HI_ACCEL
  Read_hiaccel(buff, loc);     // 6 bytes
#endif

#if USE_LO_ACCEL
  Read_loaccel(buff, loc);   //lo acccel  // 6 bytes
#endif

#if USE_MAG
  Read_mag(buff, loc);         // 6 bytes
#endif

  static elapsedMillis timeUsed;
  //loop over the mux posistions and read the TC's
  for (int i = 0; i < 4; i++)
  {
    //set the mux posistion
    setMUX(i);
    //track time used to do extra work
    timeUsed = 0;
    //do extra work
    //use the first 100 ms to do compression
    if (i == 0)
    {
      //if we can fill a packet start compressing data to check length
      if (priorloc > PACKET_MAX)
      {
        //compress the current packet
        compressLen = blz_pack(buff, compressedData, priorloc, workspace);

        printMessage("Compressing: ");
        printMessageln(priorloc);
        printMessage("To: ");
        printMessageln(compressLen);
      }

      //if the data compresses to more then fits in a packet we need to pack a packet
      if (compressLen > PACKET_MAX || loc > BUFFER_SIZE * .95 ) //we need to roll back one measurement and make a packet
      {
        compressLen = blz_pack(buff, compressedData, priorloc - MEASURE_READ, workspace); //compress the data

        double efficency = 100 - (100 * (double(compressLen)) / (priorloc - MEASURE_READ)); // measure the efficency of the compression
        Packet p = Packet(priorloc - MEASURE_READ, compressedData, compressLen, &loc == &ploc); //pack the packet, set priority bit if it is a priority packet
        printMessage("Measure_Read: ");  
        printMessageln(MEASURE_READ);
        //Code for storing buffer on
      Pnumber++;
      String Flight = "";
      Flight += 1;
      Flight = Flight + "_PT_";
      Flight += Pnumber;
      Flight = Flight + ".bin";

       printMessageln(Flight);
       storedData = SD.open(Flight.c_str(),FILE_WRITE);
       storedData.write(p.getArrayBase(),p.getLength());
       storedData.close();
       printPacket(p);
  
        
        message_queue.push(p); //push into the current queue

        //copy the data that wasn't packed into the packet
        for (unsigned int j = 0; j < loc - (priorloc - MEASURE_READ); j++)
        {
          buff[j] = buff[priorloc - MEASURE_READ + j];
        }
        loc -= (priorloc - MEASURE_READ);

        //stat tracking
        //if priority queue
        if (&loc == &ploc)
        {
          avgTimeSinceP += timeSinceP / 1000;
          avgCompressP += efficency;
          timeSinceP = 0;
          numPriority++;
        }
        //if regular
        else
        {
          avgTimeSinceR += timeSinceR / 1000;
          avgCompressR += efficency;
          timeSinceR = 0;
          numRegular++;
        }

        printMessageln("PACKET READY");
        printMessage(priorloc - MEASURE_READ);
        printMessage(" compressed to ");
        printMessageln(compressLen);
        printMessage(100 - 100 * double(compressLen) / (priorloc - MEASURE_READ));
        printMessageln("%");
        printMessage("Unused Space: ");
        printMessageln(1960 - p.getLength());
        printMessageln("Packet added");
        printMessageln(p.getLength());
        printMessage("Total Packets: ");
        printMessageln(numPriority + numRegular);
        printMessage("In queue: ");
        printMessageln(message_queue.count());
        printMessage("Time since last packet: ");
        printMessageln(timeSinceP);
        printMessageln(timeSinceR);
      }
    }
    //if we didn't already use enough time wait the rest
    while (timeUsed < 100);

    //read the data in
    Read_TC_at_MUX(buff, loc); //6 bytes per call
  }
  bytesMade += MEASURE_READ; //track the data generated
}

//void splashDown();

/*
   Tasks to be completed at each measurement cycle
*/
void do_tasks()
{
  //pull data into the priority buff
  if (measure_reads == 0)
  {
    readInData(priority_buf, ploc);
    //Serial.println(ploc);
  }
  //put data into the regular buff
  else
  {
    readInData(regular_buf, rloc);
    //Serial.println(rloc);
  }

  measure_reads++; //incremeant measure_reads
  if (measure_reads > 3) //handle wrap around
  {
    measure_reads = 0;
  }

  //check splashDown
  if (millis() > TIME_TO_SPLASH_DOWN)
  {
    Serial.println(TIME_TO_SPLASH_DOWN);
    Serial.println(millis());
    splashDown();
  }
}

/*
   routine of tasks to be called once splash down has occured
*/

void splashDown()
{
  printMessageln("Splashing Down");
  deployChute();
  unsigned long compressLen;
  splash_down = true; //set flag

  //clean up the rest of the data in the buffers and put in queue
  //see if the remainder can be made one packet
  bool inOnePacket = false;
  if (ploc + rloc < COMPRESS_BUFF_SIZE)
  {
    //Serial.println("hybrid Packet");
    //Serial.println("Copying");

    //copy the data over
    for (int i = 0; i < rloc; i++)
    {
      priority_buf[ploc + i] = regular_buf[i];
    }

    //compresss
    compressLen = blz_pack(priority_buf, compressedData, ploc + rloc, workspace);
    //see if result will fit into one packet
    //Serial.println(compressLen);
    if (compressLen <= PACKET_MAX)
    {
      inOnePacket = true;
      Packet p = Packet(ploc + rloc, compressedData, compressLen, true);
      Pnumber++;
      String Flight = "";
      Flight += 1;
      Flight = Flight + "_PT_";
      Flight += Pnumber;
      Flight = Flight + ".bin";
      
      storedData = SD.open(Flight.c_str(),FILE_WRITE);
      storedData.write(p.getArrayBase(),p.getLength());
      storedData.close();
      
      message_queue.push(p);
      numPriority++;
      double efficency = 100 - 100 * double(compressLen) / (ploc + rloc);
      avgCompressP += efficency;
      printMessageln("Packing");
      printMessage(ploc + rloc);
      printMessage(" compressed to ");
      printMessageln(compressLen);
      printMessage(100 - 100 * double(compressLen) / (ploc + rloc));
      printMessageln("%");
    }

  }

  //if we havent made it into one packet keep the two split up and pack them
  if (!inOnePacket)
  {
    //Serial.println("Split up");
    //Serial.println("Priority");

    //priority packet
    compressLen = blz_pack(priority_buf, compressedData, ploc, workspace);
    Packet p = Packet(ploc, compressedData, compressLen, true);
      Pnumber++;
      String Flight = "";
      Flight += 1;
      Flight = Flight + "_PT_";
      Flight += Pnumber;
      Flight = Flight + ".bin";
      
      storedData = SD.open(Flight.c_str(),FILE_WRITE);
      storedData.write(p.getArrayBase(),p.getLength());
      storedData.close();
    message_queue.push(p);
    numPriority++;
    double efficency = 100 - 100 * double(compressLen) / (ploc);
    avgCompressP += efficency;

    printMessage(ploc);
    printMessage(" compressed to ");
    printMessageln(compressLen);
    printMessage(100 - 100 * double(compressLen) / (ploc));
    printMessageln("%");


    //regular packet
    compressLen = blz_pack(regular_buf, compressedData, rloc, workspace);
    p = Packet(rloc, compressedData, compressLen, false);

    Pnumber++;
      Flight = "";
      Flight += 1;
      Flight = Flight + "_PT_";
      Flight += Pnumber;
      Flight = Flight + ".bin";
      
      storedData = SD.open(Flight.c_str(),FILE_WRITE);
      storedData.write(p.getArrayBase(),p.getLength());
      storedData.close();
    message_queue.push(p);
    numRegular++;
    efficency = 100 - 100 * double(compressLen) / (rloc);
    avgCompressR += efficency;

    printMessage(rloc);
    printMessage(" compressed to ");
    printMessageln(compressLen);
    printMessage(100 - 100 * double(compressLen) / (rloc));
    printMessageln("%");
  }
  /*
    Serial.print("Total Packets: ");
    Serial.println(numRegular + numPriority);
    Serial.print("In queue: ");
    Serial.println(message_queue.count());
    Serial.print("In  priority_queue: ");
    Serial.println(priority_queue.count());
  */

}

///run if packet is not sent on a try
bool ISBDCallback() {
  //handles call back flag
  if (!inCallBack)
  {
    //Serial.println("Call Back");
    inCallBack = true;
  }


  if (!splash_down && !GPS_Mode) //if we haven't splashed down or we arent in GPS Mode
  {
    do_tasks(); //make sure we are still reading in measurements and building packets
  }
  else if (!GPS_Mode) //post splash down tasks
  {

  }

  checkSerialIn(); //check for serial commands
  checkPowerOffSignal();

  return true;
}

void GPS_Setup(){
  //starting baud rate, later set to 4800 per FONA requirements
  Serial.begin(115200);
  Serial.println(F("FONA basic test, for krups code"));
  fonaSerial->begin(4800);
  //Connect to FONA
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    GPSFail = true;
  }
  else{
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found FONA"));
  // Print module IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }
  }
}

void setup() {
  
  //changes board power from USB to Battery
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  pinMode(29,OUTPUT);
  digitalWrite(29, HIGH);

  //Radio enable NOTE: Drive low when not transmitting
  digitalWrite(REN, OUTPUT);
  digitalWrite(REN, LOW);
  
  //Pin for parachute solenoids based on KRUPS V5.1
  pinMode(5,OUTPUT);
    
  analogReadRes(12); //Set Analog Reading to full 12 bit range

  int powerMode = analogRead(0); //NOT FINAL
  blinkLed(1, 250);

#if OUTPUT_MESSAGES
  Serial.begin(9600); //debug output
#endif
  Serial4.begin(19200);

  printMessageln("Power on");
  printMessageln("Initializing Sensors");
  printMessageln("Initializing Access to SD card");
  printMessageln("Initializing GPS");

  GPS_Setup();

  //Connects to SD card
  if(SD.begin(BUILTIN_SDCARD)){printMessageln("Connection to SD slot established.");}
  else{printMessageln("It failed to connect to  the SD/n");}
 
  init_Sensors();
  init_TC();

#if OUTPUT_MESSAGES && DEBUG_IRIDIUM && USE_MODEM
  //isbd.attachConsole(Serial);
  //isbd.attachDiags(Serial);
#endif

#if USE_MODEM
  isbd.adjustSendReceiveTimeout(45);
  isbd.useMSSTMWorkaround(false);
  isbd.setPowerProfile(IridiumSBD::POWERPROFILE::DEFAULT_POWER_PROFILE);

  printMessageln("Turning on Modem");
  int err;
  do
  {
    err = isbd.begin(); //try to start modem
    if (err != 0) //if we have an error report it
    {
      printMessageln("Error Turning on Modem:");
      printMessageln(err);
    }
  } while (!(err == 0)); //if err == 0 we successfully started up

  //report starting up and time to start up
  printMessageln("Modem started: ");
  printMessageln(millis() / 1000);
  printMessageln(" s");
  int signalQuality;
  isbd.getSignalQuality(signalQuality);
  Serial.print("On a scale of 0 to 5, signal quality is currently ");
  Serial.print(signalQuality);
  Serial.println(".");
#endif

  if (powerMode > 3000 && powerMode < 3250)
  {
    GPS_Test_Mode();
  }

  printMessageln("boot up complete");
  //init packet timers
  timeSinceR = 0;
  timeSinceP = 0;
  printMessageln(MEASURE_READ );


}

void loop() {

  
  if (!splash_down) //pre splashdown tasks
  {
    do_tasks(); //complete needed tasks for measurment
  }

  /*
     communication control
  */
  //if we have a valid GPS pos that hasnt been sent trasmit it
  //always has priority after splashdown
  if (splash_down && message_queue.isEmpty()) //NEEDS TO ADD WAY TO SEE IF GPS HAS TRANSMITTED
  {
  
  #if USE_MODEM
      int response = 0;//isbd.sendSBDText(currGPSPos().c_str());
  #else
      int response = 0;
      //Serial.println("Current Pos");
     // Serial.println(currGPSPos());
  #endif
    if (response == 0)
    {
      byte ArrayDummy[127];
      for( int i = 0; i < 127; i++)
      {
        ArrayDummy[i] = 'F';
      }
      digitalWrite(REN, HIGH);
      Radio.begin();
      Radio.initPacketTx();
      Radio.transmitData(ArrayDummy, 127);
      digitalWrite(REN, LOW);
      while(TimeSinceLastComm < 1000);
      TimeSinceLastComm = 0;
      
    if(GPSFail != true)
    {
    while(TimeSinceLastComm < 1000);
  // read the network/cellular status
        uint8_t n = fona.getNetworkStatus();
        Serial.print(F("Network status "));
        Serial.print(n);
        Serial.print(F(": "));
        if (n == 0) Serial.println(F("Not registered"));
        if (n == 1) Serial.println(F("Registered (home)"));
        if (n == 2) Serial.println(F("Not registered (searching)"));
        if (n == 3) Serial.println(F("Denied"));
        if (n == 4) Serial.println(F("Unknown"));
        if (n == 5) Serial.println(F("Registered roaming"));
        
      Serial.println(F("Activating GPS"));
        // turn GPS on
        if (!fona.enableGPS(true))
          Serial.println(F("Failed to turn on"));
          
      Serial.println(F("Query GPS location"));
        // check for GPS location
        char gpsdata[120]; // GPS location
        fona.getGPS(0, gpsdata, 120);
        Serial.println(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));
        Serial.println(gpsdata);
      
      Serial.println(F("Send SMS with location"));
      // send an SMS!
        char sendto[11]= "8594462441"; // number location is being sent to 
        Serial.print(F("Send to #"));
        Serial.println(F(sendto));
        Serial.print(F("Sending location...."));
        Serial.println(gpsdata);
        if (!fona.sendSMS(sendto, gpsdata)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }
      }
    }
  }
  else if (!message_queue.isEmpty()) //if there are messages to send, send em
  {
    startSendingMessage(); //send a message from the queue
    inCallBack = false;
    printMessage("In queue: ");
    printMessageln(message_queue.count());
  
  
  //Second delay to not interfere with Iriduim
  if(GPSFail != true && (millis() > 300000))
  {
    while(TimeSinceLastComm < 1000);
  // read the network/cellular status
        uint8_t n = fona.getNetworkStatus();
        Serial.print(F("Network status "));
        Serial.print(n);
        Serial.print(F(": "));
        if (n == 0) Serial.println(F("Not registered"));
        if (n == 1) Serial.println(F("Registered (home)"));
        if (n == 2) Serial.println(F("Not registered (searching)"));
        if (n == 3) Serial.println(F("Denied"));
        if (n == 4) Serial.println(F("Unknown"));
        if (n == 5) Serial.println(F("Registered roaming"));
        
      Serial.println(F("Activating GPS"));
        // turn GPS on
        if (!fona.enableGPS(true))
          Serial.println(F("Failed to turn on"));
          
      Serial.println(F("Query GPS location"));
        // check for GPS location
        char gpsdata[120];
        fona.getGPS(0, gpsdata, 120);
        Serial.println(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));
        Serial.println(gpsdata);
      
      Serial.println(F("Send SMS with location"));
      // send an SMS!
        char sendto[11]= "8594462441";
        Serial.print(F("Send to #"));
        Serial.println(F(sendto));
        Serial.print(F("Sending location...."));
        Serial.println(gpsdata);
        if (!fona.sendSMS(sendto, gpsdata)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }
   }

   }
   TimeSinceLastComm = 0;
   

  checkSerialIn();
  checkPowerOffSignal();

  //No longer need the capsule to turn off, if it is sending data we can find it, if not who cares

}
