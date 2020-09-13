#include <Arduino.h>
#include <FlexCAN.h> //https://github.com/teachop/FlexCAN_Library 
#include <Metro.h>
#include <SdFat.h>
#include <SPI.h>
#include <TimeLib.h>
#include <wire.h>
#include <MTP.h>




unsigned char mes[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
char msgString[128];                        // Array to store serial string

/////////// Pin Assignments /////////
const int led = 13;
const int lvRead = A0;
const int backfeed = 24;
const int wpump = 30;
const int tachout = 16;
const int speedout = 2; // tone() for frequency  12 will be permanent pin
const int tempout = 6;
const int sw12 = 28;
const int blowerRead = A4;
const int cabinHeat = 11;
const int driveMode = 19;

//const int chipSelect = SS;
//const int abs_f =  18;
//const int abs_r = 17;


/////////// Variables //////////////
int rpm;
int mtemp;
int hstemp;
int amps;
int potnom;
int pot;
int tachfreq = 0;
int speedfreq;
int wpduty;
int tempduty;
int run;
int dir = 0;
int brake;
int lvRead_val = 1200;
int restart;
int boost;
int CANthrot;
int modeSelect;
int maxBoost;
int throtByte1;
int throtByte2;
int throtRamp;
int pot2;
int brkNomPedal;
int brkMax;
int brakeByte1;
int brakeByte2;
int regenByte1;
int regenByte2;
int brkbuff1;
int brkbuff2;
int brkbuff3;
int brkbuff4;
int baseRegen;
int maxRegen = 90;
int idleThrot;
int iThrotByte1;
int iThrotByte2;
int neg = 4294967295;
float maxSlip;
float minSlip;
float fslip;
float temp = 0.00;
float pack = 0.00;
float cellh = 0.000;
float celll = 0.000;
float temp2 = 0.00;
float pack2 = 0.00;
float cellh2 = 0.000;
float celll2 = 0.000;
float lvBatt = 0.00;
String Year = year();
String Mon = month();
String Day = day();
String Hour = hour();
String Min = minute();
String Sec = second();
int charge;
int blower_val;
//int tender = 0;

///////////// Timers /////////////
Metro tenderTimer = Metro(800000);
Metro debug = Metro(500);
Metro latency = Metro(250);
Metro temp_latency = Metro(3000);
elapsedMillis tenderMillis;


CAN_message_t msg;
CAN_message_t inMsg;
CAN_filter_t filter;


///////////////////////////  DATA INPUT PARSING //////////////////////////


const byte numChars = 120;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
char incomingData[numChars] = { 0 };
const byte numChars2 = 120;
char receivedChars2[numChars];
char tempChars2[numChars];        // temporary array for use when parsing
char incomingData2[numChars] = { 0 };

boolean newData = false;
boolean newData2 = false;

/////////////////////////////////  SD LOGGER ////////////////////////////////////////
// SD chip select pin.  Be sure to disable any other SPI devices such as Enet.
const uint8_t chipSelect = SS;

// Interval between data records in milliseconds.
// The interval must be greater than the maximum SD write latency plus the
// time to acquire and write data to the SD to avoid overrun errors.
// Run the bench example to check the quality of your SD card.
const uint32_t SAMPLE_INTERVAL_MS = 250;

// Log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "Data"
//------------------------------------------------------------------------------
boolean file_setup = false;
boolean log_active = false;
boolean sd_begin = false;

// File system object.
//SdFatSdio sd;
SdFatSdioEX sd;

// Log file.
SdFile file;

// Time in micros for next data record.
uint32_t logTime;

// Error messages stored in flash.
#define error(msg) sd.errorPrint(F(msg))

//////////////////////////////// MTP //////////////////////////////////////
MTPStorage_SD storage;
MTPD          mtpd(&storage);


volatile int  status = 0;
volatile bool sdfound = 0;
volatile int  count = 1;


void setup() {

    pinMode(led, OUTPUT);
    pinMode(tachout, OUTPUT);
    pinMode(wpump, OUTPUT);
    pinMode(tempout, OUTPUT);
    pinMode(backfeed, OUTPUT);
    pinMode(speedout, OUTPUT);
    pinMode(sw12, INPUT_PULLDOWN);
    pinMode(cabinHeat, OUTPUT);
    pinMode(driveMode, INPUT_PULLDOWN);
    analogWriteFrequency(wpump, 100);

   //analogWriteFrequency(tachout, 0);
   // digitalWrite(30, HIGH);

    // enable WDT
    noInterrupts();                                         // don't allow interrupts while setting up WDOG
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;                         // unlock access to WDOG registers
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    delayMicroseconds(1);                                   // Need to wait a bit..

    WDOG_TOVALH = 0x1000;
    WDOG_TOVALL = 0x0000;
    WDOG_PRESC = 0;
    WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
        WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
        WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
    interrupts();
    /////////////////

    Can0.begin(500000);
    
    //set filters for standard
    for (int i = 0; i < 8; i++)
    {
        Can0.setFilter(filter, i);
    }
    //set filters for extended
    for (int i = 9; i < 13; i++)
    {
        Can0.setFilter(filter, i);
    }

    //RTC_IER |= 0x10;  // Enable seconds IRQ from RTC peripheral
    //NVIC_ENABLE_IRQ(IRQ_RTC_SECOND); // Enable seconds IRS function in NVIC

    digitalWrite(led, HIGH);
    Serial.begin(1152000);
    Serial3.begin(9600);  //(19200);
    
    
 
    
}

void loop() {
        
    while (Can0.available())
    {
        Can0.read(inMsg);
        decodeCAN();  
        
    }
    charging();
    if (digitalRead(sw12) == LOW) {
        run = 0;
    }

    modeSwitch();
    waterpump();
    dcdc();
    heater();  
    resetwdog(); 
    boostMap();
    idleThrottle();
    regenStuff();

    
  


    /*getData();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
        // this temporary copy is necessary to protect the original data
        //   because strtok() used in parseData() replaces the commas with \0
        parse_BMS_1();
        newData = false;
    
    }
    if (newData2 == true) {
        strcpy(tempChars2, receivedChars2);
        // this temporary copy is necessary to protect the original data
        //   because strtok() used in parseData() replaces the commas with \0
        parse_BMS_2();
        newData2 = false;

    }*/
    
    //test_params();
    if (latency.check() == 1) {
        gaugeupdate();
    }


    if (debug.check() == 1) {
        outputs();
    }
                 
    resetwdog();


    if ((dir == 454)) {
        log_active = true;
        
    }
    else { log_active = false; }

    if ((log_active == true)) {
        SD_logger();
    }

    else if (log_active == false) {    
        file.close();
        file_setup = false;
        
        MTP_loop(); 
    }

    resetwdog();
}


void dcdc() {

    
        msg.id = 0x1D4;
        msg.len = 2;
        msg.buf[0] = 0xA0;
        msg.buf[1] = 0xBA; //0xBA 14.7v   0xAB 13.5v

        Can0.write(msg);
    }


    
    



void decodeCAN() {

    if (inMsg.id == 0x135) {
        if ((((inMsg.buf[3] << 8) + inMsg.buf[2])) <= 2000) {

            amps = (((inMsg.buf[3] << 8) + inMsg.buf[2]) * 1.83);
        }
        else if ((((inMsg.buf[3] << 8) + inMsg.buf[2])) >= 3000) {

            amps = (((((inMsg.buf[3] << 8) + inMsg.buf[2]) - 65535) * 1.83) * -1);

        }
        rpm = (((inMsg.buf[1] << 8) + inMsg.buf[0]));

        if ((inMsg.buf[4]) > 0) {
            mtemp = (inMsg.buf[4]); //motor temp C
        }
          
        if ((inMsg.buf[5]) > 0) {
            hstemp =(inMsg.buf[5]); //heatsink temp C
        }
        if ((((inMsg.buf[7] << 8)) + inMsg.buf[6]) <= 2000) {
            potnom = (((inMsg.buf[7] << 8)) + inMsg.buf[6]);
        }
        else if ((((inMsg.buf[7] << 8)) + inMsg.buf[6]) >= 2000) {
            potnom = ((((inMsg.buf[7] << 8)) + inMsg.buf[6]) - 65535);
        }
    }

    else if (inMsg.id == 79) {
       
         dir = (inMsg.buf[0]);    
         
    }

    else if (inMsg.id == 0x136) {

        run = (inMsg.buf[0]);
    }   

    else if (inMsg.id == 0x2D0) {
        
        charge = (inMsg.buf[0]);
    }

    else if (inMsg.id == 0x12D) {
        restart = ((inMsg.buf[1] << 8) + inMsg.buf[0]);//(inMsg.buf[0]);
    }

    else if (inMsg.id == 0x113) {
        pot = ((inMsg.buf[1] << 8) + inMsg.buf[0]);
        pot2 = ((inMsg.buf[3] << 8) + inMsg.buf[2]);
    }

    else if (inMsg.id == 0x581) {
        if (inMsg.buf[0] == 0x43 && inMsg.buf[3] == 64 ) {
            brkbuff1 = (inMsg.buf[4]);
            brkbuff2 = (inMsg.buf[5]);
            brkbuff3 = (inMsg.buf[6]);
            brkbuff4 = (inMsg.buf[7]);
        }
    }
}




void getData() {

    static boolean recvInProgress = false;
    static boolean recvInProgress2 = false;
    static byte ndx = 0;
    static byte ndx2 = 0;
    char startMarker = '$';
    char endMarker = '%';
    char rc;
    char rc2;

    while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
            yield();
        }
        else if (rc == startMarker) {
            recvInProgress = true;
            yield();
        }
    }
    //////////////////////////////////////////////////////////////////////
    while (Serial4.available() > 0 && newData2 == false) {
        rc2 = Serial4.read();
        if (recvInProgress2 == true) {
            if (rc2 != endMarker) {
                receivedChars2[ndx2] = rc2;
                ndx2++;
                if (ndx2 >= numChars2) {
                    ndx2 = numChars2 - 1;
                }
            }
            else {
                receivedChars2[ndx2] = '\0'; // terminate the string
                recvInProgress2 = false;
                ndx2 = 0;
                newData2 = true;
            }
            yield();
        }
        else if (rc2 == startMarker) {
            recvInProgress2 = true;
            yield();
        }
    }
}


void parse_BMS_1() {
    //////////////////////// Serial data BMS 1 //////////////////////////////
    char* strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars, ",");      // get the first part - the string
    strcpy(incomingData, strtokIndx); // copy it to messageFromPC

    strtokIndx = strtok(NULL, ","); //1 this continues where the previous call left off
    temp = atof(strtokIndx);

    strtokIndx = strtok(NULL, ","); //2
    pack = atof(strtokIndx);

    strtokIndx = strtok(NULL, ","); //3
    cellh = atof(strtokIndx);

    strtokIndx = strtok(NULL, ","); //4
    celll = atof(strtokIndx);
}


void parse_BMS_2() {
    /////////////////// Serial data BMS 2 /////////////////////////
    char* strtokIndx2; // this is used by strtok() as an index

    strtokIndx2 = strtok(tempChars, ",");      // get the first part - the string
    strcpy(incomingData2, strtokIndx2); // copy it to messageFromPC

    strtokIndx2 = strtok(NULL, ","); //1 this continues where the previous call left off
    temp2 = atof(strtokIndx2);

    strtokIndx2 = strtok(NULL, ","); //2
    pack2 = atof(strtokIndx2);

    strtokIndx2 = strtok(NULL, ","); //3
    cellh2 = atof(strtokIndx2);

    strtokIndx2 = strtok(NULL, ","); //4
    celll2 = atof(strtokIndx2);
}

void batteryTender() {
 
        lvRead_val = analogRead(lvRead);

        if (lvRead_val < 920) {  //0-1023 (3.3v) range R1=510 ohm, R2= 150 ohm *breadboard voltage divider 510 long side*
            
            digitalWrite(backfeed, HIGH);
            tenderMillis = 0;
        }
        if ( tenderMillis > 800000){
           
            digitalWrite(backfeed, LOW);
                     
        }       
        if (dir == 255) {
            digitalWrite( backfeed, LOW);
        }
    
}



void waterpump()
{
    if (charge == 2) {
        if (dir == 255) {
            wpduty = map(pot, 700, 4095, 50, 255);
            analogWrite(wpump, wpduty);
        }
        else { analogWrite(wpump, 0); }

    }
    else if (charge == 1) {
        
        analogWrite(wpump, 100);
    }

    
}

void charging() {

    if (charge == 1) {
        digitalWrite(backfeed, HIGH);
    }

    else if (charge == 2){
        batteryTender();
        }
    }


void heater() {

    blower_val = analogRead(blowerRead);

    if (blower_val > 500) {
        digitalWrite(cabinHeat, HIGH);

    }
    else {
        digitalWrite(cabinHeat, LOW);
    }


}

void resetEsp() {

    //pinMode(30, OUTPUT);
    //delay(1);
    //pinMode(30, INPUT);
}

void gaugeupdate() {
    //tach
    tachfreq = ((amps * 6.3) * .05);   //amps * 6.3 for true value, amps * 12 for shit value
    //analogWriteFrequency(tachout,tachfreq);
    //analogWrite(tachout, 125);
    tone(tachout, tachfreq);

    //speedo
    speedfreq = (rpm * .014);
    analogWriteFrequency(speedout, speedfreq);
    analogWrite(speedout, 125);
    //tone(speedout, speedfreq);

    //temp
    
    tempduty = map(mtemp, 0, 90, 50, 205);
    analogWrite(tempout, tempduty);
    
}

void outputs() {

    Serial3.print("Inverter Temp: ");
    Serial3.println((mtemp*1.8)+32);
    Serial3.print("Drive Mode: ");
    if (digitalRead(driveMode)==LOW) {
        Serial3.println("Chill");
    }
    else if (digitalRead(driveMode) == HIGH) {
        Serial3.println("Party");
    
    }
    
    Serial3.print("Status: ");
    if (restart == 64){
        Serial3.println("Pot Pressed");
    }
    else if (restart == 128) {
        Serial3.println("TMPHS");
    }
    else if (restart == 256) {
        Serial3.println("Wait Start");
    }
    else if (restart == 32) {
        Serial3.println("MProt");
    }
    else if (restart == 0) {
        Serial3.println("None");
    }
    else if (restart == 1) {
        Serial3.println("UDC Low");
    }
    else if (restart == 2) {
        Serial3.println("UDC High");
    }  
    Serial3.print("DC Voltage: ");
    Serial3.println("No Sensor");

    Serial3.println(dir);

    Serial3.println("brakenom");
    Serial3.println(brkbuff1);
    Serial3.println(brkbuff2);
    Serial3.println(brkbuff3);
    Serial3.println(brkbuff4);


    //Serial3.print("Pack1");
    //Serial3.println(pack);
    Serial3.println(".");
    Serial3.println(".");
    Serial3.println(".");
    Serial3.println(".");

    }

void test_params() {
   
   rpm = 0;
   mtemp= 120;
     //hstemp;
   amps= 200;  
   potnom=25;

}



//////////////////////////////// LOGGER MAIN ////////////////////////////////////

void SD_logger() {

    if (file_setup != true) {
        logger_setup();
        file_setup = true;
    }

    // Time for next record.
    logTime += 1000UL * SAMPLE_INTERVAL_MS;

    // Wait for log time.
    int32_t diff;
    do {
        diff = micros() - logTime;
    } while (diff < 0);

    // Check for data rate too high.
    if (diff > 100) {
        error("Missed data record");
    }

    logData();

    // Force data to SD and update the directory entry to avoid data loss.
    if (!file.sync() || file.getWriteError()) {
        error("write error");
    }

}

///////////////////////////////////////  LOGGER FUNCTIONS ////////////////////////////////////////////////

// Log a data record.
void logData() {
    int data[] = { rpm,amps,potnom };

    // Write data to file.  Start with log time in micros.
    //log_sec = (logTime / 10000000);
    file.print(logTime);
    file.write(',');

   
    for (int i = 0; i < 3; i++) {
        file.print(data[i]); {
            file.write(',');
        }

    }
    file.println();
}


void logger_setup() {

    
    const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
    char fileName[48] = FILE_BASE_NAME "00.csv";


    sd.begin();
    if (!sd.begin()) {
        //sd.initErrorHalt();
    }
  
    // Find an unused file name.
    if (BASE_NAME_SIZE > 6) {
        error("FILE_BASE_NAME too long");
    }
    while (sd.exists(fileName)) {
        if (fileName[BASE_NAME_SIZE + 1] != '9') {
            fileName[BASE_NAME_SIZE + 1]++;
            
        }
        else if (fileName[BASE_NAME_SIZE] != '9') {
            fileName[BASE_NAME_SIZE + 1] = '0';
            fileName[BASE_NAME_SIZE]++;
            
        }
        else {
            error("Can't create file name");
        }
       
    }
    
    if (!file.open(fileName, O_WRONLY | O_CREAT | O_EXCL)) {
        error("file.open");
        
    }


    Serial3.print(F("Logging to: "));
    Serial3.println(fileName);
   

    // Write data header.
    writeHeader();

    // Start on a multiple of the sample interval.
    logTime = micros() / (1000UL * SAMPLE_INTERVAL_MS) + 1;
    logTime *= 1000UL * SAMPLE_INTERVAL_MS;


}

void writeHeader() {
    file.print(__TIMESTAMP__);
    file.println();
    file.print(F("micros"));
    file.print(",");
    file.print("rpm,amps,potnom");
    file.println();

}

void rtc_seconds_isr() { 
    if (count-- == 0) {
        digitalWrite(LED_BUILTIN, status);
        //Serial.println("I should be commented out");
        status = !status;
        if (sdfound)
            count = 2;
        else
            count = 1;
    }
}

void MTP_loop() {
    
    if (SD.begin()) {
        sdfound = true;
        mtpd.loop();
    }
    else {

        sdfound = false;
    }
}

void resetwdog()
{
    noInterrupts();                                     //   No - reset WDT
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
}



void oops() 
{
   // if (digitalRead(backfeed) == LOW) {
        if (restart == 256) {
            msg.id = 0x12C;
            msg.len = 1;
            msg.buf[0] = 0x20; //0xC == 12, start/brake // OxE == 14, start/brake/fwd// 0x1A ==26 start/brake/rev

            Can0.write(msg);
        }
   // }
}



void throttleLimit() {
  
}

void party() {
    //boost max
    maxBoost = 230;
    //fweak
    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD
    msg.buf[1] = 0x00; 
    msg.buf[2] = 0x20; //
    msg.buf[3] = 0x01;//index:fweak=2-1
    msg.buf[4] = 0xC0;
    msg.buf[5] = 0x1D;//238=7616=0x1DC0
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    Can0.write(msg);


    //slip max
    maxSlip = (3.24 * 32);
    if (pot >= 3000) {
        minSlip = map(pot, 3000, 4095, (.95 * 32), maxSlip);

    }
    else { minSlip = (.95 * 32); }

    if (rpm <= 4000) {
        fslip = map(rpm, 0, 4000, minSlip, maxSlip);
    }
    else { fslip = maxSlip; }
    //fslipmax
    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20; 
    msg.buf[3] = 0x05;//index:boost=0
    msg.buf[4] = fslip;
    msg.buf[5] = 0x00;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    Can0.write(msg);


    // throtramp

    if (rpm < 2000 && pot < 3000) {
        throtRamp = (8 * 32);
    }
    else if (rpm >= 2000 && rpm <= 4500) {
        throtRamp = map(rpm, 2000, 4500, (8 * 32), (75 * 32));
    }
    else {
        throtRamp = (75 * 32);
    }

    //throtRamp = (3 * 32);
    throtByte1 = throtRamp & 0xFF;
    throtByte2 = (throtRamp >> 8) & 0xFF;

    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20;
    msg.buf[3] = 49;//index
    msg.buf[4] = throtByte1;
    msg.buf[5] = throtByte2;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    Can0.write(msg);
   

}

void chill() {
    //boost max
    maxBoost = 215;
    //fweak
    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD
    msg.buf[1] = 0x00; 
    msg.buf[2] = 0x20; //
    msg.buf[3] = 0x01;//index:boost=0
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x20;//258=8102=0x2000
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    Can0.write(msg);

    //slip max
    maxSlip = (3.08 * 32);
    if (pot >= 3000) {
        minSlip = map(pot, 3000, 4095, (.95 * 32), maxSlip);

    }
    else { minSlip = (.95 * 32); }

    if (rpm <= 4000) {
        fslip = map(rpm, 0, 4000, minSlip, maxSlip);
    }
    else { fslip = maxSlip; }
    //fslipmax
    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20; //
    msg.buf[3] = 0x05;//index:boost=0
    msg.buf[4] = fslip;
    msg.buf[5] = 0x00;//258=8102=0x2000
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    Can0.write(msg);

    // throtramp

    if (rpm < 2000 && pot < 3000) {
        throtRamp = (8 * 32);
    }
    else if (rpm >= 2000 && rpm <= 4500) {
        throtRamp = map(rpm, 2000, 4500, (8 * 32), (25 * 32));
    }
    else {
        throtRamp = (25 * 32);
    }

//throtRamp = (3 * 32);
    throtByte1 = throtRamp & 0xFF;
    throtByte2 = (throtRamp >> 8) & 0xFF;

    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD
    msg.buf[1] = 0x00; 
    msg.buf[2] = 0x20; 
    msg.buf[3] = 49;//index
    msg.buf[4] = throtByte1;
    msg.buf[5] = throtByte2;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    Can0.write(msg);



}
void boostMap()
{
    if (pot > 3700) {

        boost = map(pot, 2700, 4095, 100, maxBoost);

        msg.id = 0x601; //set parameter ID
        msg.len = 8;
        msg.buf[0] = 0x40; //CMD
        msg.buf[1] = 0x00; 
        msg.buf[2] = 0x20; //
        msg.buf[3] = 0x00;//index:boost=0, count down for index number
        msg.buf[4] = 0x00;
        msg.buf[5] = boost;//value x 32
        msg.buf[6] = 0x00;
        msg.buf[7] = 0x00;
        Can0.write(msg);
    }

    else {
        msg.id = 0x601; //set parameter ID
        msg.len = 8;
        msg.buf[0] = 0x40; //CMD
        msg.buf[1] = 0x00; //
        msg.buf[2] = 0x20; //
        msg.buf[3] = 0x00;
        msg.buf[4] = 0x00;
        msg.buf[5] = 175;//value x 32
        msg.buf[6] = 0x00;
        msg.buf[7] = 0x00;
        Can0.write(msg);
    }

}

void regenStuff() {

    //baseRegen
    if (rpm < 1750) {
        baseRegen = map(rpm, 0, 1750, 0, 15);
    }
    else {
        baseRegen = 45;
    }

    //brakenompedal
    if (pot2 > 3700) {
        brkNomPedal = (neg - (maxRegen * 32));
    }

    else {
        brkNomPedal = map(pot2, 600, 3700, (neg - (baseRegen * 32)), (neg - (maxRegen * 32)));
    }

    //brkNomPedal = (neg -  (11 * 32));

    brakeByte1 = brkNomPedal & 0xFF;
    brakeByte2 = (brkNomPedal >> 8) & 0xFF;


    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD SET
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20;
    msg.buf[3] = 53;//index
    msg.buf[4] = brakeByte1;
    msg.buf[5] = brakeByte2;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFF;
    Can0.write(msg);

    //brakemax
    brkMax = (neg - (baseRegen * 32));
    regenByte1 = brkMax & 0xFF;
    regenByte2 = (brkMax >> 8) & 0xFF;

    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD SET
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20;
    msg.buf[3] = 56;//index
    msg.buf[4] = regenByte1;
    msg.buf[5] = regenByte2;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFF;
    Can0.write(msg);

}

void idleThrottle() {
    if (rpm < 90) {
        msg.id = 0x601; //set parameter ID
        msg.len = 8;
        msg.buf[0] = 0x40; //CMD
        msg.buf[1] = 0x00; //
        msg.buf[2] = 0x20; //
        msg.buf[3] = 64;
        msg.buf[4] = 0x20;
        msg.buf[5] = 0x00;//value x 32
        msg.buf[6] = 0x00;
        msg.buf[7] = 0x00;
        Can0.write(msg);
    }

    else {
        msg.id = 0x601; //set parameter ID
        msg.len = 8;
        msg.buf[0] = 0x40; //CMD
        msg.buf[1] = 0x00; //
        msg.buf[2] = 0x20; //
        msg.buf[3] = 64;
        msg.buf[4] = 0x00;
        msg.buf[5] = 0x00;//value x 32
        msg.buf[6] = 0x00;
        msg.buf[7] = 0x00;
        Can0.write(msg);
    }

    idleThrot = map(pot2, 600, 1250, (20 * 32), 0);

    iThrotByte1 = idleThrot & 0xFF;
    iThrotByte2 = (idleThrot >> 8) & 0xFF;

    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20; //
    msg.buf[3] = 63;//index
    msg.buf[4] = iThrotByte1;
    msg.buf[5] = iThrotByte2;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    Can0.write(msg);
}



void modeSwitch() {
    if (digitalRead(driveMode) == LOW) {
      chill();
      
    }
    if (digitalRead(driveMode) == HIGH) {
        party();
    }
}


/*Also this version has CANOpen implemented. You can send so called SDO messages:

Id    Len cmd  index  subindex data
0x601 8   0x40 0x2000 paramidx value

cmd is 1 byte, index is 2 bytes(little endian), subindex 1 byte, data 4 bytes.
The subindex is the parameter index, which is a bit hard to find right now.Basically if you type
"list" you count at which position the parameter shows up.For example boost has subindex 0 because
its the very first.
The value is the desired value * 32. So if you want to set boost to 2000 you would send
Id    Len cmd  index     subindex data
0x601 8   0x40 0x00 0x20 0x00     0x00 0xFA 0x00 0x00
*/

