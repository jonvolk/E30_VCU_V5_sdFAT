#include <Arduino.h>
#include <FlexCAN.h> //https://github.com/teachop/FlexCAN_Library 
#include <Metro.h>
#include <SdFat.h>
#include <SPI.h>
#include <TimeLib.h>
#include <wire.h>
#include <MTP.h>
#include <Smoothed.h>
#include <EEPROM.h>
//#include <config.h>

#define FORWARD 8
#define NUETRAL 1
#define REVERSE 2

/////////// Pin Assignments /////////
const int led = 13;

const int lvRead = A0;
const int backfeed = 24;
const int wpump = 30;
const int tachout = 16;
const int speedout = 2; // tone() for frequency  12 will be permanent pin
const int tempout = 6;
const int steerInit = 5; //output pin 27
const int sw12 = 28;
const int blowerRead = A4;
const int cabinHeat = 11;
const int driveMode = 19;
const int inclinometer = A1;

///////////////////timer////////////////
IntervalTimer encoder;
//test
/////////// Variables //////////////
int rpm;
int mtemp;
int hstemp;
int amps;
int potnom;
int pot;
float regenRamp;
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
int minBoost;
int pot2;
int brkNomPedal;
int brkMax;
int slipstart;
int baseRegen;
int maxRegen;
int idleThrot;
int idleThrotMax;
int currentPot2;
int neg = 4294967295;
int fweak;
int idleRPM;
int peakPower;
int power;
int peakTemp;
int peakAmps;
int brakePedal;
int underVoltTime;
int shift;
int screen;
bool underVoltInit;
//int angleAvg;
float throtRamp;
float ampMin;
float packVolt;
float maxSlip;
float minSlip;
float fslip;
float fslipmin;
float angle;
float lvBatt = 0.00;
String Year = year();
String Mon = month();
String Day = day();
String Hour = hour();
String Min = minute();
String Sec = second();
int charge;
int blower_val;
bool startup;
//bool brake;
//int tender = 0;



//////////////Smoothing///////////////

Smoothed <int> idleRamp;
Smoothed <float> fslipRamp;
Smoothed <int> angleAvg;
Smoothed <int> tempAvg;


///////////// Timers /////////////
Metro tenderTimer = Metro(800000);
Metro debug = Metro(2000);
Metro latency = Metro(250);
Metro temp_latency = Metro(300);
elapsedMillis tenderMillis;
Metro idleTime = Metro(1000);


CAN_message_t msg;
CAN_message_t inMsg;
CAN_filter_t filter;





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


/////////////////////// EEPROM ///////////////////////
//LDUParams parameters;

void setup() {

    pinMode(led, OUTPUT);
    pinMode(tachout, OUTPUT);
    pinMode(wpump, OUTPUT);
    pinMode(tempout, OUTPUT);
    pinMode(backfeed, OUTPUT);
    pinMode(speedout, OUTPUT);
    pinMode(sw12, INPUT_PULLDOWN);
    pinMode(cabinHeat, OUTPUT);
    pinMode(steerInit, OUTPUT);
    pinMode(driveMode, INPUT_PULLDOWN);
    pinMode(inclinometer, INPUT);
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

    
    CAN_filter_t allPassFilter;
    allPassFilter.ext = 1;
    for (uint8_t filterNum = 8; filterNum < 16;filterNum++) { //only use half the available filters for the extended IDs
        Can0.setFilter(allPassFilter, filterNum);
    }

/*
    CAN_filter_t mailbox_Filter;
    Can0.setMask(0x7FF << 18, 4);    // bit shifting 18 bits left, mask is checking for a match of all 11 bits.
    mailbox_Filter.id = 0x38E;      // Filter is looking for 0x05, so with a 'full match' requirement defined by the mask,
    mailbox_Filter.ext = 0;         // this will be the only ID received in this mailbox.
    mailbox_Filter.rtr = 0;
    Can0.setFilter(mailbox_Filter, 4);

    CAN_filter_t allBlockExtendedFilter;
    allBlockExtendedFilter.id = 0x581;
    allBlockExtendedFilter.ext = 0;                    // Notice: Extended bit is toggled to 1 (was 0 for Standard)
    allBlockExtendedFilter.rtr = 0;
    for (uint8_t filterNum = 5; filterNum < 16;filterNum++) {
        Can0.setMask(0x581, filterNum);              // Notice: no bit-shifting for 29-bit Extended ID's!
        Can0.setFilter(allBlockExtendedFilter, filterNum);
    }
    */
    //RTC_IER |= 0x10;  // Enable seconds IRQ from RTC peripheral
    //NVIC_ENABLE_IRQ(IRQ_RTC_SECOND); // Enable seconds IRS function in NVIC

    digitalWrite(led, HIGH);
    Serial.begin(1152000);
    Serial3.begin(9600);  //(19200);
    //encoder.begin(shift_dir, 250000);
     
    idleRamp.begin(SMOOTHED_AVERAGE, 60);
    fslipRamp.begin(SMOOTHED_AVERAGE, 60);
    angleAvg.begin(SMOOTHED_EXPONENTIAL, 8);
    tempAvg.begin(SMOOTHED_EXPONENTIAL, 8);
   
    digitalWrite(steerInit, LOW);


 
    
    //EEPROM.read(0, parameters);
 
    
}

void loop() {
        
    while (Can0.available())
    {
        Can0.read(inMsg);
        decodeCAN(); 
        if (charge == 2) {
            shift_dir();
            chill();
            idleThrottle();
            regenStuff();
        }  
        //canread();
        
    }
    charging();

    if (digitalRead(sw12) == LOW) {
        run = 0;
    }

    angle = analogRead(inclinometer);
    angleAvg.add(angle);


    if (charge == 2) {

        boostMap();
    
    }

    waterpump();
    dcdc();
    heater();  
    resetwdog();  
    powerSteer();
    powerCalc();   
  //test_params();
    
    tempAvg.add(mtemp);
    tempduty = map(tempAvg.get(), 0, 90, 50, 205);
    analogWrite(tempout, tempduty);


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
    dcdc();
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
         brake = (inMsg.buf[1]);
         
    }

    else if (inMsg.id == 0x136) {

        run = (inMsg.buf[0]);
        packVolt = ((inMsg.buf[2] << 8) + inMsg.buf[1]);
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

    else if (inMsg.id == 0x38E) {

        brakePedal = ((inMsg.buf[4] << 8) + inMsg.buf[3])-4415;
    }

    else if (inMsg.id == 0x18FF11F2 && (inMsg.buf[6] == 3) && (inMsg.buf[2] == 2)) {

        //shift = (inMsg.buf[3]);
    }
    else if (inMsg.id == 0x18FF0FF2) {

        screen = (inMsg.buf[0]);

    }

    

}


void batteryTender() {
 
        lvRead_val = analogRead(lvRead);

        if (lvRead_val < 920 && underVoltInit == false) {  //0-1023 (3.3v) range R1=510 ohm, R2= 150 ohm *breadboard voltage divider 510 long side*
             
            underVoltInit = true;  
            tenderMillis = 0;
        }


        if (tenderMillis > 5000 && lvRead_val < 920) {

            digitalWrite(backfeed, HIGH);
            underVoltInit = false;

        }

        if (tenderMillis > 800000) {

            digitalWrite(backfeed, LOW);
            underVoltInit = false;

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

    if (blower_val > 500 && run == 1) {
        digitalWrite(cabinHeat, HIGH);

    }
    else {
        digitalWrite(cabinHeat, LOW);
    }
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
    
  
    
}

void outputs() {

    //Serial.print("runmode");
    //Serial.println(run);

    Serial3.print("brake pedal: ");
    Serial3.println(brakePedal);

    Serial3.print("DC Voltage: ");
    Serial3.println(packVolt);

    Serial3.print("Amps: ");
    Serial3.println(amps);

    Serial3.print("Inverter Temp: ");
    Serial3.println((mtemp * 1.8) + 32);

    Serial3.println("|");
    Serial3.println("|");

    Serial3.print("Peak HP: ");
    Serial3.println(peakPower);

   
    Serial3.print("Peak Inv Temp: ");
    Serial3.println((peakTemp * 1.8) + 32);

    Serial3.print("Inclinometer: ");
    Serial3.println(angleAvg.get());

    Serial3.print("Shift: ");
    Serial3.println(shift);

    Serial3.print("12v: ");
    Serial3.println(lvRead_val);

    Serial3.print("backfeed: ");
    Serial3.println(digitalRead(backfeed));


    

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
    noInterrupts(); //   No - reset WDT
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
}

void canSet(int index, float value) {
    int val = (value * 32);
    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40;
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20;
    msg.buf[3] = index;
    msg.buf[4] = val & 0xFF;
    msg.buf[5] = (val >> 8) & 0xFF;
    msg.buf[6] = (val >> 16) & 0xFF;
    msg.buf[7] = (val >> 24) & 0xFF;
    Can0.write(msg);
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



void throttle() {

  
}



void party() {

    //boost
    maxBoost = 1720;
    minBoost = 1720;

    //fweak
    if (pot > 1800 && pot < 3200) {
        fweak = map(pot, 1800, 3200, 400, 238);
    }
    else if (pot >= 3200) {

        fweak = 238;
    }
    else {
        fweak = 400;
    }
    canSet(1, fweak);


    //fslipmin
    fslipmin = 2.3;
    canSet(4, fslipmin);
 
    //fslipmax
    maxSlip = (3.18 * 32);
    if (pot >= 2800) {
        minSlip = map(pot, 2800, 4095, (fslipmin * 32), maxSlip);
    }
    else { minSlip = (fslipmin * 32); }

    if (rpm <= 4500) {
        fslip = map(rpm, 0, 4200, minSlip, maxSlip);
    }
    else { fslip = maxSlip; }

    canSet(5, fslip / 32);


    // throtramp
    if (pot < 900) {
        throtRamp = .45;
    }
    else if (pot >= 900 && pot < 3000) {
        throtRamp = map(pot, 1500, 3700, 1, 85);
    }
    else {
        throtRamp = 85;

    }
    canSet(49, throtRamp);

    //slipstart
    slipstart = 32;
    canSet(52, slipstart);

    //ampmin
    ampMin = 1.1;
    canSet(51, ampMin);

}



void chill() {

    //boost
    maxBoost = 1875;
    minBoost = 1875;

    //fweak
    if (pot > 1800 && pot < 3200) {
        fweak = map(pot, 1800, 3200, 400, 258);
    }
    else if (pot >= 3200) {

        fweak = 258;
    }
    else {
        fweak = 400;
    }
    canSet(1, fweak);


    //fslipmin
    fslipmin = 3.08; //2.5; //was 2.3
    canSet(4, fslipmin);


    //fslipmax
    maxSlip = (3.08 * 32);
    if (pot >= 3200) {
        minSlip = map(pot, 3200, 4095, (fslipmin * 32), maxSlip);

    }
    else { minSlip = (fslipmin * 32); }

    if (rpm <= 4500) {
        fslip = map(rpm, 0, 4200, minSlip, maxSlip);
    }
    else { fslip = maxSlip; }

    canSet(5, fslip / 32);


    // throtramp

    if (pot < 1500) {
        throtRamp = .45;
    }
    else if (pot >= 1500 && pot < 3000) {
        throtRamp = map(pot, 1500, 3700, 1, 25);
    }
    else {
        throtRamp = 25;

    } 
    canSet(49, throtRamp);
    //slipstart
    slipstart = 32;
    canSet(52, slipstart);

    //ampmin
    ampMin = 1.1;
    canSet(51, ampMin);
}


void powerCalc() {

    power = ((packVolt * amps) * 1.32) / 1000;
    if (power > peakPower) {
        peakPower = power;
    }
    if (mtemp > peakTemp) {
        peakTemp = mtemp;
    }
    if (amps > peakAmps) {
        peakAmps = amps;
    }
    if (run == 0){
        peakPower = 0;
        peakTemp = 0;
        peakAmps = 0;
    }
}


void boostMap()
{
    
    if (pot > 3700) {
        boost = map(pot, 3700, 4095, minBoost, maxBoost);       
        canSet(0,boost);   
    }
    else {
        boost = minBoost;
        canSet(0,boost);
    }
}

void regenStuff() {

    // This method allows both pedal off regen in combination with variable pot2 value. Previously was not able to achieve this with recent firmware.

    baseRegen = 0; //base throttle off regen value
    maxRegen = 94;  //maximum full brake pressure regen value
   

    if (brakePedal > 700 ) {
        brkNomPedal = ((neg - (maxRegen * 32))/32); //sets POT2 value for maximum regen
    }
    else {
        brkNomPedal = map(brakePedal, 1, 700, ((neg - (baseRegen * 32))/32), ((neg - (maxRegen * 32)))/32); //maps brake pedal regen between base and max
    }
    canSet(53, brkNomPedal);

    //incline compensation
    if (brakePedal < 3) {
        if (angleAvg.get() >= 550 && angleAvg.get() < 600) {
            baseRegen = map(angleAvg.get(), 550, 600, 10, 45);
        }
        else if (angleAvg.get() >= 600) {
            baseRegen = 45;
        }
        else baseRegen = 10;
    }

    //regenramp
    
    if (rpm <= 10000) {
        regenRamp = map(rpm, 0, 10000, (32 * .1), (32 * .4));
    }
    else regenRamp = (.4 * 32);
    canSet(54, (regenRamp / 32));
    

    
    //brakemax
    brkMax = ((neg - (baseRegen * 32))/32);
    canSet(56, brkMax);
}


void idleThrottle() {
    
    if (run == 0 ) {
        canSet(64,1);  // Sets idlemode to brakeoff when inverter is not in opmode "run"
    }
    else {
        canSet(64, 0); //Sets idlemode to alwayson when inverter is in "run"
    }
 
    idleRamp.add(brakePedal);
    idleThrot = map(idleRamp.get(), 1, 400, idleThrotMax, 0);
    canSet(63, idleThrot);
    
    //incline compensation
    if (dir == 255) {
        
        if (angleAvg.get() < 470) {

            idleThrotMax = map(angleAvg.get(), 410, 470, 27, 20);
        }
        else {
            idleThrotMax = 20;
        }
        

        //idleThrotMax = 20;
            
    }
    else {
        //idleThrotMax = map(angleAvg.get(), 450, 620, 8, 27);
        idleThrotMax = 15;
    }
      
    idleThrotMax = 22;
    
    idleRPM = 1750; 
    canSet(62, idleRPM);
}


void modeSwitch() {
    if (digitalRead(driveMode) == LOW) {
      chill();      
    }
    if (digitalRead(driveMode) == HIGH) {
       party();   
    }
}


void canread()
{
    char msgString[128];

    digitalWrite(led, HIGH);
    Can0.read(inMsg);
    Serial.print(millis());
    if ((inMsg.id & 0x80000000) == 0x80000000)    // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (inMsg.id & 0x1FFFFFFF), inMsg.len);
    else
        sprintf(msgString, ",0x%.3lX,false,%1d", inMsg.id, inMsg.len);

    Serial.print(msgString);

    if ((inMsg.id & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        Serial.print(msgString);
    }
    else {
        for (byte i = 0; i < inMsg.len; i++) {
            sprintf(msgString, ", 0x%.2X", inMsg.buf[i]);
            Serial.print(msgString);
        }
    }


    Serial.println();
    digitalWrite(led, LOW);
}

void powerSteer() {
    if (run == 1) {
        digitalWrite(steerInit, HIGH);
    }
    else {
        digitalWrite(steerInit, LOW);
    }

}

void shift_dir() {

    /*
    can rx canio 300 0 5 32
    Bit 0: cruise
    Bit 1: start
    Bit 2: brake
    Bit 3: forward
    Bit 4: reverse
    Bit 5: bms
    00010000 Forward 0x10
    00001000 Reverse 0x08
    */



    switch (screen)
    {
    case 7: //reverse
        shift = REVERSE;
        break;

    case 9: //reverse
        shift = REVERSE;
        break;

    case 6: //nuetral
        shift = NUETRAL;
        break;

    case 11: //nuetral
        shift = NUETRAL;
        break;

    case 8: // forward
        shift = FORWARD;
        break;

    case 10: // forward
        shift = FORWARD;
        break;

    default:
        break;
    }


    switch (shift)
    {
    case FORWARD:
        msg.id = 0x113;
        msg.len = 1;
        msg.buf[0] = 0x10;
        Can0.write(msg);
        break;

    case NUETRAL:
        msg.id = 0x113;
        msg.len = 1;
        msg.buf[0] = 0x00;
        Can0.write(msg);
        break;

    case REVERSE:
        msg.id = 0x113; //275 dec
        msg.len = 1;
        msg.buf[0] = 0x08;
        Can0.write(msg);
        break;

    default:
        break;
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

/* Serial3.print("Drive Mode: ");
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
 else if (r estart == 1) {
     Serial3.println("UDC Low");
 }
 else if (restart == 2) {
     Serial3.println("UDC High");
 }  */




