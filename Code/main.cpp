#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>
#include <SwitecX12.h>            //for stepper motors
#include <Adafruit_GFX.h>         //it looks like this is needed for i2c 7 seg
#include <Adafruit_LEDBackpack.h> //for segmented display
#include <SoftwareSerial.h>       //using software serial for debugging
#include <comms.h> //contains functions for speeduino serial data

// Board Constants
#define RESET 17
#define CLT_DIR 2
#define CLT_STEP 3
#define CLT_LED 13
#define OIL_DIR 8
#define OIL_STEP 9
#define OIL_LED 14
#define GAS_DIR 6
#define GAS_STEP 7
#define GAS_LED 16
#define AFR_DIR 4
#define AFR_STEP 5
#define AFR_LED 15
#define RPM_DIR 10
#define RPM_STEP 11
#define RPM_LED 12
#define AUX1_IN 20 // A6
#define AUX2_IN 21 // A7
#define SDA 18
#define SCL 19

// program constants
const int STEPS = 315 * 12;
#define CLT_low 150
#define CLT_high 250
#define CLT_warn_low 100
#define CLT_warn_high 220

#define OIL_low 0
#define OIL_high 1000
#define OIL_warn_low 50
#define OIL_warn_high 1000

#define GAS_low 0
#define GAS_high 100
#define GAS_warn_low 25
#define GAS_warn_high 110 //essentially never

#define AFR_low 1000
#define AFR_high 2000
#define AFR_warn_low 1000
#define AFR_warn_high 1700

#define RPM_low 0
#define RPM_high 8000
#define RPM_warn_low 200
#define RPM_warn_high 7000

// global variables
int AFR = AFR_low;
int GAS = GAS_low;
int CLT = CLT_low;
int RPM = RPM_low;
int OIL = OIL_low;
int BAT = 0;

// define Motors
SwitecX12 motor_CLT(STEPS, CLT_STEP, CLT_DIR);
SwitecX12 motor_OIL(STEPS, OIL_STEP, OIL_DIR);
SwitecX12 motor_GAS(STEPS, GAS_STEP, GAS_DIR);
SwitecX12 motor_AFR(STEPS, AFR_STEP, AFR_DIR);
SwitecX12 motor_RPM(STEPS, RPM_STEP, RPM_DIR);

// Seven Segment Display Setup
Adafruit_7segment speedo = Adafruit_7segment();

// Functions
//SoftwareSerial SoftSerial(AUX1_IN, AUX2_IN); // RX, TX (7 & 8 on chassis connector)
//#define  SoftSerial //choose if speeduino is on software or hardware port
#define SerialSpeeduino Serial //choose if debug is on software or hardware port
void startupSerial(){
  SerialSpeeduino.begin(115200);
  //SerialDebug.begin(115200);
}

void allMotorReset(){

  motor_CLT.setPosition(STEPS);
  motor_OIL.setPosition(STEPS);
  motor_GAS.setPosition(STEPS);
  motor_AFR.setPosition(STEPS);
  motor_RPM.setPosition(STEPS);

  motor_CLT.update();
  motor_OIL.update();
  motor_GAS.update();
  motor_AFR.update();
  motor_RPM.update();

  while(!motor_CLT.stopped && !motor_OIL.stopped && !motor_GAS.stopped && !motor_AFR.stopped && !motor_RPM.stopped){
    motor_CLT.update();
    motor_OIL.update();
    motor_GAS.update();
    motor_AFR.update();
    motor_RPM.update();
  }

  motor_CLT.setPosition(0);
  motor_OIL.setPosition(0);
  motor_GAS.setPosition(0);
  motor_AFR.setPosition(0);
  motor_RPM.setPosition(0);
  
  motor_CLT.update();
  motor_OIL.update();
  motor_GAS.update();
  motor_AFR.update();
  motor_RPM.update();

  while(!motor_CLT.stopped && !motor_OIL.stopped && !motor_GAS.stopped && !motor_AFR.stopped && !motor_RPM.stopped){
    motor_CLT.update();
    motor_OIL.update();
    motor_GAS.update();
    motor_AFR.update();
    motor_RPM.update();
  }
}

void updateMotors()
{
  int temp = map(CLT, CLT_low, CLT_high, 0, STEPS);
  temp = constrain(temp,0,STEPS);
  motor_CLT.setPosition(temp);
  motor_CLT.update();

  temp = map(OIL, OIL_low, OIL_high, 0, STEPS);
  temp = constrain(temp,0,STEPS);
  motor_OIL.setPosition(temp);
  motor_OIL.update();

  temp = map(GAS, GAS_low, GAS_high, 0, STEPS);
  temp = constrain(temp,0,STEPS);
  motor_GAS.setPosition(temp);
  motor_GAS.update();

  temp = map(AFR, AFR_low, AFR_high, 0, STEPS);
  temp = constrain(temp,0,STEPS);
  motor_AFR.setPosition(temp);
  motor_AFR.update();

  temp = map(RPM, RPM_low, RPM_high, 0, STEPS);
  temp = constrain(temp,0,STEPS);
  motor_RPM.setPosition(temp);
  motor_RPM.update();
}

/*commented out section for testing values instead of reading from Serial
bool CLTup = true; //global values for test values function
bool OILup = true;
bool RPMup = true;
bool AFRup = true;
bool GASup = true;
void testValues()
{
  // test sweep
  if (CLT == CLT_high){CLTup = false;}
  else if (CLT == CLT_low){CLTup = true;}
  if (CLTup){CLT++;}
  else{CLT--;}

  if (OIL == OIL_high){OILup = false;}
  else if (OIL == OIL_low){OILup = true;}
  if (OILup){OIL++;}
  else{OIL--;}

  if (AFR == AFR_high){AFRup = false;}
  else if (AFR == AFR_low){AFRup = true;}
  if (AFRup){AFR=AFR + .1;}
  else{AFR = AFR - .1;}

  if (GAS == GAS_high){GASup = false;}
  else if (GAS == GAS_low){GASup = true;}
  if (GASup){GAS++;}
  else{GAS--;}

  if (RPM == RPM_high){RPMup = false;}
  else if (RPM == RPM_low){RPMup = true;}
  if (RPMup){RPM++;}
  else{RPM--;}
}
*/

bool warnCheck(int VAR, int VAR_low, int VAR_high)
{
  bool result = true;
  if (VAR <= VAR_low)
  {
    result = true;
  }
  else if (VAR >= VAR_high)
  {
    result = true;
  }
  else
  {
    result = false;
  }
  return result;
}

void warnLED()
{
  if (warnCheck(CLT, CLT_warn_low, CLT_warn_high))
  {
    digitalWrite(CLT_LED, LOW);
  }
  else
  {
    digitalWrite(CLT_LED, HIGH);
  }

  if (warnCheck(OIL, OIL_warn_low, OIL_warn_high))
  {
    digitalWrite(OIL_LED, LOW);
  }
  else
  {
    digitalWrite(OIL_LED, HIGH);
  }

  if (warnCheck(RPM, RPM_warn_low, RPM_warn_high))
  {
    digitalWrite(RPM_LED, LOW);
  }
  else
  {
    digitalWrite(RPM_LED, HIGH);
  }

  if (warnCheck(GAS, GAS_warn_low, GAS_warn_high))
  {
    digitalWrite(GAS_LED, LOW);
  }
  else
  {
    digitalWrite(GAS_LED, HIGH);
  }

  if (warnCheck(AFR, AFR_warn_low, AFR_warn_high))
  {
    digitalWrite(AFR_LED, LOW);
  }
  else
  {
    digitalWrite(AFR_LED, HIGH);
  }
}

void readSpeeduino()
{
  // serial operation, frequency based request
  static uint32_t lastUpdate = millis();
  if (millis() - lastUpdate > 100) //too quick and it will mess up stepper control
  {
    requestData(50);
    lastUpdate = millis();
  }

  // get refresh rate
  static uint32_t lastRefresh = millis();
  uint16_t refreshRate = 1000 / (millis() - lastRefresh); //not currently using but here just incase
  lastRefresh = millis();

  //pull data, use getword for ones with two bytes and getbyte for just bytes
  RPM = getWord(14); //engine speed in RPM
  CLT = getByte(7)-40;  //coolant in deg C, subtracting coolant offset to allow neg value
  CLT = (1.8*CLT)+32; //convert to deg F
  OIL = getWord(41); //oil pressure, from AUX 0
  OIL = map(OIL,0,1023,-30,1030); //convert OIL into 0-100psi, x10 for better resolution
  OIL = constrain(OIL,0,1000);
  AFR = getByte(10); //AFR based on calibration range 9-19 in my case
  AFR = map(AFR,0,255,900,1900); //convert AFR into 9-19 based on calibration for 0-5v, also x100 to get decimal place into long variable
  AFR = constrain(AFR,900,1900);
  GAS = getWord(43); //fuel level AUX input 1, coming in 0-1023 range
  GAS = map(GAS,49,532,100,0); //convert gas into 0-100% based on calibration data
  GAS = constrain(GAS,0,100);
  BAT = getByte(9); //battery voltage * 10

  //addresses that aren't used
  //getWord(4); MAP
  //getByte(9); Battery voltage
  //getWord(100); Speed in KPH
  //(int8_t)getByte(23); ignition advance
  //(int8_t)getByte(3); injector dwell
}

void setupLEDs(){
  //Setup Pins
  pinMode(CLT_LED, OUTPUT);
  pinMode(OIL_LED, OUTPUT);
  pinMode(AFR_LED, OUTPUT);
  pinMode(GAS_LED, OUTPUT);
  pinMode(RPM_LED, OUTPUT);
  // flash LEDs
  digitalWrite(CLT_LED, LOW);
  digitalWrite(OIL_LED, LOW);
  digitalWrite(AFR_LED, LOW);
  digitalWrite(GAS_LED, LOW);
  digitalWrite(RPM_LED, LOW);
  // startup 7 seg display
  speedo.begin(0x70);
  speedo.print("8888");
  speedo.writeDisplay();
  //delay(500);
  digitalWrite(CLT_LED, HIGH);
  digitalWrite(OIL_LED, HIGH);
  digitalWrite(AFR_LED, HIGH);
  digitalWrite(GAS_LED, HIGH);
  digitalWrite(RPM_LED, HIGH);
  speedo.clear();
  speedo.writeDisplay();
}

void setup()
{
  digitalWrite(LED_BUILTIN,HIGH);
  setupLEDs();
  startupSerial();
  digitalWrite(RESET, HIGH); // turn off reset of stepper chip
  speedo.print(8008);
  speedo.writeDisplay();
  allMotorReset();
  
}
const int freq = 500; //display update time in milliseconds
unsigned long prevUpdate = 0;
void loop()
{
  //testValues();
  readSpeeduino();
  warnLED();
  updateMotors();

//only update the display at lower frequency, too high frequency and it messes up the steppers
  if(freq < (millis()-prevUpdate)){
    speedo.print(BAT/10.0);
    speedo.writeDisplay();
    prevUpdate = millis();
  }
}

/*Reference of everything sent with 'n' command
fullStatus[0] = currentStatus.secl; //secl is simply a counter that increments each second. Used to track unexpected resets (Which will reset this count to 0)
  fullStatus[1] = currentStatus.status1; //status1 Bitfield, inj1Status(0), inj2Status(1), inj3Status(2), inj4Status(3), DFCOOn(4), boostCutFuel(5), toothLog1Ready(6), toothLog2Ready(7)
  fullStatus[2] = currentStatus.engine; //Engine Status Bitfield, running(0), crank(1), ase(2), warmup(3), tpsaccaen(4), tpsacden(5), mapaccaen(6), mapaccden(7)
  fullStatus[3] = (byte)(divu100(currentStatus.dwell)); //Dwell in ms * 10
  fullStatus[4] = lowByte(currentStatus.MAP); //2 bytes for MAP
  fullStatus[5] = highByte(currentStatus.MAP);
  fullStatus[6] = (byte)(currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET); //mat
  fullStatus[7] = (byte)(currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //Coolant ADC
  fullStatus[8] = currentStatus.batCorrection; //Battery voltage correction (%)
  fullStatus[9] = currentStatus.battery10; //battery voltage
  fullStatus[10] = currentStatus.O2; //O2
  fullStatus[11] = currentStatus.egoCorrection; //Exhaust gas correction (%)
  fullStatus[12] = currentStatus.iatCorrection; //Air temperature Correction (%)
  fullStatus[13] = currentStatus.wueCorrection; //Warmup enrichment (%)
  fullStatus[14] = lowByte(currentStatus.RPM); //rpm HB
  fullStatus[15] = highByte(currentStatus.RPM); //rpm LB
  fullStatus[16] = currentStatus.AEamount; //acceleration enrichment (%)
  fullStatus[17] = currentStatus.corrections; //Total GammaE (%)
  fullStatus[18] = currentStatus.VE; //Current VE 1 (%)
  fullStatus[19] = currentStatus.afrTarget;
  fullStatus[20] = lowByte(currentStatus.PW1); //Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[21] = highByte(currentStatus.PW1); //Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[22] = currentStatus.tpsDOT; //TPS DOT
  fullStatus[23] = currentStatus.advance;
  fullStatus[24] = currentStatus.TPS; // TPS (0% to 100%)
  //Need to split the int loopsPerSecond value into 2 bytes
  fullStatus[25] = lowByte(currentStatus.loopsPerSecond);
  fullStatus[26] = highByte(currentStatus.loopsPerSecond);

  //The following can be used to show the amount of free memory
  currentStatus.freeRAM = freeRam();
  fullStatus[27] = lowByte(currentStatus.freeRAM); //(byte)((currentStatus.loopsPerSecond >> 8) & 0xFF);
  fullStatus[28] = highByte(currentStatus.freeRAM);

  fullStatus[29] = (byte)(currentStatus.boostTarget >> 1); //Divide boost target by 2 to fit in a byte
  fullStatus[30] = (byte)(currentStatus.boostDuty / 100);
  fullStatus[31] = currentStatus.spark; //Spark related bitfield, launchHard(0), launchSoft(1), hardLimitOn(2), softLimitOn(3), boostCutSpark(4), error(5), idleControlOn(6), sync(7)

  //rpmDOT must be sent as a signed integer
  fullStatus[32] = lowByte(currentStatus.rpmDOT);
  fullStatus[33] = highByte(currentStatus.rpmDOT);

  fullStatus[34] = currentStatus.ethanolPct; //Flex sensor value (or 0 if not used)
  fullStatus[35] = currentStatus.flexCorrection; //Flex fuel correction (% above or below 100)
  fullStatus[36] = currentStatus.flexIgnCorrection; //Ignition correction (Increased degrees of advance) for flex fuel

  fullStatus[37] = currentStatus.idleLoad;
  fullStatus[38] = currentStatus.testOutputs; // testEnabled(0), testActive(1)

  fullStatus[39] = currentStatus.O2_2; //O2
  fullStatus[40] = currentStatus.baro; //Barometer value

  fullStatus[41] = lowByte(currentStatus.canin[0]);
  fullStatus[42] = highByte(currentStatus.canin[0]);
  fullStatus[43] = lowByte(currentStatus.canin[1]);
  fullStatus[44] = highByte(currentStatus.canin[1]);
  fullStatus[45] = lowByte(currentStatus.canin[2]);
  fullStatus[46] = highByte(currentStatus.canin[2]);
  fullStatus[47] = lowByte(currentStatus.canin[3]);
  fullStatus[48] = highByte(currentStatus.canin[3]);
  fullStatus[49] = lowByte(currentStatus.canin[4]);
  fullStatus[50] = highByte(currentStatus.canin[4]);
  fullStatus[51] = lowByte(currentStatus.canin[5]);
  fullStatus[52] = highByte(currentStatus.canin[5]);
  fullStatus[53] = lowByte(currentStatus.canin[6]);
  fullStatus[54] = highByte(currentStatus.canin[6]);
  fullStatus[55] = lowByte(currentStatus.canin[7]);
  fullStatus[56] = highByte(currentStatus.canin[7]);
  fullStatus[57] = lowByte(currentStatus.canin[8]);
  fullStatus[58] = highByte(currentStatus.canin[8]);
  fullStatus[59] = lowByte(currentStatus.canin[9]);
  fullStatus[60] = highByte(currentStatus.canin[9]);
  fullStatus[61] = lowByte(currentStatus.canin[10]);
  fullStatus[62] = highByte(currentStatus.canin[10]);
  fullStatus[63] = lowByte(currentStatus.canin[11]);
  fullStatus[64] = highByte(currentStatus.canin[11]);
  fullStatus[65] = lowByte(currentStatus.canin[12]);
  fullStatus[66] = highByte(currentStatus.canin[12]);
  fullStatus[67] = lowByte(currentStatus.canin[13]);
  fullStatus[68] = highByte(currentStatus.canin[13]);
  fullStatus[69] = lowByte(currentStatus.canin[14]);
  fullStatus[70] = highByte(currentStatus.canin[14]);
  fullStatus[71] = lowByte(currentStatus.canin[15]);
  fullStatus[72] = highByte(currentStatus.canin[15]);

  fullStatus[73] = currentStatus.tpsADC;
  fullStatus[74] = getNextError(); // errorNum (0:1), currentError(2:7)

  fullStatus[75] = currentStatus.launchCorrection;
  fullStatus[76] = lowByte(currentStatus.PW2); //Pulsewidth 2 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[77] = highByte(currentStatus.PW2); //Pulsewidth 2 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[78] = lowByte(currentStatus.PW3); //Pulsewidth 3 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[79] = highByte(currentStatus.PW3); //Pulsewidth 3 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[80] = lowByte(currentStatus.PW4); //Pulsewidth 4 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[81] = highByte(currentStatus.PW4); //Pulsewidth 4 multiplied by 10 in ms. Have to convert from uS to mS.

  fullStatus[82] = currentStatus.status3; // resentLockOn(0), nitrousOn(1), fuel2Active(2), vssRefresh(3), halfSync(4), nSquirts(6:7)
  fullStatus[83] = currentStatus.engineProtectStatus; //RPM(0), MAP(1), OIL(2), AFR(3), Unused(4:7)
  fullStatus[84] = lowByte(currentStatus.fuelLoad);
  fullStatus[85] = highByte(currentStatus.fuelLoad);
  fullStatus[86] = lowByte(currentStatus.ignLoad);
  fullStatus[87] = highByte(currentStatus.ignLoad);
  fullStatus[88] = lowByte(currentStatus.injAngle); 
  fullStatus[89] = highByte(currentStatus.injAngle); 
  fullStatus[90] = currentStatus.idleDuty;
  fullStatus[91] = currentStatus.CLIdleTarget; //closed loop idle target
  fullStatus[92] = currentStatus.mapDOT; //rate of change of the map 
  fullStatus[93] = (int8_t)currentStatus.vvt1Angle;
  fullStatus[94] = currentStatus.vvt1TargetAngle;
  fullStatus[95] = currentStatus.vvt1Duty;
  fullStatus[96] = lowByte(currentStatus.flexBoostCorrection);
  fullStatus[97] = highByte(currentStatus.flexBoostCorrection);
  fullStatus[98] = currentStatus.baroCorrection;
  fullStatus[99] = currentStatus.ASEValue; //Current ASE (%)
  fullStatus[100] = lowByte(currentStatus.vss); //speed reading from the speed sensor
  fullStatus[101] = highByte(currentStatus.vss);
  fullStatus[102] = currentStatus.gear; 
  fullStatus[103] = currentStatus.fuelPressure;
  fullStatus[104] = currentStatus.oilPressure;
  fullStatus[105] = currentStatus.wmiPW;
  fullStatus[106] = currentStatus.status4; // wmiEmptyBit(0), vvt1Error(1), vvt2Error(2), fanStatus(3), UnusedBits(4:7)
  fullStatus[107] = (int8_t)currentStatus.vvt2Angle;
  fullStatus[108] = currentStatus.vvt2TargetAngle;
  fullStatus[109] = currentStatus.vvt2Duty;
  fullStatus[110] = currentStatus.outputsStatus;
  fullStatus[111] = (byte)(currentStatus.fuelTemp + CALIBRATION_TEMPERATURE_OFFSET); //Fuel temperature from flex sensor
  fullStatus[112] = currentStatus.fuelTempCorrection; //Fuel temperature Correction (%)
  fullStatus[113] = currentStatus.VE1; //VE 1 (%)
  fullStatus[114] = currentStatus.VE2; //VE 2 (%)
  fullStatus[115] = currentStatus.advance1; //advance 1 
  fullStatus[116] = currentStatus.advance2; //advance 2 
  fullStatus[117] = currentStatus.nitrous_status;
  fullStatus[118] = currentStatus.TS_SD_Status; //SD card status
  fullStatus[119] = lowByte(currentStatus.EMAP); //2 bytes for EMAP
  fullStatus[120] = highByte(currentStatus.EMAP);
  fullStatus[121] = currentStatus.fanDuty;

*/