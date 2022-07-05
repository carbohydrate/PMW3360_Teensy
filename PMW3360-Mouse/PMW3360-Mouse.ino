#include <SPI.h>
#include <Mouse.h>
#include <Bounce2.h>

#define numButtons 2

// Registers
#define Product_ID 0x00
#define Revision_ID 0x01
#define Motion 0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Raw_Data_Sum 0x08
#define Maximum_Raw_data 0x09
#define Minimum_Raw_data 0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Control 0x0D
#define Config1 0x0F
#define Config2 0x10
#define Angle_Tune 0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower 0x15
#define Rest1_Rate_Upper 0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower 0x18
#define Rest2_Rate_Upper 0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower 0x1B
#define Rest3_Rate_Upper 0x1C
#define Observation 0x24
#define Data_Out_Lower 0x25
#define Data_Out_Upper 0x26
#define Raw_Data_Dump 0x29
#define SROM_ID 0x2A
#define Min_SQ_Run 0x2B
#define Raw_Data_Threshold 0x2C
#define Config5 0x2F
#define Power_Up_Reset 0x3A
#define Shutdown 0x3B
#define Inverse_Product_ID 0x3F
#define LiftCutoff_Tune3 0x41
#define Angle_Snap 0x42
#define LiftCutoff_Tune1 0x4A
#define Motion_Burst 0x50
#define LiftCutoff_Tune_Timeout 0x58
#define LiftCutoff_Tune_Min_Length 0x5A
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst 0x64
#define LiftCutoff_Tune2 0x65

const int resetPin = 8; //Optional (not implemented)
const int chipSelectPin = 10;

bool buttonsState[numButtons] = {false, false};
const int mouse1 = 7;
const int mouse2 = 5;
//Bounce b = Bounce();
Bounce2::Button button1 = Bounce2::Button();
Bounce2::Button button2 = Bounce2::Button();

byte initComplete = 0;
unsigned long lastTS;
int16_t dx, dy;

extern const unsigned short firmware_length;
extern const unsigned char firmware_data[];

//If any of your settings are variables, you may create a SPISettings object to hold the 3 settings. Then you can give the object name to SPI.beginTransaction(). Creating a named SPISettings object may be more efficient when your settings are not constants, especially if the maximum speed is a variable computed or configured, rather than a number you type directly into your sketch.
//move this to SPI.beginTransaction later
//PMW3360 says 2.0MHz 50% duty cycle
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE3);

void setup() {
  Serial.begin(9600);
  //pinMode(resetPin, INPUT_PULLUP);
  pinMode(chipSelectPin, OUTPUT);

  //pinMode(7, INPUT_PULLUP);
  //pinMode(5, INPUT_PULLUP);

  button1.attach(mouse1, INPUT_PULLUP);
  button1.interval(25);  //25ms
  button1.setPressedState(LOW);

  button2.attach(mouse2, INPUT_PULLUP);
  button2.interval(25);  //25ms
  button2.setPressedState(LOW);
  
  SPI.begin();
  Serial.println("ON");
  powerUp();

  //delay(5000);

  //dispRegisters();
  initComplete=9;
}

void loop() {
  unsigned long elapsed = micros() - lastTS;

  //.5ms
  if(elapsed > 500) {
  //if(elapsed > 500000) {
    //burst info page 22
    byte burstBuffer[12];
    //Serial.println(elapsed);

    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
    chipSelectLow();

    SPI.transfer(Motion_Burst);
    delayMicroseconds(35); // waits for tSRAD_MOTBR

    SPI.transfer(burstBuffer, 12); // read burst buffer
    delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns

    chipSelectHigh();
    SPI.endTransaction();

    /*
    BYTE[00] = Motion    = if the 7th bit is 1, a motion is detected.
           ==> 7 bit: MOT (1 when motion is detected)
           ==> 3 bit: 0 when chip is on surface / 1 when off surface
           ] = Observation  
    BYTE[02] = Delta_X_L = dx (LSB)
    BYTE[03] = Delta_X_H = dx (MSB) 
    BYTE[04] = Delta_Y_L = dy (LSB)
    BYTE[05] = Delta_Y_H = dy (MSB)
    BYTE[06] = SQUAL     = Surface Quality register, max 0x80
                         - Number of features on the surface = SQUAL * 8
    BYTE[07] = Raw_Data_Sum   = It reports the upper byte of an 18‐bit counter which sums all 1296 raw data in the current frame;
                               * Avg value = Raw_Data_Sum * 1024 / 1296
    BYTE[08] = Maximum_Raw_Data  = Max raw data value in current frame, max=127
    BYTE[09] = Minimum_Raw_Data  = Min raw data value in current frame, max=127
    BYTE[10] = Shutter_Upper     = Shutter LSB
    BYTE[11] = Shutter_Lower     = Shutter MSB, Shutter = shutter is adjusted to keep the average raw data values within normal operating ranges
    */

    int motion = (burstBuffer[0] & 0x80) > 0;
    //int surface = (burstBuffer[0] & 0x08) > 0;   // 0 if on surface / 1 if off surface
 
    int xl = burstBuffer[2];
    int xh = burstBuffer[3];
    int yl = burstBuffer[4];
    int yh = burstBuffer[5];

    //int squal = burstBuffer[6];

    int x = xh<<8 | xl;
    int y = yh<<8 | yl;

    dx += x;
    dy += y;

    //chipSelectHigh();

    if(motion) {
      //Serial.println(dx);
      //Serial.println(dy);
      signed char mdx = constrain(dx, -127, 127);
      signed char mdy = constrain(dy, -127, 127);
      
      Mouse.move(mdx, mdy, 0);
      
      dx = 0;
      dy = 0;
    }
    lastTS = micros();
  }
  
  button1.update(); // Update the Bounce instance
  button2.update();
  //We set setPressedState to LOW in setup as the default is HIGH, so the pressed state is set when LOW.
  if(button1.isPressed() && buttonsState[0] == false) {
    Mouse.press(MOUSE_LEFT);
    buttonsState[0] = true;
    Serial.println("BUTTON1 PRESSED BOUCE");
  } else if(!button1.isPressed() && buttonsState[0]) {
    Mouse.release(MOUSE_LEFT);
    buttonsState[0] = false;
    Serial.println("BUTTON1 PRESSED BOUCE RELEASE");
  }
  
  if(button2.isPressed() && buttonsState[1] == false) {
    Mouse.press(MOUSE_RIGHT);
    buttonsState[1] = true;
    Serial.println("BUTTON2 PRESSED BOUCE");
  } else if(!button2.isPressed() && buttonsState[1]) {
    Mouse.release(MOUSE_RIGHT);
    buttonsState[1] = false;
    Serial.println("BUTTON2 PRESSED BOUCE RELEASE");
  }
  
  //byte mo = readReg(Motion);
  //byte dxl = readReg(Delta_X_L);
  //byte dxh = readReg(Delta_X_H);
  //byte dyl = readReg(Delta_Y_L);
  //byte dyh = readReg(Delta_Y_H);

  //Serial.println("LOOP REGISTERS 0X02-0X06");
  //Serial.println(mo, BIN);
  //Serial.println(dxl);
  //Serial.println(dxh);
  //Serial.println(dyl);
  //Serial.println(dyh);

  //delay(10);
}

//page 26 of datasheet
void powerUp(void) {
  //chipSelectHigh();
  //chipSelectLow();
  //chipSelectHigh();
  //writeReg(Shutdown, 0xb6); // Shutdown first
  //delay(300);

  chipSelectLow();
  delayMicroseconds(40);
  chipSelectHigh();
  delayMicroseconds(40);

  writeReg(Power_Up_Reset, 0x5A); // force reset
  delay(50); // wait for it to reboot

  // read registers 0x02 to 0x06 (and discard the data)
  byte mo = readReg(Motion);
  byte dxl = readReg(Delta_X_L);
  byte dxh = readReg(Delta_X_H);
  byte dyl = readReg(Delta_Y_L);
  byte dyh = readReg(Delta_Y_H);

  Serial.println("STARTUP REGISTERS 0X02-0X06");
  Serial.println(mo);
  Serial.println(dxl);
  Serial.println(dxh);
  Serial.println(dyl);
  Serial.println(dyh);

  byte mo1 = readReg(Motion);
  Serial.println(mo1);
  
  // upload the firmware
  uploadFirmware();
  delay(10);

  //setCPI(CPI);
  Serial.println("Optical Chip Initialized");
}

//adns_com_begin()
void chipSelectLow() {
  //active low
  digitalWrite(chipSelectPin, LOW);
}

//adns_com_end()
void chipSelectHigh() {
  //bring high to disable
  digitalWrite(chipSelectPin, HIGH);
}

byte readReg(byte addr) {
  SPI.beginTransaction(spiSettings);
  chipSelectLow();

  // send adress of the register, with MSBit = 0 to indicate it's a read
  //Serial.println("readReg");
  //Serial.println(addr);
  //Serial.println(0x7F);
  //Serial.println(addr & 0x7F);  //0111 1111
  //Serial.println(addr & 0x7F);  //0111 1111
  SPI.transfer(addr & 0x7F);
  delayMicroseconds(35); // tSRAD
  //read data
  byte data = SPI.transfer(0);

  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  chipSelectHigh();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS
  SPI.endTransaction();

  //Serial.print("readRegData: ");
  //Serial.println(data, BIN);

  return data;
}

void writeReg(byte addr, byte data) {
  chipSelectLow();
  SPI.beginTransaction(spiSettings);
  

  //Serial.print("writeReg: ");
  //Serial.println(addr);
  //Serial.println(data);

  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(addr | 0x80);
  //sent data
  SPI.transfer(data);

  delayMicroseconds(20); // tSCLK-NCS for write operation
  chipSelectHigh();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound
  SPI.endTransaction();
}

//page 23
void uploadFirmware() {
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware...");

  //Write 0 to Rest_En bit of Config2 register to disable Rest mode.
  writeReg(Config2, 0x00);

  // write 0x1d in SROM_enable reg for initializing
  writeReg(SROM_Enable, 0x1D);

  // wait for more than one frame period
  delay(10); //datasheet says 10ms

  // write 0x18 to SROM_enable to start SROM download
  writeReg(SROM_Enable, 0x18);

  // write the SROM file (=firmware data)
  SPI.beginTransaction(spiSettings);
  chipSelectLow();
  SPI.transfer(SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);

  // send all bytes of the firmware
  unsigned char c;
  for (int i = 0; i < firmware_length; i++) {
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  chipSelectHigh();
  SPI.endTransaction();

  //Read the SROM_ID register to verify the ID before any other register reads or writes.
  readReg(SROM_ID);
  //byte test = readReg(SROM_ID);
  //Serial.print("SROM_ID: ");
  //Serial.println(test, BIN);
  //Serial.println(test, HEX);

  //Write 0x00 (rest disable) to Config2 register for wired mouse or 0x20 for wireless mouse design. 
  writeReg(Config2, 0x00);

  //chipSelectHigh();
}

/*
void dispRegisters(void){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x02  };
  char* oregname[] = {"Product_ID","Inverse_Product_ID","SROM_Version","Motion"};
  byte regres;

  byte testSReg = readReg(SROM_ID);
  Serial.println(testSReg, BIN);
  Serial.println(testSReg, HEX);

  SPI.beginTransaction(spiSettings);
  chipSelectLow();

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    Serial.println(regres,BIN);  
    Serial.println(regres,HEX);  
    delay(1);
  }
  chipSelectHigh();
  SPI.endTransaction();
}
*/
