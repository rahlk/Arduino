//Arduino 1.0+ only

#include <Wire.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D
int sensitivity;
int xx;
int yy;
int zz;

#define xAxis 3
#define yAxis 1
#define zAxis 0

float x;
float y;
float z;

float xOff=0.00f;
float yOff=0.00f;
float zOff=0.00f;

float xaOffset=0;
float yaOffset=0;
float zaOffset=0;

float xaRaw=0;
float yaRaw=0;
float zaRaw=0;

void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  xx = ((xMSB << 8) | xLSB);
  x = xx*getScale();
  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  yy = ((yMSB << 8) | yLSB);
  y = yy*getScale();
  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  zz = ((zMSB << 8) | zLSB);
  z = zz*getScale();
}

float getScale()
{
  if(sensitivity==2000)
    return 0.07f;
  else if(sensitivity==500)
    return 0.0175f;
  else if(sensitivity==250)
    return 0.00875f;
}

void getOffset() {
  for(int i=0; i<60; i++) {
    getGyroValues();
    xOff+=x;
    yOff+=y;
    zOff+=z;
  }
  xOff/=60;
  yOff/=60;
  zOff/=60;
}
int setupL3G4200D(int scale){

  sensitivity = scale;
  //From  Jim Lindblom of Sparkfun's code
  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b11001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }
  else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }
  else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
//  Serial.print("AA");
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}
void getAccOffset()
{ 
  for (int i=1; i <= 60; i++){        
    xaOffset += analogRead(3);    
    yaOffset += analogRead(1);
    zaOffset += analogRead(0);
  }
  xaOffset /=60;  
  yaOffset /=60;
  zaOffset /=60;
}

