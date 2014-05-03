#include  "Comp6DOF_n0m1.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include "HMC5883L.h"

void setup();
void loop();
void getOffset();
void getData();
void getAngle();
void getAzimuth();

MPU6050 accelgyro; // LEFT FOOT
MPU6050 accelgyro1(0x69); // RIGHT FOOT

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t axL, ayL, azL;
int16_t gxL, gyL, gzL;
int16_t axR, ayR, azR;
int16_t gxR, gyR, gzR;

int16_t axOffR, ayOffR, azOffR;
int16_t gxOffR, gyOffR, gzOffR;
int16_t axOffL, ayOffL, azOffL;
int16_t gxOffL, gyOffL, gzOffL;
int16_t mxOff, myOff, mzOff;

boolean  flag=1;

float AxR, AyR, AzR;
float GxR, GyR, GzR;
float AxL, AyL, AzL;
float GxL, GyL, GzL;
float angle_z;
float prev_angle_z;
float fAngle_z;
float fAngle_zold;
float YawU;
float Yaw;
float YawOffset;
float yawRaw=0;
float yawFiltered=0;
float yawFilteredOld=0;

float alpha = 0.5;
float dt = 0.00;

float  alphaYaw  =  0.2;
float prevGyro_x = 0.00;
float prevGyro_y = 0.00;
float prevGyro_x_L = 0.00;
float prevGyro_y_L = 0.00;
float prevGyro_z = 0.00;
float prevGyro_z_2 = 0.00;
float prevGyro_z_3 = 0.00;
float prevGyro_z_4 = 0.00;
float prevGyro_z_5 = 0.00;
float  gyro_x  =  0.00;
float  gyro_y  =  0.00;
float  gyro_z  =  0.00;
float angle_x = 0.00;
float angle_y = 0.00;
float accAngle_x =0.00;
float accAngle_y =0.00;
float accAngle_z =0.00;
unsigned int timer=0;
#define  address  0x1E  //0011110b,  I2C  7bit  address  of  HMC5883

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  // initialize serial communication
  Serial.begin(57600);
  flag=1;
  // initialize device
  accelgyro.initialize();
  accelgyro1.initialize();
  // verify connection
  getOffsetR();
  getOffsetL();
}
void loop() {
  getAngle();
  timer=timer+1;
  delay(10);
}

void getOffsetR() {
  for(int i=0; i<20; i++)
  {
    accelgyro1.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    axOffR+=ax;
    ayOffR+=ay;
    azOffR+=az;
    gxOffR+=gx;
    gyOffR+=gy;
    gzOffR+=gz;
  }

  gxOffR/=20;
  gyOffR/=20;
  gzOffR/=20;
  axOffR/=20;
  ayOffR/=20;
  azOffR/=20;
}
void getOffsetL() {
  for(int i=0; i<20; i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    axOffL+=ax;
    ayOffL+=ay;
    azOffL+=az;
    gxOffL+=gx;
    gyOffL+=gy;
    gzOffL+=gz;
  }

  gxOffL/=20;
  gyOffL/=20;
  gzOffL/=20;
  axOffL/=20;
  ayOffL/=20;
  azOffL/=20;
}

void getDataL() {
 accelgyro.getMotion6(&axL, &ayL, &azL, &gxL, &gyL, &gzL); // Left
  // LEFT
  GxL=((gxL-gxOffL)/16.40);
  GyL=((gyL-gyOffL)/16.40);
  GzL=((gzL-gzOffL)/16.40);
  AxL=((axL-axOffL)/2048.00);
  AyL=((ayL-ayOffL)/2048.00);
  AzL=((azL-azOffL)/2048.00)+1;	// Az=Ay;	Ay=Az
  unsigned int time=millis();
  Serial.print("L");
  Serial.print("\t");
  Serial.print(timer);
  Serial.print("\t");
  Serial.print(AxL);
  Serial.print("\t");
  Serial.print(AyL);
  Serial.print("\t");
  Serial.print(AzL);
  Serial.print("\t");
  Serial.print(GxL);
  Serial.print("\t");
  Serial.print(GyL);
  Serial.print("\t");
  Serial.print(GzL);
  Serial.print("\t"); 
}
  
  void getDataR() {
  accelgyro1.getMotion6(&axR, &ayR, &azR, &gxR, &gyR, &gzR);  // Right
  // RIGHT
  GxR=((gxR-gxOffR)/16.40);
  GyR=((gyR-gyOffR)/16.40);
  GzR=((gzR-gzOffR)/16.40);
  AxR=((axR-axOffR)/2048.00);
  AyR=((ayR-ayOffR)/2048.00);
  AzR=((azR-azOffR)/2048.00)+1;	// Az=Ay;	Ay=Az
  unsigned int time=millis();
  Serial.print("R");
  Serial.print("\t");
  Serial.print(timer);
  Serial.print("\t");
  Serial.print(AxR);
  Serial.print("\t");
  Serial.print(AyR);
  Serial.print("\t");
  Serial.print(AzR);
  Serial.print("\t");
  Serial.print(GxR);
  Serial.print("\t");
  Serial.print(GyR);
  Serial.print("\t");
  Serial.print(GzR);
  Serial.print("\t");
}

void getAngle() {
  getDataR();
  dt  =  millis()-dt;
  accAngle_x  =  atan2(AxR,  sqrt(AyR*AyR  +  AzR*AzR))*180/PI;
  accAngle_y  =  atan2(AyR,  sqrt(AxR*AxR  +  AzR*AzR))*180/PI;
  accAngle_z  =  atan2(-AyR,AxR)*180/PI;
  // Implement complementary filter
  angle_x   =   (1-alpha)*(angle_x+(GxR-prevGyro_x)*dt/1000)   +   (alpha)*accAngle_x;
  prevGyro_x = GxR;
  angle_y   =   (1-alpha)*(angle_y+(GyR-prevGyro_y)*dt/1000)   +   (alpha)*accAngle_y;
  prevGyro_y = GyR;
  angle_z  =  Yaw;
  delay(10);
  dt=millis();
  //serial transmission
  Serial.print(angle_x);
  Serial.print("\t");
  Serial.println(angle_y);


  getDataL();
  dt  =  millis()-dt;
  accAngle_x  =  atan2(AxL,  sqrt(AyL*AyL  +  AzL*AzL))*180/PI;
  accAngle_y  =  atan2(AyL,  sqrt(AxL*AxL  +  AzL*AzL))*180/PI;
  accAngle_z  =  atan2(-AyL,AxL)*180/PI;
  // Implement complementary filter
  angle_x_L   =   (1-alpha)*(angle_x_L+(GxL-prevGyro_x_L)*dt/1000)   +   (alpha)*accAngle_x;
  prevGyro_x_L = GxL;
  angle_y_L   =   (1-alpha)*(angle_y_L+(GyL-prevGyro_y_L)*dt/1000)   +   (alpha)*accAngle_y;
  prevGyro_y_L = GyL;
  angle_z  =  Yaw;
  delay(10);
  dt=millis();
  //serial transmission
  Serial.print(angle_x_L);
  Serial.print("\t");
  Serial.println(angle_y_L);
}

