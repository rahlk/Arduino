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

MPU6050 accelgyro(0x69);
HMC5883L mag;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
int16_t mxMax=763, myMax=336, mzMax=375;
int16_t mxMin=-437, myMin=-715, mzMin=-573;

float mxMap, myMap, mzMap;

int16_t axOff, ayOff, azOff;
int16_t gxOff, gyOff, gzOff;
int16_t mxOff, myOff, mzOff;

boolean  flag=1;

float Ax, Ay, Az;
float Gx, Gy, Gz;
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

#define  address  0x1E  //0011110b,  I2C  7bit  address  of  HMC5883

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  // initialize serial communication
  Serial.begin(115200);

  flag=1;
  // initialize device
  accelgyro.initialize();
  // verify connection
  mag.initialize();

  getOffset();
}
void loop() {
  getAngle();
}

void getOffset() {
  for(int i=0; i<20; i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mag.getHeading(&mx, &my, &mz);
    axOff+=ax;
    ayOff+=ay;
    azOff+=az;
    gxOff+=gx;
    gyOff+=gy;
    gzOff+=gz;
  }

  gxOff/=20;
  gyOff/=20;
  gzOff/=20;
  axOff/=20;
  ayOff/=20;
  azOff/=20;
  // Magnetometer offset.

  mxOff  =  (mxMax+mxMin)/2;
  myOff  =  (myMax+myMin)/2;
  mzOff  =  (mzMax+mzMin)/2;
}

void getData() {
  mag.getHeading(&mx, &my, &mz);
  mx = mx - mxOff;
  my = my - myOff;
  mz = mz - mzOff;

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Gx=((gx-gxOff)/16.40);
  Gy=((gy-gyOff)/16.40);
  Gz=((gz-gzOff)/16.40);
  Ax=((ax-axOff)/2048.00);
  Ay=((ay-ayOff)/2048.00);
  Az=((az-azOff)/2048.00)+1;	// Az=Ay;	Ay=Az
  unsigned int time=millis();
  Serial.print(time);
  Serial.print("\t");
  Serial.print(Ax);
  Serial.print("\t");
  Serial.print(Ay);
  Serial.print("\t");
  Serial.print(Az);
  Serial.print("\t");
  Serial.print(Gx);
  Serial.print("\t");
  Serial.print(Gy);
  Serial.print("\t");
  Serial.print(Gz);
  Serial.print("\t");
  delay(10);
}

void getAngle() {
  getData();
  dt  =  millis()-dt;
  accAngle_x  =  atan2(Ax,  sqrt(Ay*Ay  +  Az*Az))*180/PI;
  accAngle_y  =  atan2(Ay,  sqrt(Ax*Ax  +  Az*Az))*180/PI;
  accAngle_z  =  atan2(-Ay,Ax)*180/PI;
  // Implement complementary filter
  angle_x   =   (1-alpha)*(angle_x+(Gx-prevGyro_x)*dt/1000)   +   (alpha)*accAngle_x;
  prevGyro_x = Gx;
  angle_y   =   (1-alpha)*(angle_y+(Gy-prevGyro_y)*dt/1000)   +   (alpha)*accAngle_y;
  prevGyro_y = Gy;
  getAzimuth();
  angle_z  =  Yaw;
  delay(20);
  dt=millis();
  //serial transmission
  Serial.print(angle_x);
  Serial.print("\t");
  Serial.println(angle_y);
}

void getAzimuth()
{
  //this part is required to normalize the magnetic vector
  //get Min  and  Max  Reading  for Magnetic  Axis
  if (mx>mxMax) {
    mxMax = mx; 
    getOffset(); 
  }
  if (my>myMax) {
    myMax = my; 
    getOffset(); 
  }
  if (mz>mzMax) {
    mzMax = mz; 
    getOffset(); 
  }

  if (mx<mxMin) {
    mxMin = mx; 
    getOffset();
  }
  if (my<myMin) {
    myMin = my; 
    getOffset();
  }
  if (mz<mzMin) {
    mzMin = mz; 
    getOffset();
  }

  //Map the incoming Data from -1 to 1
  mxMap  =  float(map(mx,  mxMin,  mxMax,  -1000,  1000))/1000.0;
  myMap  =  float(map(my,  myMin,  myMax,  -1000,  1000))/1000.0;
  mzMap  =  float(map(mz,  mzMin,  mzMax,  -1000,  1000))/1000.0;
  //normalize  the  magnetic  vector

  float norm= sqrt( sq(mxMap) + sq(myMap) + sq(mzMap));
  mxMap /=norm;
  myMap /=norm;
  mzMap /=norm;
  yawRaw=atan2( (myMap*cos(angle_x) + mzMap*sin(angle_x) ) , (mxMap*cos(angle_y) + myMap*sin (angle_y)*sin(angle_x)  - mzMap*sin(angle_y)*cos(angle_x))  ) *180/PI;
  YawU=atan2(-myMap, mxMap) *180/PI;
  // Change yaw range to 0-360 form -180 - 180
  if(YawU<0) {
    YawU+=360.00; 
  }
  //	Apply  Low  Pass  Filter to Yaw  to remove  any  high-frequency  magnetic  noise
  Yaw=  yawFilteredOld  +  0.8*  (YawU  - yawFilteredOld);
  yawFilteredOld=Yaw;
}



