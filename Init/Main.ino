#include <Wire.h>
void setup(){
  
//  analogWrite(6,182);
  Wire.begin();
  Serial.begin(115200);
//  Serial.println("starting up L3G4200D");
  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  getOffset();
  getAccOffset();
  Serial.println("Echo");
  delay(1500); //wait for the sensor to be ready 
}
void loop(){
 
    getGyroValues();  // This will update x, y, and z with new values
    
    xaRaw=(analogRead(xAxis)-xaOffset)*3.3 / (1023.0*0.8);
    yaRaw=(analogRead(yAxis)-yaOffset)*3.3 / (1023.0*0.8);
    zaRaw=(analogRead(zAxis)-zaOffset)*3.3 / (1023.0*0.8)+1;
    
    Serial.print(x - xOff);
    Serial.print(" ");
    Serial.print(y - yOff);
    Serial.print(" ");
    Serial.print(z - zOff);
    Serial.print(" ");
    Serial.print(xaRaw);
    Serial.print(" ");
    Serial.print(yaRaw);
    Serial.print(" ");
    Serial.print(zaRaw);
    Serial.println(" ");
//    delayMicroseconds(50); //Just here to slow down the serial to make it more readable
}


