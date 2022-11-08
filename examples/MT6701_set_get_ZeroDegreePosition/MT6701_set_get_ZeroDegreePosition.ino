/*
 * 1. Open Serial Monitor at 9600 and there is no line ending
 * 2. Send the character "s" to the Serial Monitor
 * 
 * CORRECT SERIAL MONITOR LOGS

   Sensor Found!

   Reading Current Value...
   Zero Degree Position (HEX): 0 [0 - default, but may be different]

   Changing Value... Value Changed

   Saving New Values...

   Reading New Value...
   Zero Degree Position After Saving (HEX): 8

   Program Complete!
 *
 */

#include <MT6701_I2C.h>

MT6701I2C SensorI2C(&Wire);

void setup() {
  Serial.begin(9600);
  
  SensorI2C.begin();
  SensorI2C.setClock();
  
  while(1) { // Waiting for the input of the character "s"
    byte c;
    if(Serial.available())
      c = Serial.read();
    if(c == 's') {
      break;
    }
    delay(100);
  }

  while(!SensorI2C.isConnected()) {
    Serial.println("Sensor not Connected!");
    delay(500);
  }
  Serial.println("Sensor Found!");
  Serial.println();

  delay(300); // The delay is not important. so it's easier to see in the SM

  Serial.println("Reading Current Value...");
  word zdp;
  zdp = SensorI2C.getZeroDegreePositionData();
  Serial.print("Zero Degree Position (HEX): ");
  Serial.println(zdp, HEX);
  Serial.println();

  delay(300); // The delay is not important. so it's easier to see in the SM

  Serial.print("Changing Value... ");
  word zdp_new = 0x0008; // New value
  if(SensorI2C.setZeroDegreePositionDataVerify(zdp_new)) {
    Serial.println("Value Changed");
  } else {
    Serial.println("Value Change Error");
  }
  Serial.println();

  delay(300); // The delay is not important. so it's easier to see in the SM

  Serial.println("Saving New Values...");
  Serial.println();
  // It's important to save the new values after the change.
  // Called once even after setting multiple values
  // else values return to default after power off
  SensorI2C.saveNewValues();

  delay(700); // >600ms

  Serial.println("Reading New Value...");
  word zdp_after;
  zdp_after = SensorI2C.getZeroDegreePositionData();
  Serial.print("Zero Degree Position After Saving (HEX): ");
  Serial.println(zdp_after, HEX);
  Serial.println();

  Serial.println("Program Complete!");
}

void loop() {
  // nop
}
