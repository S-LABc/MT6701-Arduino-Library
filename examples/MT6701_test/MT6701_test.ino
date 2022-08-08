/*
 * MT6701_test
 * 
 * Демонстрация некоторых возможнойстей библиотеки MT6701_I2C.h для датчика MT6701
 * 
 * Документация к датчику:
 * http://www.magntek.com.cn/upload/MT6701_Rev.1.5.pdf
 * 
 * Все методы можно посмотреть в файле MT6701_I2C.h или тут
 * https://github.com/S-LABc/MT6701-Arduino-Library/blob/main/src/MT6701_I2C.h
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.0 / Скляр Роман S-LAB
 */
 
#include <MT6701_I2C.h>

// раскомментировать для заупска функции сохранения значений
//#define SAVE_VALUES

/*
 * STM32_MT6701_MODE_PIN   PC13
 * ESP32_MT6701_MODE_PIN   2
 * ARDUINO_MT6701_MODE_PIN 3
 * или любой другой
 * 
 * MT6701I2C_MODE_I2C_SSI - притягивает контакт MODE к VCC
 * MT6701I2C_MODE_UVW_ABZ - притягивает контакт MODE к GND
 */
MT6701I2C SensorI2C(&Wire);
//MT6701I2C SensorI2C(&Wire, ARDUINO_MT6701_MODE_PIN, MT6701I2C_MODE_I2C_SSI);
 
void setup() {
  Serial.begin(115200);
  
  SensorI2C.begin();
  
  //SensorI2C.setClock100kHz();
  SensorI2C.setClock400kHz();
  //SensorI2C.setClock1MHz();
  //SensorI2C.setClock(200000); // другое значение частоты
  
  while(!Serial) {}

  while(!SensorI2C.isConnected()) {
    Serial.println("Датчик не обнаружен");
    delay(500);
  }
}
 
 
void loop() {
#ifdef SAVE_VALUES
  saveNewSettings();
#else
  readValues();
#endif
}

void readValues() {
  Serial.print("Raw: ");
  Serial.println(SensorI2C.getRawAngle());

  Serial.print("Degrees: ");
  Serial.println(SensorI2C.getDegreesAngle());

  Serial.print("Radians: ");
  Serial.println(SensorI2C.getRadiansAngle());

  Serial.print("Resolution UVW: ");
  Serial.println(SensorI2C.getOutputResolutionUVW()); // setOutputResolutionUVW()

  Serial.print("Resolution ABZ: ");
  Serial.println(SensorI2C.getOutputResolutionABZ()); // setOutputResolutionABZ()

  Serial.print("Output Type: ");
  MT6701I2COutputType output_type = SensorI2C.getOutputType();
  if(output_type == MT6701I2_OUTPUT_TYPE_ABZ) { // setOutputTypeABZ()
    Serial.println("ABZ");
  } else if(output_type == MT6701I2_OUTPUT_TYPE_UVW) { // setOutputTypeUVW()
    Serial.println("UVW");
  }

  Serial.print("Output Mode: ");
  MT6701I2COutputMode output_mode = SensorI2C.getOutputMode();
  if(output_mode == MT6701I2_OUTPUT_MODE_ANALOG) { // setOutputModeAnalog()
    Serial.println("ANALOG");
  } else if(output_mode == MT6701I2_OUTPUT_MODE_PWM) { // setOutputModePWM()
    Serial.println("PWM");
  }

  Serial.print("Rotation Direction: ");
  MT6701I2CDirection output_dir = SensorI2C.getOutputRotationDirection();
  if(output_dir == MT6701I2_DIRECTION_CLOCKWISE) { // setOutputRotationDirectionClockwise()
    Serial.println("CLOCKWISE");
  } else if(output_dir == MT6701I2_DIRECTION_COUNTERCLOCKWISE) { // setOutputRotationDirectionCounterclockwise()
    Serial.println("COUNTERCLOCKWISE");
  }
  
  Serial.println();

  delay(200);
}

void saveNewSettings() {
  Serial.print("Write Resolution ABZ: ");
  word new_res_abz = 732;
  if(SensorI2C.setOutputResolutionABZVerify(new_res_abz)) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
  }

  Serial.print("Write Output Mode PWM: ");
  if(SensorI2C.setOutputModePWMVerify()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
  }

  Serial.print("Write Rotation Direction Clockwise: ");
  if(SensorI2C.setOutputRotationDirectionClockwiseVerify()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
  }

  Serial.println("Saving New Values...");
  SensorI2C.saveNewValues();
  delay(700); // >600мс
  Serial.println("Saved Successfully. Reconnect Power");
  
  while(1) {}
}
