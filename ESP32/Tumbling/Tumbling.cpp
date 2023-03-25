#include "Tumbling.h"

TwoWire ESPBus = TwoWire(1); // I2C bus for ESP32 to control
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &ESPBus);

// Initialize state, anomaly, outputs array
int currentState;
int oldCurrentState = undeployed;
int anomaly = noAnomaly;
float outputsArray[3] = {0,0,0};

// Initialize tumbling detection variables
float tumbleTime = 0;
float tumbleStart = 0;
float stillStart = 0;
float stillTime = 0;

// Initialize battery level
float batteryLevel = 100.0;
float oldBatteryLevel = 0.0;
float battThreshold = 10.0;

void sendReport(int anomaly, float oldBatteryLevel, float batteryLevel) {
  String message;
  // need to add support for multiple anomalies

  int battLev = int(floor(batteryLevel));
//  Serial.print("batteryLevel: ");
//  Serial.println(batteryLevel);
//  Serial.print("battLev: ");
//  Serial.println(battLev);
  int oldBattLev = int(floor(oldBatteryLevel));
  
  if (anomaly == battAnomaly){
    message = "Anomaly detected: battery";
    Serial.println(message);
  }
  else if (anomaly == IMUAnomaly){
    message = "Anomaly detected: IMU";
    Serial.println(message);
  }
  else if (anomaly == Pi5Anomaly){
    message = "Anomaly detected: Pi5V";
    Serial.println(message);
  }
  else if (anomaly == SDRAnomaly){
    message = "Anomaly detected: SDR";
    Serial.println(message);
  }

  if (battLev != oldBattLev){
    message = "Battery Level: ";
    Serial.print(message);
    Serial.println(battLev);
  }
//  else if (anomaly == noAnomaly){
//    message = "No anomaly detected";
//  }
  Serial.print("Message for I2C transfer to Pi: ");
  Serial.println(message);
  
}

void tumblingDetection(float (& outputsArray)[3]) {
  //Serial.println("Tumbling detection");
  // fix this function. Need to pass in tumble time as an input
  
  int tumbleState = notTumbling;
  int tumbleStart = outputsArray[2];
  int stillStart = outputsArray[3];
  /* Beginning of Tumbling Detection Function*/
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  double x = -1000000, y = -1000000 , z = -1000000;

  x = angVelocityData.acceleration.x;
  y = angVelocityData.acceleration.y;
  z = angVelocityData.acceleration.z;

    /* Display the floating point data */  
//  Serial.print("X: ");
//  Serial.print(x);
//  Serial.print("\tY: ");
//  Serial.print(y);
//  Serial.print("\tZ: ");
//  Serial.print(z);
//  Serial.println("");
//  Serial.print("TumbleTime: ");
//  Serial.println(tumbleTime);
//  Serial.print("TumbleStart: ");
//  Serial.println(tumbleStart);

  if ((abs(x) > 1) || (abs(y) > 1) || (abs(z) > 1)) {
    
    stillStart = 0;
    
    if ((tumbleState == notTumbling)&&(tumbleStart == 0)){
      tumbleStart = millis();
    }
    else {
      tumbleTime = millis()-tumbleStart;
      if (tumbleTime > 3000){
        tumbleState = tumbling;
        tumbleTime = 0;
        tumbleStart = 0;
      }
    }
  }
  else {
    if (tumbleState == tumbling) {
      tumbleStart = 0;
      
      if (stillStart == 0){
        stillStart = millis();
      }
      else {
        stillTime = millis()-stillStart;
        if (stillTime > 3000){
          tumbleState = notTumbling;
          stillTime = 0;
          stillStart = 0;
        }
      }
    }
  }

  // problem with tumble state being = 128
//  Serial.print("State from tumbling detection: ");
//  Serial.println(tumbleState);
  
  tumbleState = float(tumbleState);
  outputsArray[1] = tumbleState;
  outputsArray[2] = tumbleStart;
  outputsArray[3] = stillStart;
//  Serial.println(outputsArray[1]);

  /* End of Tumbling Detection Function*/
}

void updateState(int currentState, int oldCurrentState) {

  if (oldCurrentState != currentState){
    if (currentState == tumblingState){
      // turn on motor board
      digitalWrite(Pi5_en, HIGH);
      Serial.println("Pi: ON");
  //    digitalWrite(Pi33_en, HIGH);
      digitalWrite(SDR_en, LOW);
      Serial.println("SDR: OFF");
      digitalWrite(PR_en, HIGH);
      Serial.println("Photoresistor: ON");
    }
    else if (currentState == dayCycle){
      // turn on motor board
      digitalWrite(PR_en, HIGH);
      Serial.println("Photoresistor: ON");
      digitalWrite(Pi5_en, HIGH);
      Serial.println("Pi: ON");
  //    digitalWrite(Pi33_en, HIGH);
      digitalWrite(SDR_en, LOW);
      Serial.println("SDR: OFF");
      digitalWrite(IMU_en, HIGH);
      Serial.println("IMU: ON");
    }
    else if (currentState == nightCycle){
      // turn off motor board
      digitalWrite(PR_en, HIGH);
      Serial.println("Photoresistor: ON");
  //    digitalWrite(Pi33_en, LOW);
      digitalWrite(Pi5_en, LOW);
      Serial.println("Pi: OFF");
      digitalWrite(SDR_en, LOW);
      Serial.println("SDR: OFF");
      digitalWrite(IMU_en, LOW);
      Serial.println("IMU: OFF");
    }
    else if (currentState == lowPower){
      // turn off motor board
      digitalWrite(PR_en, LOW);
      Serial.println("Photoresistor: OFF");
  //    digitalWrite(Pi33_en, LOW);
      digitalWrite(Pi5_en, LOW);
      Serial.println("Pi: OFF");
      digitalWrite(SDR_en, LOW);
      Serial.println("SDR: OFF");
      digitalWrite(IMU_en, LOW);
      Serial.println("IMU: OFF");
    }
    else if (currentState == downlink){
      // turn on motor board
      digitalWrite(PR_en, HIGH);
      Serial.println("Photoresistor: ON");
  //    digitalWrite(Pi33_en, HIGH);
      digitalWrite(Pi5_en, HIGH);
      Serial.println("Pi: ON");
      digitalWrite(SDR_en, HIGH);
      Serial.println("SDR: ON");
      digitalWrite(IMU_en, HIGH);
      Serial.println("IMU: ON"); 
    }
    else if (currentState == lowPower){
      // turn on motor board
      digitalWrite(PR_en, LOW);
      Serial.println("Photoresistor: OFF");
  //    digitalWrite(Pi33_en, HIGH);
      digitalWrite(Pi5_en, LOW);
      Serial.println("Pi: OFF");
      digitalWrite(SDR_en, LOW);
      Serial.println("SDR: OFF");
      digitalWrite(IMU_en, LOW);
      Serial.println("IMU: OFF"); 
    }
    else if (currentState == error){
      Serial.println("State update error");
    }
  }
  
}

int checkAnomalies(int currentState) {
  /* Read in all voltages, check if they are
     in operating range from datasheeet */

  bool IMUon;
  bool Pion;
  bool SDRon;

  if ((currentState == dayCycle) || (currentState == downlink) || (currentState == tumblingState)){
    Pion = true;
  }
  else {
    Pion = false;
  }

  if ((currentState != lowPower)&&(currentState != nightCycle)){
    IMUon = true;
  }
  else {
    IMUon = false;
  }

  if ((currentState == downlink)){
    SDRon = true;
  }
  else {
    SDRon = false;
  }

  int battv;
  int IMUv;
  int Pi5v;
  int SDRv;

  //battv = analogRead(batt_in);
  IMUv = analogRead(IMU_in);
  Pi5v = digitalRead(Pi5_in);
  SDRv = digitalRead(SDR_in);

//  Serial.print("SDRv: ");
//  Serial.println(SDRv);

//  Serial.println("");
//  Serial.println("Battery voltage");
//  Serial.println(battv);
//  Serial.println("IMU voltage");
//  Serial.println(IMUv);
//  Serial.println("Pi5 voltage");
//  Serial.println(Pi5v);
//  Serial.println("");

  //if (battv < floor(200.0*3)) {
  //  return battAnomaly;
  //}
  if (((IMUv < floor(200.0*2.4))||(IMUv > floor(200.0*3.6)))&&(IMUon)) {
    return IMUAnomaly;
  }
//  else if (((Pi5v < floor(4.75*200.0))||(Pi5v > floor(5.25*200.0)))&&(Pion)) {
  else if ((Pi5v < 1)&&(Pion)) {
    return Pi5Anomaly;
  }
//  else if (((SDRv < floor(4.75*200.0))||(SDRv > floor(5.25*200.0)))&&(SDRon)) {
  else if ((SDRv < 1)&&(SDRon)) {
    return SDRAnomaly;
  }
  else {
    return noAnomaly;
  }
  
}

int dayCycleCheck() {
  int val;
  val = analogRead(PR_in);
//  Serial.print("PR voltage: ");
//  Serial.println(val);

  if (val > 750){
    return 1;
  }
  else {
    return 0;
  }
}

int checkState(int currentState, int oldCurrentState) {
  int dayCycleBool;
  int lowPowerBool;
  int downlinkBool;
  int tumblingBool = 0;

  if ((currentState == undeployed)||(currentState == tumbling)){
    tumblingDetection(outputsArray); //might have to modify this to run in the background
    tumblingBool = outputsArray[1];
    tumblingBool = int(tumblingBool);
  }

  if (currentState == undeployed) {
    if (!tumblingBool) {
      currentState = undeployed;
//      if (currentState != oldCurrentState){
//        Serial.println("Undeployed state detected");
//      }
    }
    else {
      currentState = tumblingState;
      if (currentState != oldCurrentState){
        Serial.println("Tumbling state detected");
      }
    } 
  }
  else if (!tumblingBool){
//    Serial.println("Boolcheck");
    dayCycleBool = dayCycleCheck();
    //lowPowerBool = lowPowerCheck();
    //downlinkBool = downlinkCheck();
    lowPowerBool = false;    
    downlinkBool = false;

    if ((dayCycleBool)&&(!downlinkBool)&&(!lowPowerBool)){
      currentState = dayCycle;
      if (currentState != oldCurrentState){
        Serial.println("Day cycle detected");
      }
    }
    else if ((downlinkBool)&&(!lowPowerBool)){
      currentState = downlink;
      if (currentState != oldCurrentState){
        Serial.println("Downlink state detected");
      }
    }
    else if (lowPowerBool){
      currentState = lowPower;
      if (currentState != oldCurrentState){
        Serial.println("Low power detected");
      }
    }
    else if ((!dayCycleBool)&&(!downlinkBool)&&(!lowPowerBool)){
      currentState = nightCycle;
      if (currentState != oldCurrentState){
        Serial.println("Night cycle detected");
      }
    }
    else {
      currentState = error;
      if (currentState != oldCurrentState){
        Serial.println("State error detected");
      }
    }
  }
  else {
    currentState = tumblingState;
    if (currentState != oldCurrentState){
      Serial.println("Tumbling State detected");
    }
  }
  
//  Serial.print("Current State: ");
//  Serial.println(currentState);

  return currentState;
}

int8_t temperature_read() {
  int8_t boardTemp = bno.getTemp();
  return boardTemp;
}
