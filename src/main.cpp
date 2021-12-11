#include <Arduino.h>
#include "gripper.h"

Gripper gripper;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  gripper.setup();
}

void loop() {
  // put your main code here, to run repeatedly:
   static bool flag = true;

  if(flag){
    gripper.gripperClose();
    flag = false;
    delay(1000);
  } else{
    gripper.gripperOpen();
    flag = true;
    delay(1000);
  }
}