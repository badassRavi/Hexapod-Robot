#include <InvokController.h>

// Controller Object Instantiation
Controller myController("websocket", 80, false);

// Timer
double elapsedTime = 0;
double nowTime = 0;
int mdt=1000;

void setup() {
  Serial.begin(115200);

  // Controller Setup
  myController.setHostname("Joystick car and polar"); // mDNS
  myController.begin();
}

void loop() {

  // Print data every second
  elapsedTime = millis() - nowTime;
  if (elapsedTime > mdt && myController.isConnected() &&  myController.joystick.getButtonState()==0 ){
    Serial.printf("Intensity : [ %.1f ], Theta : [ %.1f ], State : [ %s ]\n", 
    myController.joystick.getIntensity(), myController.joystick.getTheta(),
    myController.joystick.getButtonState() ? "true" : "false");
    elapsedTime = 0;
    nowTime = millis();
  }
  else if (elapsedTime > mdt && myController.isConnected() &&  myController.joystick.getButtonState()==1){
    Serial.printf("x : [ %.1f ], y : [ %.1f ], State : [ %s ]\n", 
    myController.joystick.getX(), myController.joystick.getY(),
    myController.joystick.getButtonState() ? "true" : "false");
    elapsedTime = 0;
    nowTime = millis();
  }

  // Controller Loop
  myController.loop();
}