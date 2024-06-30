#include <InvokController.h>

// Controller Object Instantiation
Controller myController("websocket", 80, false);

// Timer
// double elapsedTime = 0;
// double nowTime = 0;
// int mdt=500;
int polarval=1;
int carval=0;

void setup() {
  Serial.begin(115200);

  // Controller Setup
  myController.setHostname("Joystick car and polar"); // mDNS
  myController.begin();
}

void loop() {

  // Print data every second          
  // elapsedTime = millis() - nowTime;
  if (myController.isConnected() &&  myController.joystick.getButtonState()==carval){
  Serial.printf("%d %d %d\n", myController.joystick.getButtonState(),
  myController.joystick.getX(), myController.joystick.getY());
  // elapsedTime = 0;
  // nowTime = millis();
  }
  else if (myController.isConnected() &&  myController.joystick.getButtonState()==polarval ){
    Serial.printf("%d %d %d\n",myController.joystick.getButtonState(),
    myController.joystick.getIntensity(), myController.joystick.getTheta());
    // elapsedTime = 0;
    // nowTime = millis();
  }

  // Controller Loop
  myController.loop();
}