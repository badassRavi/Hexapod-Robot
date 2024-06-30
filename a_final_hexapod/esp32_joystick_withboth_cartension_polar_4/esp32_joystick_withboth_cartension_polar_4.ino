#include <InvokController.h>

// Controller Object Instantiation
Controller myController("websocket", 80, false);

// Timer
// double elapsedTime = 0;
// double nowTime = 0;
// int mdt=500;
bool polarval=true;
bool carval=false;

void setup() {
  Serial.begin(115200);

  // Controller Setup
  myController.setHostname("Joystick car and polar"); // mDNS
  myController.begin();
}
int s,x,y,p,i;

void loop() {

  // Print data every second          
  // elapsedTime = millis() - nowTime;
  if (myController.isConnected() &&  myController.joystick.getButtonState()==carval){
    s=myController.joystick.getButtonState();
    x=myController.joystick.getX();
    y=myController.joystick.getY();
    Serial.println(String(0)+" "+String(x)+" "+String(y));





  // Serial.printf("%d %d %d\n", myController.joystick.getButtonState(),
  // myController.joystick.getX(), myController.joystick.getY());
  // elapsedTime = 0;
  // nowTime = millis();
  }
  else if (myController.isConnected() &&  myController.joystick.getButtonState()==polarval){
    s=myController.joystick.getButtonState();
    i=myController.joystick.getIntensity();
    p=myController.joystick.getTheta();
    Serial.println(String(1)+" "+String(i)+" "+String(p));




    // Serial.printf("%d %d %d\n",myController.joystick.getButtonState(),
    // myController.joystick.getIntensity(), myController.joystick.getTheta());
    // elapsedTime = 0;
    // nowTime = millis();
  }

  // Controller Loop
  myController.loop();
}