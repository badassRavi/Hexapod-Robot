#include <math.h>
#include "HCPCA9685.h"

#define  I2CAdd 0x40
#define  I2CAddB 0x41
#define root3 1.7320508
#define side_dis 0.2
#define angular_dis 0.17
#define z_dis 0.2
HCPCA9685 HCPCA9685(I2CAddB),HCPCA9685B(I2CAdd);
// Servo s1,s2,s3;
// r is also used somewhere else

int dt=1;

// int s1pin=8;
// int s2pin=9;
// int s3pin=10;
float a=4.5;
float b=3;

int m1,m2,m3;

float x,y,z,r;
float X=0.001;
  float Y,Z;
  float XX=-8.401;
  float YY,ZZ;
  float p=4.2;
  float q=10;
  float P=8.401;
  float Q,R=-7.5;
  float PP=0.001;
  float QQ,RR=-7.5;
int theta,phai,beta,alpha,gamma,GAMMA;
float forcos;

float pi=3.14159265;

// int s1angle;
// int s2angle;
// int s3angle;

float l1=7.7;
float l2=11.1;

//functions
float raddtodeg(float th){
  float ans;
  ans=th*180/pi;
  return ans;
}
float degtoradd(float angle){
  float deg;
  deg=angle*pi;
  deg=deg/180;
  return deg;
}


float rlength(float x,float y, float z){
  float r;
  r=pow((x*x+y*y+z*z),0.5);
  return r;
}



int s1angle(float x,float y){
  float theta;
  int thetha;
  theta=atan(y/x);
  thetha=raddtodeg(theta);
  if (theta<0){
    
  thetha=180+thetha;
  }

  return thetha;
}



int s2angle(float l1,float l2,float z,float r){
  int phai,alpha;
  alpha=raddtodeg(acos((l1*l1+r*r-l2*l2)/(2*l1*r)));
  phai=raddtodeg(acos(z/r));
  return phai-alpha;
}


int s3angle(float l1,float l2,float r){
  int beta;
  beta=raddtodeg(acos((l1*l1+l2*l2-r*r)/(2*l1*l2)));
  return beta;
}
//functions end





//main function
void jp(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamma=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  // Serial.print(theta);
  // Serial.print(" ");
  // Serial.print(30+gamma);
  // Serial.print(" ");
  // Serial.println(beta);

  GAMMA=30+gamma;
  m1=map(theta,0,180,10,430);
  
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);
  
  // Serial.print(m1);
  // Serial.print(" ");
  // Serial.print(m2);
  // Serial.print(" ");
  // Serial.println(m3);

  HCPCA9685B.Servo(3,m1);
  HCPCA9685B.Servo(4,430-m2);
  HCPCA9685B.Servo(5,430-m3);
  
}

void jp2(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamma=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  // Serial.print(theta);
  // Serial.print(" ");
  // Serial.print(30+gamma);
  // Serial.print(" ");
  // Serial.println(beta);

  GAMMA=30+gamma;
  m1=map(theta,0,180,10,430);
  
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);
  
  // Serial.print(m1);
  // Serial.print(" ");
  // Serial.print(m2);
  // Serial.print(" ");
  // Serial.println(m3);

  
  
  HCPCA9685.Servo(0, m1);
  HCPCA9685.Servo(1, m2);
  HCPCA9685.Servo(2, m3);


 
}
void jp3(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamma=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  // Serial.print(theta);
  // Serial.print(" ");
  // Serial.print(30+gamma);
  // Serial.print(" ");
  // Serial.println(beta);

  GAMMA=30+gamma;
  m1=map(theta,0,180,10,430);
  
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);
  
  // Serial.print(m1);
  // Serial.print(" ");
  // Serial.print(m2);
  // Serial.print(" ");
  // Serial.println(m3);

  
  HCPCA9685.Servo(6, m1);
  HCPCA9685.Servo(7, m2);
  HCPCA9685.Servo(8, m3);
  // HCPCA9685.Servo(3, m1);
  // HCPCA9685.Servo(4, m2);
  // HCPCA9685.Servo(5, m3);
  // HCPCA9685.Servo(6, m1);
  // HCPCA9685.Servo(7, m2);
  // HCPCA9685.Servo(8, m3);


  // HCPCA9685B.Servo(0,430-m1);
  // HCPCA9685B.Servo(1,430-m2);
  // HCPCA9685B.Servo(2,430-m3);
  // HCPCA9685B.Servo(3,430-m1);
  // HCPCA9685B.Servo(4,430-m2);
  // HCPCA9685B.Servo(5,430-m3);
  // HCPCA9685B.Servo(6,430-m1);
  // HCPCA9685B.Servo(7,430-m2);
  // HCPCA9685B.Servo(8,430-m3);
}

void jp4(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamma=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  // Serial.print(theta);
  // Serial.print(" ");
  // Serial.print(30+gamma);
  // Serial.print(" ");
  // Serial.println(beta);

  GAMMA=30+gamma;
  m1=map(theta,0,180,10,430);
  
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);
  
  // Serial.print(m1);
  // Serial.print(" ");
  // Serial.print(m2);
  // Serial.print(" ");
  // Serial.println(m3);

  
  // HCPCA9685.Servo(0, m1);
  // HCPCA9685.Servo(1, m2);
  // HCPCA9685.Servo(2, m3);
  HCPCA9685.Servo(3, m1);
  HCPCA9685.Servo(4, m2);
  HCPCA9685.Servo(5, m3);
  // HCPCA9685.Servo(6, m1);
  // HCPCA9685.Servo(7, m2);
  // HCPCA9685.Servo(8, m3);


  // HCPCA9685B.Servo(0,430-m1);
  // HCPCA9685B.Servo(1,430-m2);
  // HCPCA9685B.Servo(2,430-m3);
  // HCPCA9685B.Servo(3,430-m1);
  // HCPCA9685B.Servo(4,430-m2);
  // HCPCA9685B.Servo(5,430-m3);
  // HCPCA9685B.Servo(6,430-m1);
  // HCPCA9685B.Servo(7,430-m2);
  // HCPCA9685B.Servo(8,430-m3);
}

void jp5(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamma=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  // Serial.print(theta);
  // Serial.print(" ");
  // Serial.print(30+gamma);
  // Serial.print(" ");
  // Serial.println(beta);

  GAMMA=30+gamma;
  m1=map(theta,0,180,10,430);
  
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);
  
  // Serial.print(m1);
  // Serial.print(" ");
  // Serial.print(m2);
  // Serial.print(" ");
  // Serial.println(m3);

  
  // HCPCA9685.Servo(0, m1);
  // HCPCA9685.Servo(1, m2);
  // HCPCA9685.Servo(2, m3);
  // HCPCA9685.Servo(3, m1);
  // HCPCA9685.Servo(4, m2);
  // HCPCA9685.Servo(5, m3);
  // HCPCA9685.Servo(6, m1);
  // HCPCA9685.Servo(7, m2);
  // HCPCA9685.Servo(8, m3);


  // HCPCA9685B.Servo(0,430-m1);
  // HCPCA9685B.Servo(1,430-m2);
  // HCPCA9685B.Servo(2,430-m3);
  // HCPCA9685B.Servo(3,430-m1);
  // HCPCA9685B.Servo(4,430-m2);
  // HCPCA9685B.Servo(5,430-m3);
  HCPCA9685B.Servo(6,m1);
  HCPCA9685B.Servo(7,430-m2);
  HCPCA9685B.Servo(8,430-m3);
}

void jp6(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamma=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  // Serial.print(theta);
  // Serial.print(" ");
  // Serial.print(30+gamma);
  // Serial.print(" ");
  // Serial.println(beta);

  GAMMA=30+gamma;
  m1=map(theta,0,180,10,430);
  
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);
  
  // Serial.print(m1);
  // Serial.print(" ");
  // Serial.print(m2);
  // Serial.print(" ");
  // Serial.println(m3);

  
  // HCPCA9685.Servo(0, m1);
  // HCPCA9685.Servo(1, m2);
  // HCPCA9685.Servo(2, m3);
  // HCPCA9685.Servo(3, m1);
  // HCPCA9685.Servo(4, m2);
  // HCPCA9685.Servo(5, m3);
  // HCPCA9685.Servo(6, m1);
  // HCPCA9685.Servo(7, m2);
  // HCPCA9685.Servo(8, m3);


  HCPCA9685B.Servo(0,m1);
  HCPCA9685B.Servo(1,430-m2);
  HCPCA9685B.Servo(2,430-m3);
  // HCPCA9685B.Servo(3,430-m1);
  // HCPCA9685B.Servo(4,430-m2);
  // HCPCA9685B.Servo(5,430-m3);
  // HCPCA9685B.Servo(6,430-m1);
  // HCPCA9685B.Servo(7,430-m2);
  // HCPCA9685B.Servo(8,430-m3);
}






void setup(){
  
  Serial.begin(9600);
  
  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);
  HCPCA9685B.Init(SERVO_MODE);
  HCPCA9685B.Sleep(false);

  x=-4.2;
  y=10;
  z=-7.5;
  
 
  // jp(x,y,z);
  // delay(20);


}
void loop(){
  
  float r=-7.5;
  //Left and Right legs
 for (float i=1;i<=42;i=i+1){
  x=x+side_dis;

  jp(x,y,z);

  X=X+angular_dis;
  Y=root3*X;
  Z=-7.5;
  jp2(x,y,z);
  

  XX=XX+angular_dis;
  YY=-root3*(XX);
  ZZ=-7.5;
  jp3(x,y,z);
  //Code for second set of wheels

  

  p=p-side_dis;
  if (i<=21){
    r=r+z_dis;
  }
  else{
    r=r-z_dis;
  }
  jp4(p,q,r);
  
  P=P-angular_dis;
  Q=root3*P;
  if (i<=21){
    R=R+z_dis;
  }
  else{
    R=R-z_dis;
  }
  jp5(p,q,r);
  

  PP=PP-angular_dis;
  QQ=-root3*(PP);
  if (i<=21){
    RR=RR+z_dis;
  }
  else{
    RR=RR-z_dis;
  }
  jp6(p,q,r);
  // delay(dt);
}

Serial.println("loop 2");
for (float i=1;i<=42;i=i+1){
  x=x-side_dis;
  if (i<=21){
    z=z+z_dis;
  }
  else{
    z=z-z_dis;
  }
  jp(x,y,z);
  X=X-angular_dis;
  Y=root3*X;
  if (i<=21){
    Z=Z+z_dis;
  }
  else{
    Z=Z-z_dis;
  }
  

  jp2(x,y,z);
  XX=XX-angular_dis;
  YY=-root3*(XX);
  if (i<=21){
    ZZ=ZZ+z_dis;
  }
  else{
    ZZ=ZZ-z_dis;
  }
  jp3(x,y,z);


  //CODE FOR SECOND SET OF LEGS
  p=p+side_dis;

  jp4(p,q,r);

  P=P+angular_dis;
  Q=root3*P;
  R=-7.5;
  jp5(p,q,r);
  
  
  PP=PP+angular_dis;
  QQ=-root3*PP;
  RR=-7.5;
  jp6(p,q,r);


  
  //delay(dt);
}
//Back Left and Right Legs

 
// for (float i=1;i<=12;i++){
//   X=X+0.4;
//   Y=root3*(X+5);
//   Z=-10;
//   Serial.print(X);
//   Serial.print(" ");
//   Serial.print(Y);
//   Serial.print(" ");
//   Serial.println(Z);
//   jp(X,Y,Z);
//   delay(100);
// }

// for (float i=1;i<=12;i++){
//   X=X-0.4;
//   Y=root3*(X+5);
//   if (i<=6){
//     Z=Z+1.4;
//   }
//   else{
//     Z=Z-1.4;
//   }
//   Serial.print(X);
//   Serial.print(" ");
//   Serial.print(Y);
//   Serial.print(" ");
//   Serial.println(Z);
//   jp(X,Y,Z);
//   delay(100);

//}
//Front left and Right legs
// float X=-2.64;
// float Y,Z;

// for (float i=1;i<=12;i++){
//   X=X+0.4;
//   Y=root3*(-X+5);
//   Z=-10;
//   Serial.print(X);
//   Serial.print(" ");
//   Serial.print(Y);
//   Serial.print(" ");
//   Serial.println(Z);
//   jp(X,Y,Z);
//   delay(100);
// }

// for (float i=1;i<=12;i++){
//   X=X-0.4;
//   Y=root3*(-X+5);
//   if (i<=6){
//     Z=Z+1.4;
//   }
//   else{
//     Z=Z-1.4;
//   }
//   Serial.print(X);
//   Serial.print(" ");
//   Serial.print(Y);
//   Serial.print(" ");
//   Serial.println(Z);
//   jp(X,Y,Z);
//   delay(100);

// }


}