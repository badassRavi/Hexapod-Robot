#include <math.h>
#include "HCPCA9685.h"

#define  I2CAdd 0x40
#define  I2CAddB 0x41

HCPCA9685 HCPCA9685(I2CAdd),HCPCA9685B(I2CAddB);
int dt=30;
float a=4.5;
float b=3;

int m1,m2,m3;

float r;
int theta,phai,beta,alpha,gamm,GAMMA;
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
void RL2(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamm=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  
  GAMMA=30+gamm;
  m1=map(theta,0,180,10,430);
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);
  
  HCPCA9685B.Servo(3,m1);
  HCPCA9685B.Servo(4,430-m2);
  HCPCA9685B.Servo(5,430-m3);
  
}

void LL1(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamm=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  
  GAMMA=30+gamm;
  m1=map(theta,0,180,10,430);
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);
  
  HCPCA9685.Servo(6, m1);
  HCPCA9685.Servo(7, m2);
  HCPCA9685.Servo(8, m3);


 
}
void LL3(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamm=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  GAMMA=30+gamm;

  m1=map(theta,0,180,10,430);
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);
  
  HCPCA9685.Servo(0, m1);
  HCPCA9685.Servo(1, m2);
  HCPCA9685.Servo(2, m3);
  }

void LL2(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamm=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  GAMMA=30+gamm;

  m1=map(theta,0,180,10,430);
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);
  
  HCPCA9685.Servo(3, m1);
  HCPCA9685.Servo(4, m2);
  HCPCA9685.Servo(5, m3);
 }

void RL1(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamm=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  GAMMA=30+gamm;

  m1=map(theta,0,180,10,430);
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);

  HCPCA9685B.Servo(6,m1);
  HCPCA9685B.Servo(7,430-m2);
  HCPCA9685B.Servo(8,430-m3);
}

void RL3(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamm=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  GAMMA=30+gamm;

  m1=map(theta,0,180,10,430);
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);
  
  HCPCA9685B.Servo(0,m1);
  HCPCA9685B.Servo(1,430-m2);
  HCPCA9685B.Servo(2,430-m3);
  }

void setup(){

HCPCA9685.Init(SERVO_MODE);
HCPCA9685.Sleep(false);
HCPCA9685B.Init(SERVO_MODE);
HCPCA9685B.Sleep(false);
RL1(0,10,-7.5);
RL2(0,10,-7.5);
RL3(0,10,-7.5);
LL1(0,10,-7.5);
LL2(0,10,-7.5);
LL3(0,10,-7.5);

Serial.begin(115200);
}


int state;
int Oangle;
int intensity;
int dx,dy;
float ri=3;
float X0=0;
float Y0=7;
float YC0=6;
float Z0=-7.5;
float ZC0=-10;
float delr=0.15;
float delz=0.3;

String datax,datay,dataOangle,dataintensity,datastate;

void loop(){
  while (Serial.available()==0){}
  datastate=Serial.readStringUntil(' ');
  state=datastate.toInt();

if (datastate=="1"){
  dataintensity=Serial.readStringUntil(' ');
  dataOangle=Serial.readStringUntil('\n');
  intensity=dataintensity.toInt();
  Oangle=dataOangle.toInt();
  Serial.print(datastate);
  Serial.print(" ");
  Serial.print(intensity);
  Serial.print(" ");
  Serial.println(Oangle);
  if (intensity>=15){
    for (int i=1;i<=40;i++){
      ri=ri-delr;
      LL2(X0+ri*cos(degtoradd(Oangle-90)),Y0+ri*sin(degtoradd(Oangle-90)),-7.5);
      RL1(X0+ri*cos(degtoradd(Oangle+43.43)),Y0+ri*sin(degtoradd(Oangle+43.43)),-7.5);
      RL3(X0+ri*cos(degtoradd(Oangle+133.43)),Y0+ri*sin(degtoradd(Oangle+133.43)),-7.5);

      if (i<=20){
        Z0=Z0+delz;
        RL2(X0+ri*cos(degtoradd(Oangle-90)),Y0+ri*sin(degtoradd(Oangle-90)),Z0);
        LL1(X0+ri*cos(degtoradd(Oangle+133.43)),Y0+ri*sin(degtoradd(Oangle+133.43)),Z0);
        LL3(X0+ri*cos(degtoradd(Oangle+43.43)),Y0+ri*sin(degtoradd(Oangle+43.43)),Z0);
      }
      else {
        Z0=Z0-delz;
        RL2(X0+ri*cos(degtoradd(Oangle-90)),Y0+ri*sin(degtoradd(Oangle-90)),Z0);
        LL1(X0+ri*cos(degtoradd(Oangle+133.43)),Y0+ri*sin(degtoradd(Oangle+133.43)),Z0);
        LL3(X0+ri*cos(degtoradd(Oangle+43.43)),Y0+ri*sin(degtoradd(Oangle+43.43)),Z0);

      }
      delay(dt);
    }
    for (int i=1;i<=40;i++){
      ri=ri+delr;
      RL2(X0+ri*cos(degtoradd(Oangle-90)),Y0+ri*sin(degtoradd(Oangle-90)),-7.5);
      LL1(X0+ri*cos(degtoradd(Oangle+133.43)),Y0+ri*sin(degtoradd(Oangle+133.43)),-7.5);
      LL3(X0+ri*cos(degtoradd(Oangle+43.43)),Y0+ri*sin(degtoradd(Oangle+43.43)),-7.5);

      if (i<=20){
        Z0=Z0+delz;
        LL2(X0+ri*cos(degtoradd(Oangle-90)),Y0+ri*sin(degtoradd(Oangle-90)),Z0);
        RL1(X0+ri*cos(degtoradd(Oangle+43.43)),Y0+ri*sin(degtoradd(Oangle+43.43)),Z0);
        RL3(X0+ri*cos(degtoradd(Oangle+133.43)),Y0+ri*sin(degtoradd(Oangle+133.43)),Z0);
      }
      else {
        Z0=Z0-delz;
        LL2(X0+ri*cos(degtoradd(Oangle-90)),Y0+ri*sin(degtoradd(Oangle-90)),Z0);
        RL1(X0+ri*cos(degtoradd(Oangle+43.43)),Y0+ri*sin(degtoradd(Oangle+43.43)),Z0);
        RL3(X0+ri*cos(degtoradd(Oangle+133.43)),Y0+ri*sin(degtoradd(Oangle+133.43)),Z0);
      }
      delay(dt);

    }
  }
  else {
    // LL1(0,10,-8);
    // LL2(0,10,-8);
    // LL3(0,10,-8);
    // RL1(0,10,-8);
    // RL2(0,10,-8);
    // RL3(0,10,-8);
    }

}
else if (datastate=="0"){
  datax=Serial.readStringUntil(' ');
  datay=Serial.readStringUntil('\n');
  dy=datay.toInt();
  dx=datax.toInt();
  Serial.print(datastate);
  Serial.print(" ");
  Serial.print(dx);
  Serial.print(" ");
  Serial.println(dy);
  if (dx>15){
    for (int i=1;i<=40;i++){
      ri=ri-delr;
      if (i<=20){
        ZC0=ZC0+delz;
      }
      else{
        ZC0=ZC0-delz;
      }
      LL2(ri,YC0,-10);
      RL1(ri,YC0,-10);
      RL3(ri,YC0,-10);
      RL2(-ri,YC0,ZC0);
      LL1(-ri,YC0,ZC0);
      LL3(-ri,YC0,ZC0);
      delay(dt);
    }
    for (int i=1;i<=40;i++){
      ri=ri+delr;
      if (i<=20){
        ZC0=ZC0+delz;
      }
      else{
        ZC0=ZC0-delz;
      }
      LL2(ri,YC0,ZC0);
      RL1(ri,YC0,ZC0);
      RL3(ri,YC0,ZC0);
      RL2(-ri,YC0,-10);
      LL1(-ri,YC0,-10);
      LL3(-ri,YC0,-10);
      delay(dt);
    }

  }
  else if (dx<-15){
    for (int i=1;i<=40;i++){
      ri=ri-delr;
      if (i<=20){
        ZC0=ZC0+delz;
      }
      else{
        ZC0=ZC0-delz;
      }
      LL2(-ri,YC0,-10);
      RL1(-ri,YC0,-10);
      RL3(-ri,YC0,-10);
      RL2(ri,YC0,ZC0);
      LL1(ri,YC0,ZC0);
      LL3(ri,YC0,ZC0);
      delay(dt);
    }
    for (int i=1;i<=40;i++){
      ri=ri+delr;
      if (i<=20){
        ZC0=ZC0+delz;
      }
      else{
        ZC0=ZC0-delz;
      }
      LL2(-ri,YC0,ZC0);
      RL1(-ri,YC0,ZC0);
      RL3(-ri,YC0,ZC0);
      RL2(ri,YC0,-10);
      LL1(ri,YC0,-10);
      LL3(ri,YC0,-10);
      delay(dt);
    }
  }
  else {
    // LL1(0,10,-8);
    // LL2(0,10,-8);
    // LL3(0,10,-8);
    // RL1(0,10,-8);
    // RL2(0,10,-8);
    // RL3(0,10,-8);
  }
}
Serial.end();
Serial.begin(115200);
}
