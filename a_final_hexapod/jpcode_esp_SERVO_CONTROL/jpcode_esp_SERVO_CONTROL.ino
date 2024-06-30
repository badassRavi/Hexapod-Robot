#include <math.h>
#include "HCPCA9685.h"

#define  I2CAdd 0x40
#define  I2CAddB 0x41
#define  root3 1.7320508

HCPCA9685 HCPCA9685(I2CAdd),HCPCA9685B(I2CAddB);

// Servo s1,s2,s3;

String datax,datay;
int dt=1;
int jx,jy,u,v;


// int s1pin=8;
// int s2pin=9;
// int s3pin=10;
float a=4.5;
float b=3;

int m1,m2,m3,mjy,mjx;

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
void jp(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamm=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  
  GAMMA=30+gamm;
  m1=map(theta,0,180,10,430);
  
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);
  
  
  HCPCA9685B.Servo(3,430-m1);
  HCPCA9685B.Servo(4,430-m2);
  HCPCA9685B.Servo(5,430-m3);
  
}

void jp2(float x,float y,float z){
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
void jp3(float x,float y,float z){
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

void jp4(float x,float y,float z){
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

void jp5(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamm=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  GAMMA=30+gamm;
  m1=map(theta,0,180,10,430);
  
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);
  HCPCA9685B.Servo(6,430-m1);
  HCPCA9685B.Servo(7,430-m2);
  HCPCA9685B.Servo(8,430-m3);
}

void jp6(float x,float y,float z){
  theta=s1angle(x,y);
  forcos=degtoradd(theta);
  r=rlength(x-b*cos(forcos),y-b*sin(forcos),z-a);
  gamm=s2angle(l1,l2,z-a,r);
  beta=s3angle(l1,l2,r);
  GAMMA=30+gamm;
  m1=map(theta,0,180,10,430);
  
  m2=map(GAMMA,0,180,10,430);
  m3=map(beta,0,180,10,430);
  
  HCPCA9685B.Servo(0,430-m1);
  HCPCA9685B.Servo(1,430-m2);
  HCPCA9685B.Servo(2,430-m3);
  }

void jpc(float x,float y,float z){
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

void jpc2(float x,float y,float z){
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
void jpc3(float x,float y,float z){
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

void jpc4(float x,float y,float z){
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

void jpc5(float x,float y,float z){
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

void jpc6(float x,float y,float z){
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








void forward(int steps){
float side_dis=(8.4/steps);
float rangez=(steps/2);
float angular_dis=(side_dis*root3/2);
float zdis;
zdis=(6/rangez);
float x=-4.2,y=10,z=-7.5;
float X=0.2;
float Y,Z;
float XX=-8.6;
float YY,ZZ;
float p=4.2;
float q=10;
float P=8.6;
float Q,R=-7.5;
float PP=-0.2;
float QQ,RR=-7.5;
float r=-7.5;
  //Left and Right legs
 for (float i=1;i<=steps;i=i+1){
   Serial.print(rangez);
   Serial.print("\n");
   Serial.println(zdis,6);
   Serial.print("\n");
   Serial.print(steps);
   Serial.print("\n");
  
  x=x+side_dis;
  jp(x,y,z);

  X=X+angular_dis;
  Y=root3*X;
  Z=-7.5;
  jp2(X,Y,Z);


  XX=XX+angular_dis;
  YY=-root3*(XX);
  ZZ=-7.5;
  jp3(XX,YY,ZZ);
  //Code for second set of wheels

  

  p=p-side_dis;
  if (i<=rangez){
    r=r+zdis;
  }
  else{
    r=r-zdis;
  }
  jp4(p,q,r);
  
  P=P-angular_dis;
  Q=root3*P;
  if (i<=rangez){
    R=R+zdis;
  }
  else{
    R=R-zdis;
  }
  jp5(P,Q,R);
  

  PP=PP-angular_dis;
  QQ=-root3*(PP);
  if (i<=rangez){
    RR=RR+zdis;
  }
  else{
    RR=RR-zdis;
  }
  jp6(PP,QQ,RR);
  // delay(dt);
}
Serial.println("loop 2");
for (float i=1;i<=steps;i=i+1){
  x=x-side_dis;
  if (i<=rangez){
    z=z+zdis;
  }
  else{
    z=z-zdis;
  }
  jp(x,y,z);
  X=X-angular_dis;
  Y=root3*X;
  if (i<=rangez){
    Z=Z+zdis;
  }
  else{
    Z=Z-zdis;
  }
  jp2(X,Y,Z);
  XX=XX-angular_dis;
  YY=-root3*(XX);
  if (i<=rangez){
    ZZ=ZZ+zdis;
  }
  else{
    ZZ=ZZ-zdis;
  }
  jp3(XX,YY,ZZ);


  //CODE FOR SECOND SET OF LEGS
  p=p+side_dis;

  jp4(p,q,r);

  P=P+angular_dis;
  Q=root3*P;
  R=-7.5;
  jp5(P,Q,R);
  
  
  PP=PP+angular_dis;
  QQ=-root3*(PP);
  RR=-7.5;
  jp6(PP,QQ,RR);


  
  //delay(dt);
}
}

void ccw(int steps){
  float side_dis=(8./steps);
  float x=-4.,y=5,z=-7.5;
  float p=4.  ,q=5,r=-7.5;
  float rangez=(steps/2);
  float zdis;
  zdis=(6/rangez);  
  for (float i=1;i<=steps;i=i+1){
  if (i<=rangez){
    r=r+zdis;
  }
  else{
    r=r-zdis;
  }
  x=x+side_dis;
  p=p-side_dis;
  jpc(x,y,z);
  jpc2(x,y,z);
  jpc3(x,y,z);
  jpc4(p,q,r);
  jpc5(p,q,r);
  jpc6(p,q,r);
  }
  for (float i=1;i<=steps;i=i+1){
  x=x-side_dis;
  p=p+side_dis;  
  if (i<=rangez){
    z=z+zdis;
  }
  else{
    z=z-zdis;
  }
  jpc(x,y,z);
  jpc2(x,y,z);
  jpc3(x,y,z);
  jpc4(p,q,r);
  jpc5(p,q,r);
  jpc6(p,q,r);
  }
  

  
}
void cw(int steps){
  float side_dis=(8./steps);
  float x=-4.,y=5,z=-7.5;
  float p=4.  ,q=5,r=-7.5;
  float rangez=(steps/2);
  float zdis;
  zdis=(6/rangez);  
  for (float i=1;i<=steps;i=i+1){
  if (i<=rangez){
    r=r+zdis;
  }
  else{
    r=r-zdis;
  }
  x=x+side_dis;
  p=p-side_dis;
  jpc(p,q,r);
  jpc2(p,q,r);
  jpc3(p,q,r);
  jpc4(x,y,z);
  jpc5(x,y,z);
  jpc6(x,y,z);
  }
  for (float i=1;i<=steps;i=i+1){
  x=x-side_dis;
  p=p+side_dis;  
  if (i<=rangez){
    z=z+zdis;
  }
  else{
    z=z-zdis;
  }
  jpc(x,y,z);
  jpc2(x,y,z);
  jpc3(x,y,z);
  jpc4(p,q,r);
  jpc5(p,q,r);
  jpc6(p,q,r);
  }
  

  
}

void setup(){
  
  
  // Serial.begin(115200);
  // Serial3.begin(115200);
  
  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);
  HCPCA9685B.Init(SERVO_MODE);
  HCPCA9685B.Sleep(false);
  jp(0,10,-7.5);
  jp2(0,10,-7.5);
  jp3(0,10,-7.5);
  jp4(0,10,-7.5);
  jp5(0,10,-7.5);
  jp6(0,10,-7.5);

  Serial.begin(115200);
  Serial2.begin(115200);
 
}



void loop(){
  
while(!Serial.available()){

  };
  mjy=0;
  // Serial.flush();
  datax=Serial.readStringUntil(' ');
  jx=datax.toInt();
  datay=Serial.readStringUntil('\n');
  //Serial.flush();  
  jy=datay.toInt();
   
  Serial.print(jy);
  Serial.print("\n"); 
  mjy=map(abs(jy),20,100,50,20);
  mjx=map(abs(jx),20,100,50,20);
  
  if (6*abs(jy)>=abs(jx)){
    if (jy>0){
    Serial.end();
     forward(mjy*2);
    Serial.begin(115200); 
     }
    }   
  else if (6*abs(jy)<abs(jx)){
   if (jx<0){
     Serial.end();
     ccw(mjx*2);
     Serial.begin(115200);  
     
  }
  }
  else if (6*abs(jy)>abs(jx)){
   if (jx>0){
     Serial.end();
     cw(mjx*2);
     Serial.begin(115200);  
    
  }
  }
 //Serial.flush();
  
  
}