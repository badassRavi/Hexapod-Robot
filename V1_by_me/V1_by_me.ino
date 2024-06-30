#define S_RXD 18
#define S_TXD 19

#include <SCServo.h>
#include <math.h>
SCSCL sc;

const float t(120.0), m(102.5), _2tm(24600.0), t2(14400.0), m2(10506.25), _2m(205.0), z_comp(26.2), y_comp(52.2), pi(3.14159265);
const int endpoint_low[6][4] = {{250,110,15,900},
                                 {230,105,60,880},
                                 {240,85,70,890},
                                 {215,45,30,930},
                                 {240,70,50,910},
                                 {275,85,60,890}};
const int endpoint_high[6][4] = {{900,970,875,40},
                                 {880,965,920,20},
                                 {890,945,935,30},
                                 {865,905,890,70},
                                 {890,930,910,50},
                                 {925,945,920,30}};

const float off_set[6] = {-60., 0.0 , 60.0, 120.0, 180.0, -120.0};
int pos[6][4], load[6][4], id[6][4], loc(0), max_trajec_pts, move;
float x[6],  y[6], z[6], r[6], lambda, theta, beta, gama, delta, last_ang, curr_ang;
unsigned long ti,tf;
 

void print()
{
  for(int i=0;i<=0;i++)
  {
    for(int j = 1; j<=4;j++)
    {
      Serial.print(" Servo ID : ");
      Serial.print(i+j);
      Serial.print(" Pos : ");
      Serial.print(pos[i][i+j]);
      Serial.print(" Load : ");
      Serial.print(load[i][i+j]);
      Serial.print("\n");
    }
  }
}

float todeg(float rad)
{
  return (rad/pi)*180.0;
}

void print_ang(int leg)
{
  Serial.print(leg);
  Serial.print(" r : ");
  Serial.print(r[leg]);
  Serial.print(" Theta : ");
  Serial.print(todeg(theta));
  Serial.print(" Lambda : ");
  Serial.print(todeg(lambda));
  Serial.print(" Beta : ");
  Serial.print(todeg(beta));
  Serial.print(" Gamma : ");
  Serial.print(todeg(gama));
  Serial.print(" Delta : ");
  Serial.print(todeg(delta));
  Serial.print(" Time taken (us) : ");
  Serial.print(tf-ti);
  Serial.print("\n");
}
void ik(int leg)
{
  ti = micros();
  float temp1 = (sqrt((x[leg]*x[leg]) + (y[leg]*y[leg])) - y_comp);
  float temp2 = (z_comp-z[leg]);
  r[leg] = sqrt( temp2*temp2 + temp1*temp1);
  // Serial.print(r[leg]);
  // Serial.print(" ");
  if(r[leg] >=222.5) return;
  float r2 = r[leg]*r[leg];
  //calculating theta
  theta = asin((z_comp - z[leg])/r[leg]);

  //calculating lambda
  if(x[leg])  //for non zero values of x[leg]
  {
    lambda = atan(y[leg]/x[leg]);
    if(lambda<0)
    {
      lambda = pi + lambda;
    }
  }
  else 
  {
    lambda = pi/2.0;
  }
  // Serial.print(lambda);
  // Serial.print(" ");
  if(lambda<0 || lambda>pi) return; //this is to prevent any bizzare movement which may result in something breaking.

  //calculating beta
  beta = acos((t2+m2-r2)/_2tm);
  // Serial.print(beta);
  // Serial.print(" ");
  if(beta<0.5890485 || beta > pi ) return; //this is to prevent any bizzare movement which may result in something breaking.

  //calculating gamma
  gama = acos( (m2+r2-t2) / (_2m*r[leg]) );

  //calculating delta
  delta = (pi/2.0) + theta - gama;
  // Serial.print(delta);
  // Serial.print(" ");
  if(delta<0 || delta>pi) return; //this is to prevent any bizzare movement which may result in something breaking.
  tf = micros();
  pos[leg][0] = map(todeg(beta),33.75,180,endpoint_low[leg][0],endpoint_high[leg][0]);
  Serial.print(id[leg][0]);
  Serial.print(" ");
  pos[leg][2] = map(todeg(delta),0,180,endpoint_low[leg][2],endpoint_high[leg][2]);
  Serial.print(id[leg][2]);
  Serial.print(" ");
  pos[leg][3] = map(todeg(lambda),0,180,endpoint_low[leg][3],endpoint_high[leg][3]);
  Serial.print(id[leg][3]);
  Serial.print(" ");
  sc.WritePos(id[leg][0], pos[leg][0], 0, 500);
  delay(10);
  sc.WritePos(id[leg][2], pos[leg][2], 0, 500);
  delay(10);
  sc.WritePos(id[leg][3], pos[leg][3], 0, 500);
  delay(10);
  print_ang(leg);
}

void generate_trajec(float deg)
{
  float m_slope = tan((deg/180)*pi);
  float m_slope_2 = m_slope*m_slope;
  float m_disp = ( (100*(1+m_slope_2)) / (1+(16*m_slope_2)) );
  float t_slope = tan((deg+60/180)*pi);
  float t_slope_2 = t_slope*t_slope;
  float t_disp = ( (100*(1+t_slope_2)) / (1+(16*t_slope_2)) );
  float b_slope = tan((deg-60/180)*pi);
  float b_slope_2 = b_slope*b_slope;
  float b_disp = ( (100*(1+b_slope_2)) / (1+(16*b_slope_2)) );
  float disp = std::min({m_disp , t_disp , b_disp});
}

// void coordinate(int leg, float deg)
// {
//     float m = tan((deg/180)*pi);
//     float x = 10./sqrt(1+(16*m*m));
//     float y = (10*m)/sqrt(1+(16*m*m));

//     float x_ep1 = x;
//     float x_ep2 = -x;
//     float y_ep1 = y;
//     float y_ep2 = -y;

//     float r = 5;
//     //float targx = x[leg] + r*cos((deg/180)*pi);
// }

// void drive()
// {
//   for(int leg = 0; leg<1;leg++)
//   {
//     x[leg] = x_trjec[loc];
//     y[leg] = y_trjec[loc];
//     z[leg] = z_trjec[loc];
//     ik(leg);
//   }
//   if(loc == max_trajec_pts) loc=0;
//   else loc++;
// }


void boot_seq()
{
  for(int i =0;i<6;i++)
  {
    x[i] = 0.;
    y[i] = 180.;
    z[i] = -100.;
    ik(i);
  }
}


void setup() {
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  Serial.begin(115200);
  sc.pSerial = &Serial1;
  delay(1000);
  for(int i=0;i<6;i++)
  {
    for(int j = 1; j<=4;j++)
    {
      id[i][j-1]=(4*i+j);
      pos[i][j-1]=sc.ReadPos(4*i+j);
      delay(10);
      load[i][j-1]=sc.ReadLoad(4*i+j);
      delay(10);
    }
    sc.WritePos(id[i][1], int(endpoint_low[i][1]+430) , 0, 500);
    delay(10);
  }
  boot_seq();
}


void loop() {
  if(Serial.available()>0)
  {
    // curr_ang = Serial.readStringUntil('\r').toFloat();
    // move = 1;
    int leg = Serial.readStringUntil(' ').toFloat();
    x[leg] = Serial.readStringUntil(' ').toFloat();
    y[leg] = Serial.readStringUntil(' ').toFloat();
    z[leg] = Serial.readStringUntil('\r').toFloat();
    Serial.print(x[leg]);
    Serial.print(" ");
    Serial.print(y[leg]);
    Serial.print(" ");
    Serial.print(z[leg]);
    Serial.print("\n");
    ik(leg);
  }
  // x[0]=0;
  // y[0]=150;
  // z[0]=-50;
  // ik(0);
  // delay(1000);
  // z[0]=-170;
  // ik(0);
  // delay(1000);
  // if(curr_ang != last_ang)
  // {
  //   generate_trajec(curr_ang);
  //   drive();
  // }
  // else  drive();

}