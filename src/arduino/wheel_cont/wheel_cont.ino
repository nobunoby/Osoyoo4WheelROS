#include <ros.h>
#include <geometry_msgs/Twist.h>

#define speedPinR 9   //  RIGHT WHEEL PWM pin D45 connect front MODEL-X ENA 
#define RightMotorDirPin1  22    //Front Right Motor direction pin 1 to Front MODEL-X IN1  (K1)
#define RightMotorDirPin2  24   //Front Right Motor direction pin 2 to Front MODEL-X IN2   (K1)                                 
#define LeftMotorDirPin1  26    //Left front Motor direction pin 1 to Front MODEL-X IN3 (  K3)
#define LeftMotorDirPin2  28   //Left front Motor direction pin 2 to Front MODEL-X IN4 (  K3)
#define speedPinL 10   // Left WHEEL PWM pin D7 connect front MODEL-X ENB

#define speedPinRB 11   //  RIGHT WHEEL PWM pin connect Back MODEL-X ENA 
#define RightMotorDirPin1B  5    //Rear Right Motor direction pin 1 to Back MODEL-X IN1 (  K1)
#define RightMotorDirPin2B 6    //Rear Right Motor direction pin 2 to Back MODEL-X IN2 (  K1) 
#define LeftMotorDirPin1B 7    //Rear left Motor direction pin 1 to Back MODEL-X IN3  K3
#define LeftMotorDirPin2B 8  //Rear left Motor direction pin 2 to Back MODEL-X IN4  k3
#define speedPinLB 12    //   LEFT WHEEL  PWM pin D8 connect Rear MODEL-X ENB

#define WHEEL_RADIUS  0.04 // diameter 80mm
#define WHEEL_SEPARATION_WIDTH 0.107
#define WHEEL_SEPARATION_LENGTH 0.0465


ros::NodeHandle nh;

void callback(const geometry_msgs::Twist& vel)
{
  float lin_x, lin_y, ang_z;
  lin_x = vel.linear.x;
  lin_y = vel.linear.y;
  ang_z = vel.angular.z;

  int fl = (lin_x - lin_y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z) / WHEEL_RADIUS / 10 * 250;
  int fr = (1/WHEEL_RADIUS) * (lin_x + lin_y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z)/10*250;
  int rl = (1/WHEEL_RADIUS) * (lin_x + lin_y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z)/10*250;
  int rr = (1/WHEEL_RADIUS) * (lin_x - lin_y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z)/10*250;

  //nh.loginfo("receieved cmd_vel");
  FL(fl);
  FR(fr);
  RL(rl);
  RR(rr);
}

ros::Subscriber<geometry_msgs::Twist> sub("/ugv/cmd_vel", &callback);

void setup()
{
  init_GPIO();
  nh.initNode();
  nh.subscribe(sub);
}


void loop()
{
  nh.spinOnce();
  /*
  float lin_x, lin_y, ang_z;
  lin_x = 0.2;  // 10cm/s
  wheel_vel(lin_x, lin_y, ang_z);
  delay(3000);

  lin_y = 0.2;
  wheel_vel(lin_x, lin_y, ang_z);  // go_advance(250);
  delay(3000);
  
  brake();
  delay(5000);  
  */
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void wheel_vel(float lin_x, float lin_y, float ang_z)
{
  int fl = (lin_x - lin_y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z) / WHEEL_RADIUS / 10 * 250;
  int fr = (1/WHEEL_RADIUS) * (lin_x + lin_y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z)/10*250;
  int rl = (1/WHEEL_RADIUS) * (lin_x + lin_y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z)/10*250;
  int rr = (1/WHEEL_RADIUS) * (lin_x - lin_y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*ang_z)/10*250;

  FL(fl);
  FR(fr);
  RL(rl);
  RR(rr);
}
*/
void FL(int fl)
{
  int speed;
  if (fl >= 0)  // rotate fw
  {
    digitalWrite(LeftMotorDirPin1,LOW);
    digitalWrite(LeftMotorDirPin2,HIGH);
    speed = fl;
  }
  else if (fl < 0)  // rotate bw
  {
    digitalWrite(LeftMotorDirPin1,HIGH);
    digitalWrite(LeftMotorDirPin2,LOW);
    speed = -1 * fl;
  }
  analogWrite(speedPinL,speed);
}

void FR(int fr)
{
  int speed;
  if (fr >= 0)  // rotate fw
  {
    digitalWrite(RightMotorDirPin1,LOW);
    digitalWrite(RightMotorDirPin2,HIGH);
    speed = fr;
  }
  else if (fr < 0)  // rotate bw
  {
    digitalWrite(RightMotorDirPin1,HIGH);
    digitalWrite(RightMotorDirPin2,LOW);
    speed = -1 * fr;
  }
  analogWrite(speedPinR,speed);
}

void RL(int rl)
{
  int speed;
  if (rl >= 0)  // rotate fw
  {
    digitalWrite(LeftMotorDirPin1B,LOW);
    digitalWrite(LeftMotorDirPin2B,HIGH);
    speed = rl;
  }
  else if (rl < 0)  // rotate bw
  {
    digitalWrite(LeftMotorDirPin1B,HIGH);
    digitalWrite(LeftMotorDirPin2B,LOW);
    speed = -1 * rl;
  }
  analogWrite(speedPinLB,speed);
}

void RR(int rr)
{
  int speed;
  if (rr >= 0)  // rotate fw
  {
    digitalWrite(RightMotorDirPin1B,LOW);
    digitalWrite(RightMotorDirPin2B,HIGH);
    speed = rr;
  }
  else if (rr < 0)  // rotate bw
  {
    digitalWrite(RightMotorDirPin1B,HIGH);
    digitalWrite(RightMotorDirPin2B,LOW);
    speed = -1 * rr;
  }
  analogWrite(speedPinRB,speed);
}

void brake()
{
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
  digitalWrite(RightMotorDirPin1,LOW);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1B,LOW);
  digitalWrite(LeftMotorDirPin2B,LOW);
  digitalWrite(RightMotorDirPin1B,LOW);
  digitalWrite(RightMotorDirPin2B,LOW);
}

 
 
void stop_Stop()    //Stop
{
  analogWrite(speedPinLB,0);
  analogWrite(speedPinRB,0);
  analogWrite(speedPinL,0);
  analogWrite(speedPinR,0);
}


//Pins initialize
void init_GPIO()
{
  pinMode(RightMotorDirPin1, OUTPUT); 
  pinMode(RightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
 
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT);
  
  pinMode(RightMotorDirPin1B, OUTPUT); 
  pinMode(RightMotorDirPin2B, OUTPUT); 
  pinMode(speedPinLB, OUTPUT);  
 
  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT); 
  pinMode(speedPinRB, OUTPUT);
   
  stop_Stop();
}
