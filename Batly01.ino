#include <ArduinoHardware.h>
#include <ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <Encoder.h>

#define COMMAND_RATE 20 //hz

#define Motor1A 4
#define Motor1B 5

#define Motor2A 10
#define Motor2B 11

#define Motor3A 8
#define Motor3B 9

#define Motor4A 6
#define Motor4B 7


Encoder Enc_A(2, 3);
Encoder Enc_B(22, 23);
Encoder Enc_C(14, 15);
Encoder Enc_D(20, 21);


ros::NodeHandle nh;
geometry_msgs::Twist msg;
geometry_msgs::Twist enc;

ros::Publisher Enc("dist", &enc);


float x=0;
float y=0;
float z=0;

unsigned long g_prev_command_time = 0;
void roverCallBack(const geometry_msgs::Twist& cmd_vel)
{
  float x = cmd_vel.linear.x;
  float y = cmd_vel.linear.y;
  float z = cmd_vel.angular.z;
 
    float spee = (x+y)/z ;
   float movero = x>0.0 and y >0.0;
   float movere = x<0.0 and y <0.0;
   float movego = x ;
   float moveleft = y ;
   float moveleftfor = x>0.0 and y < 0.0 ;
   float moverightback = x < 0.0 and y >0.0;
   float moveth = z ;
  
   
   /////////////////////////////////////////////////////////////////////
   if(moveth>0.0) //rostation
   {
     Right(max(min(moveth*10,100),35));
     // Right(max(min(abs(moveth*10),100),35));
      
    }
     else if(moveth<0.0)
    {
      left(max(min(abs(moveth*10),100),30));
      
    }
   ///////////////////////////////////////////////////////////////////////////
   if(movego>0.0)
   {
    forward(max(min(movego*10,100),30));
    //forward(max(min(abs(movego*10),100),30));
    }
    else if(movego<0.0)
     {
    backward(max(min(abs(movego*10),100),30));
    }
  //////////////////////////////////////////////////////////////////////////////////////////////   
   if(moveleft>0.0)
   {
    slideleft(max(min(moveleft*100,100),30));
    //slideleft(max(min(abs(moveleft*10),100),30));
    }
    else if(moveleft<0.0)
     {
    slideRight(max(min(abs(moveleft*100),100),30));
    }
  ///////////////////////////////////////////////////////////
   if(moveleftfor)
   {
    leftForward(max(min (abs(spee*10),100),30));
    }
    else if(moverightback)
     {
    Rightbackward(max(min(abs(spee*10),100),30));
    }
 /////////////////////////////////////////////////////////////////////////////
 if(movero)
   {
    RightForward(max(min(abs(spee*10),100),30));
    }
    else if(movere)
     {
    leftbackward(max(min(abs(spee*10),100),30));
    }


 
 
   g_prev_command_time = millis();
}
  
ros::Subscriber <geometry_msgs::Twist> Motor("/cmd_vel", roverCallBack);


long encPos_A;
long encPos_B;
long encPos_C;
long encPos_D;


void setup() {
  pinMode(Motor1A,OUTPUT); pinMode(Motor1B,OUTPUT);
  pinMode(Motor2A,OUTPUT); pinMode(Motor2B,OUTPUT);
  pinMode(Motor3A,OUTPUT); pinMode(Motor3B,OUTPUT);
  pinMode(Motor4A,OUTPUT); pinMode(Motor4B,OUTPUT);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(Motor);
  nh.advertise(Enc);
 
}



void loop() 
{ 
  // encoder();
  // printDebug();
  static unsigned long prev_control_time = 0;
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
        encoder();
        //printDebug();
        prev_control_time = millis();
    }
  
   if((millis() - g_prev_command_time) >= 400 )
    {
     stop();
        
     g_prev_command_time = millis();
   
    }
    
 
  nh.spinOnce();
  delay(10);
   

}

void encoder()
{
  
  double f1, f2, f3 , f4;
  float dWheel = 0.098; //ความกว้างล่อ
  float pi = 3.14;
  float wheelD = 0;
  float fr_wheels_dist = 0.32/2;//ความยาวถึงล่อ
  float lr_wheels_dist = 0.335/2;
  int reso = 1200;//660; //3300;//3500 //
  
  //encPos_A = Enc_A.read();
  //encPos_B = Enc_B.read();
  //encPos_C = Enc_C.read();
  //encPos_D = Enc_D.read();
   encPos_A = Enc_A.readAndReset();
   encPos_B = Enc_B.readAndReset();
   encPos_C = Enc_C.readAndReset();
   encPos_D = Enc_D.readAndReset();
  
   wheelD = dWheel * pi;
  f1 = (wheelD * encPos_A) / reso;
  f2 = (wheelD * encPos_B) / reso;
  f3 = (wheelD * encPos_C) / reso;
  f4 = (wheelD * encPos_D) / reso;

  float average_rpm_x = (float)(f1 + f2 + f3 + f4) *(60/ 4); // RPM
  //convert revolutions per minute to revolutions per second
  float linear_x = ( average_rpm_x * wheelD); // m/s
  

  float average_rpm_y = (float)(-f1 + f2 + f3 - f4)*( 60 / 4); // RPM
  //convert revolutions per minute in y axis to revolutions per second
  //float rps = average_rpm_y / 60;
  float linear_y = (average_rpm_y * wheelD); // m/s

  float average_rpm_a = (float)(-f1 + f2 - f3 + f4) *(60/(4*(fr_wheels_dist + lr_wheels_dist)));
  //convert revolutions per minute to revolutions per second
 // float rpz = average_rpm_a / 60;
  float angular_z = (average_rpm_a  * wheelD);
  
  enc.linear.x = linear_x;
  enc.linear.y =  linear_y;
  enc.angular.z =  angular_z;
  Enc.publish(&enc);

}
void printDebug()
{
//    char buffer[50];
//
//    sprintf (buffer, "Encoder FrontLeft  : %ld", Enc_A.read());
//    nh.loginfo(buffer);
//    sprintf (buffer, "Encoder FrontRight : %ld",Enc_B.read());
//    nh.loginfo(buffer);
//    sprintf (buffer, "Encoder RearLeft   : %ld", Enc_C.read());
//    nh.loginfo(buffer);
//    sprintf (buffer, "Encoder RearRight  : %ld", Enc_D.read());
//    nh.loginfo(buffer);
}
