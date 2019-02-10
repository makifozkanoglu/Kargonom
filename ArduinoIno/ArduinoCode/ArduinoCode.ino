#include <ros.h>
#include <std_msgs/Int16.h>
#include <L298N.h>
#include <odompackage/EncoderTick.h>

#define encoder_LF 7
#define encoder_RF 8

int Left_Ticks=0; 
int Right_Ticks=0;

odompackage::EncoderTick TicksMsg;
ros::Publisher TicksPub("TicksPublisher", &TicksMsg);


#define ENA 2
#define RIN1 10
#define RIN2 11

#define ENB 3
#define LIN1 12
#define LIN2 13

L298N lmotor(ENB, RIN1, RIN2);
L298N rmotor(ENA, LIN1, LIN2);

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}
 
// Use one Routine to handle each group
ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
 {  
  if (digitalRead(encoder_LF)){   // read the input pin
  Left_Ticks +=1;}
 }  
ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
 {  
 if(digitalRead(encoder_RF)){  // read the input pin
  Right_Ticks += 1 ;}
 }


ros::NodeHandle  nh;
int rwheel=0;
int lwheel=0;

void RmessageCb( const std_msgs::Int16& rwheelmsg){
rwheel=rwheelmsg.data;
}
void LmessageCb( const std_msgs::Int16& lwheelmsg){
lwheel=lwheelmsg.data;
}

ros::Subscriber<std_msgs::Int16> subr("/right_motor", RmessageCb );
ros::Subscriber<std_msgs::Int16> subl("/left_motor", LmessageCb );


void setup()
{
  pciSetup(8);
  pciSetup(7);
  nh.initNode();
  nh.advertise(TicksPub);
  nh.subscribe(subr);
  nh.subscribe(subl);
  rmotor.forward();
  lmotor.forward();
}

void loop()
{
  TicksMsg.left=Left_Ticks;
  TicksMsg.right=Right_Ticks;
  TicksPub.publish(&TicksMsg);
  if (rwheel>0){
    rmotor.forward();  
    }
  else {
    rmotor.backward(); 
    rwheel=-1*rwheel; 
    } 
  if (lwheel>0){
    lmotor.forward();  
    }
  else {
    lmotor.backward(); 
    lwheel=-1*lwheel; 
    }   
  rmotor.setSpeed(rwheel);
  lmotor.setSpeed(lwheel);
  nh.spinOnce();
  delay(10);
}
