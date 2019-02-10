/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/Int16.h>
#include <L298N.h>

#define ENA 2
#define RIN1 10
#define RIN2 11

#define ENB 3
#define LIN1 12
#define LIN2 13

L298N lmotor(ENB, RIN1, RIN2);
L298N rmotor(ENA, LIN1, LIN2);


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
  nh.initNode();
  nh.subscribe(subr);
  nh.subscribe(subl);
  rmotor.forward();
  lmotor.forward();
}

void loop()
{
  nh.spinOnce();
  delay(10);
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
}
