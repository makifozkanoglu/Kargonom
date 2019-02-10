#include <ros.h>
#include <odompackage/EncoderTick.h>

#define encoder_LF 7
#define encoder_RF 8

int Left_Ticks=0; 
int Right_Ticks=0;

ros::NodeHandle  nh;

odompackage::EncoderTick TicksMsg;
ros::Publisher TicksPub("TicksPublisher", &TicksMsg);

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



void setup()
{
  pciSetup(8);
  pciSetup(7);
  nh.initNode();
  nh.advertise(TicksPub);
}

void loop()
{
TicksMsg.left=Left_Ticks;
TicksMsg.right=Right_Ticks;
TicksPub.publish(&TicksMsg);
nh.spinOnce();
delay(100);
}

