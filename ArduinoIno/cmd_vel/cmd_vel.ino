#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <AFMotor.h>


AF_DCMotor motor_left(4);
AF_DCMotor motor_right(3);


float L = 0.1; //distance between wheenls

ros::NodeHandle  nh;


void cmdVelCB( const geometry_msgs::Twist& twist)
{
  int gain = 700;
  float left_wheel_data = gain*(twist.linear.x + twist.angular.z*L);
  float right_wheel_data = gain*(twist.linear.x - twist.angular.z*L);
 
  motor_left.run(FORWARD);
  motor_left.setSpeed(left_wheel_data);
  
  motor_right.run(FORWARD);
  motor_right.setSpeed(right_wheel_data);
  Serial.print("left_wheel_data:");
  Serial.println(left_wheel_data);
  Serial.print("right_wheel_data:");
  Serial.println(right_wheel_data);
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("/cmd_vel", cmdVelCB);
  
void setup()
{
  Serial.begin(9600);           // set up Serial library at 9600 bps
  nh.initNode();
  nh.subscribe(subCmdVel);
  motor_left.setSpeed(200);
  motor_left.run(RELEASE);
  motor_right.setSpeed(200);
  motor_right.run(RELEASE);
  }
  
 void loop()
 {
  nh.spinOnce();
}
