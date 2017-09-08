#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "Adafruit_VL53L0X.h"
#include <ZerooneSupermodified.h>
#include <Wire.h>
#include <Servo.h>

#define rMotID 5
#define lMotID 7

ZerooneSupermodified motor(ZO_HW_WIRE);

ros::NodeHandle  nh;

float linearVel;
float angularVel;

void motVel( const geometry_msgs::Twist& cmd_vel){
  linearVel = cmd_vel.linear.x * 1000;
  angularVel = cmd_vel.angular.z * 1000;
  motor.moveWithVelocity(rMotID,linearVel + angularVel);
  motor.moveWithVelocity(lMotID,-linearVel + angularVel);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", motVel);

sensor_msgs::LaserScan range_msg;
ros::Publisher pub_range( "range_data", &range_msg);

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
Servo myservo;

int pos = 0;
int j = 0;
float distance[45];

char frameid[] = "/laser";

char info[] = "VL53L0X API Intialised";
char error[] = "Failed to boot VL53L0X";

void setup()
{  
  nh.initNode();
  nh.advertise(pub_range);
  nh.subscribe(sub);
  
  range_msg.header.frame_id =  frameid;
  range_msg.angle_min = -1.57;
  range_msg.angle_max = 1.57;
  range_msg.angle_increment = 0.0698;
  range_msg.time_increment = 0.022;
  range_msg.scan_time = 0.99;
  range_msg.range_min = 0.02;
  range_msg.range_max = 1.4;
  range_msg.ranges_length = 45;

  if (!lox.begin()) {
    nh.logerror(error);
  }
  else{
    nh.loginfo(info);
  }

  myservo.attach(9);
  motor.start(rMotID);
  motor.start(lMotID);
}

void loop()
{
  for (pos = 0; pos <= 180; pos += 4) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    j = pos/4;
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
    if (measure.RangeStatus != 4) {
      distance[j] = measure.RangeMilliMeter * 0.001;      
    }
    else {
      distance[j] = -1;
    }
    delay(22);
  }
  range_msg.ranges = distance;
  range_msg.header.stamp = nh.now();
  pub_range.publish(&range_msg);

  for (pos = 180; pos >= 0; pos -= 4) {
    j = pos/4;
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    if (measure.RangeStatus != 4) {
      distance[j] = measure.RangeMilliMeter * 0.001;      
    }
    else {
      distance[j] = -1;
    }
        
    delay(22);
  }
  range_msg.ranges = distance;
  range_msg.header.stamp = nh.now();
  pub_range.publish(&range_msg);
  
  nh.spinOnce();
}

