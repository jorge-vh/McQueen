#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;

Servo servo_joint;
Servo servo_gripper;

void servo_joint_cb(const std_msgs::UInt16 &cmd_msg)
{
  servo_joint.write(cmd_msg.data); // Set servo angle, should be from 0-180
  digitalWrite(13, HIGH - digitalRead(13)); // Toggle LED
}

void servo_gripper_cb(const std_msgs::UInt16 &cmd_msg)
{
  servo_gripper.write(cmd_msg.data); // Set servo angle, should be from 0-180
  // Additional logic for controlling the gripper servo
}

ros::Subscriber<std_msgs::UInt16> sub_joint("servo_joint", servo_joint_cb);
ros::Subscriber<std_msgs::UInt16> sub_gripper("servo_gripper", servo_gripper_cb);

void setup()
{
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub_joint);
  nh.subscribe(sub_gripper);

  servo_joint.attach(9);   // Attach servo_joint to pin 9
  servo_gripper.attach(10); // Attach servo_gripper to pin 10
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
