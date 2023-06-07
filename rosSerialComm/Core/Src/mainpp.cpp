/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include "main.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <string>
#include <cstring>



ros::NodeHandle nh;

void str_act_msg(const std_msgs::String& msg);

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg																						);
ros::Subscriber<std_msgs::String> stm32_comms("gripper_action", &str_act_msg);
std::string hello = "STM32 to Jetson!";

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(stm32_comms);
}

void loop(void)
{
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  const char* str = hello.c_str();
  str_msg.data = str;
  chatter.publish(&str_msg);
  nh.spinOnce();

  HAL_Delay(1000);
}

void str_act_msg(const std_msgs::String& msg){
	hello = msg.data;
}
