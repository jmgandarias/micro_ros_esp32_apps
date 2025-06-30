/*

Command the desired PWM to the the motor (DC6V210RPM - Reducer 20:1)
Author: Juan M. Gandarias
email: jmgandarias@uma.es

*/

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_subscription_t subscriber;
std_msgs__msg__Int32 PWM_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define PWM_CW_PIN 32
#define PWM_CCW_PIN 33

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// PWM configuration
const int pwm_cw_channel = 0;
const int pwm_ccw_channel = 1;
const int frequency = 1000;
const int resolution = 10;

volatile int32_t cmd_PWM = 0;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * PWM_msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));  
  cmd_PWM = PWM_msg->data;

  if (cmd_PWM >=0){
    if (cmd_PWM>1023){
      cmd_PWM = 1023;
    }
    ledcWrite(pwm_cw_channel, cmd_PWM);
    ledcWrite(pwm_ccw_channel, 0);
  }else{
    cmd_PWM = -cmd_PWM;
    if (cmd_PWM>1023){
      cmd_PWM = 1023;
    }
    ledcWrite(pwm_ccw_channel, cmd_PWM);
    ledcWrite(pwm_cw_channel, 0);
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  ledcSetup(pwm_cw_channel, frequency, resolution);
  ledcAttachPin(PWM_CW_PIN, pwm_cw_channel);

  ledcSetup(pwm_ccw_channel, frequency, resolution);
  ledcAttachPin(PWM_CCW_PIN, pwm_ccw_channel);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "pwm_motor_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "cmd_PWM"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &PWM_msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
