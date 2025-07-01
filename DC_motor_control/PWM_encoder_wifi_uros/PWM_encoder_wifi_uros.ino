/*

Get the current angular position and velocity of the motor (DC6V210RPM - Reducer 20:1)
and command the desired PWM to the the motor (DC6V210RPM - Reducer 20:1)
Author: Juan M. Gandarias
email: jmgandarias@uma.es

*/

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/int32.h>

#define ENCODER_A 26
#define ENCODER_B 27

#define PWM_CW_PIN 32
#define PWM_CCW_PIN 33

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_subscription_t subscriber;
std_msgs__msg__Int32 PWM_msg;

rcl_publisher_t pos_publisher;
std_msgs__msg__Float64 pos_msg;
rcl_publisher_t vel_publisher;
std_msgs__msg__Float64 vel_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// PWM configuration
const int pwm_cw_channel = 0;
const int pwm_ccw_channel = 1;
const int frequency = 1000;
const int resolution = 10;

volatile int32_t cmd_PWM = 0;

//Encoder counter
volatile int counter;
volatile int last_count = 0;

// Store the last time
volatile int64_t last_time = 0;

// Convert to radians
const float pulsesPerRevolution = 880.0;
const float radiansPerPulse = 2 * PI / pulsesPerRevolution;


ICACHE_RAM_ATTR void ISRENCODER_A() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    counter++;
  } else {
    counter--;
  }
}

ICACHE_RAM_ATTR void ISRENCODER_B() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    counter--;
  } else {
    counter++;
  }
}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

    // Get current time in milliseconds
    int64_t current_time = rmw_uros_epoch_millis();

    // Publish current pos
    pos_msg.data = counter * radiansPerPulse;
    RCSOFTCHECK(rcl_publish(&pos_publisher, &pos_msg, NULL));

    long delta_pos_ppr = counter - last_count;
    float elapsed_time = (current_time - last_time) / 1000.0;  // convert ms to seconds
    float delta_pos = delta_pos_ppr * radiansPerPulse;

    // Publish current vel
    vel_msg.data = delta_pos / elapsed_time;
    RCSOFTCHECK(rcl_publish(&vel_publisher, &vel_msg, NULL));

    // Update last values
    last_count = counter;
    last_time = current_time;
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
  set_microros_wifi_transports("WIFI_SSID", "WIFI_PSSWD", "PC_IP", 8888);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  ledcSetup(pwm_cw_channel, frequency, resolution);
  ledcAttachPin(PWM_CW_PIN, pwm_cw_channel);

  ledcSetup(pwm_ccw_channel, frequency, resolution);
  ledcAttachPin(PWM_CCW_PIN, pwm_ccw_channel);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), ISRENCODER_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), ISRENCODER_B, CHANGE);

  counter = 0;

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "dc_motor_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "cmd_PWM"));

  // create pos_publisher
  RCCHECK(rclc_publisher_init_default(
    &pos_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "current_pos"));

  // create vel_publisher
  RCCHECK(rclc_publisher_init_default(
    &vel_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "current_vel"));

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // we need to tell the executor to allocate enough internal resources to handle 2 entities (1 timer and 1 subscriber)
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &PWM_msg, &subscription_callback, ON_NEW_DATA));

  pos_msg.data = 0;
  last_time = rmw_uros_epoch_millis();
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
