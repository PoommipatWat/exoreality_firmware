#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>

#include "config.h"

#include "imu_bno055.h"

#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "pid.h"
#include "motor.h"

#define LED_PIN 13
#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#endif
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_init_options_t init_options;

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t rpm_publisher;
geometry_msgs__msg__Twist rpm_msg;

rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

IMU_BNO055 bno055;

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}

// Reboot the board (Teensy Only)
void doReboot() {
  SCB_AIRCR = 0x05FA0004;
}

void rclErrorLoop() 
{
  flashLED(2);
  doReboot();
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis(); 
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

void imu_pub(){
  bno055.getIMUData(imu_msg);

  struct timespec time_stamp = getTime();
  imu_msg.header.stamp.sec = time_stamp.tv_sec;
  imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  imu_msg.header.frame_id.data = "imu_link";

  imu_msg.angular_velocity_covariance[0] = 0.0001;
  imu_msg.angular_velocity_covariance[4] = 0.0001;
  imu_msg.angular_velocity_covariance[8] = 0.0001;

  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[8] = 0.0025;

  rcl_publish(&imu_publisher, &imu_msg, NULL);
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
}

void motor_control(){
  if(((millis() - prev_cmd_time) >= 5000)) 
  {
      fullStop();
//      digitalWrite(LED_PIN, HIGH);
  }

  float target_rpm_motor1, target_rpm_motor2, target_rpm_motor3;
  target_rpm_motor1 = twist_msg.linear.x;
  target_rpm_motor2 = twist_msg.linear.y;
  target_rpm_motor3 = twist_msg.linear.z;

  float current_rpm_motor1, current_rpm_motor2, current_rpm_motor3;
  current_rpm_motor1 = motor1_encoder.getRPM();
  current_rpm_motor2 = motor2_encoder.getRPM();
  current_rpm_motor3 = motor3_encoder.getRPM();

  motor1_controller.spin(motor1_pid.compute(target_rpm_motor1, current_rpm_motor1));
  motor2_controller.spin(motor2_pid.compute(target_rpm_motor2, current_rpm_motor2));
  motor3_controller.spin(motor3_pid.compute(target_rpm_motor3, current_rpm_motor3));


  rpm_msg.linear.x = current_rpm_motor1;
  rpm_msg.linear.y = current_rpm_motor2;
  rpm_msg.linear.z = current_rpm_motor3;
  rcl_publish(&rpm_publisher, &rpm_msg, NULL);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    imu_pub();
    motor_control();
  }
}

void twist_callback(const void * msgin) 
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    twist_msg = *msg;  // Update the global twist_msg with the new data
    prev_cmd_time = millis();
}

bool create_entities()
{
  flashLED(3);

  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 10);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // create node
  RCCHECK(rclc_node_init_default(&node, "microros_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"));

  RCCHECK(rclc_publisher_init_best_effort(
    &rpm_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "rpm"));

  // create timer,
  const unsigned int timer_timeout = 20;    // in ms (50 Hz)  
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create twist command subscriber
  RCCHECK(rclc_subscription_init_default( 
      &twist_subscriber, 
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel/rpm"
  ));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
      &executor, 
      &twist_subscriber, 
      &twist_msg, 
      &twist_callback, 
      ON_NEW_DATA
  ));

  syncTime();

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&rpm_publisher, &node);
  rcl_subscription_fini(&twist_subscriber, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  fullStop();

  flashLED(5);
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  pinMode(LED_PIN, OUTPUT);
  state = WAITING_AGENT;

  bno055.init();

  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}
