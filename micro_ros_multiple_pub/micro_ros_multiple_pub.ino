#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>                 //msg1类型对应的头文件,int32
#include <sensor_msgs/msg/imu.h>                //msg2类型对应的头文件,imu
#include <geometry_msgs/msg/twist.h>            //msg3类型对应的头文件,twist

rcl_publisher_t publisher1;                     //第1个publisher
rcl_publisher_t publisher2;                     //第2个publisher
rcl_publisher_t publisher3;                     //第3个publisher

std_msgs__msg__Int32 msg1;                      //msg1：int32类型
sensor_msgs__msg__Imu msg2;                     //msg2：imu类型
geometry_msgs__msg__Twist msg3;                 //msg3：twist类型

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// define 3 timer
rcl_timer_t timer1;
rcl_timer_t timer2;
rcl_timer_t timer3;

#define LED_PIN 27
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//timer1 callback
void timer1_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher1, &msg1, NULL));
    msg1.data++;
  }
}

//timer2 callback
void timer2_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
      /*在这里添加IMU的采集代码*/
      /*我用数据自加or自减的方式模拟*/
      /*ros2 interface show sensor_msgs/msg/Imu 查看IMU数据类型的详情*/
    msg2.linear_acceleration.x += 0.1;
    msg2.linear_acceleration.y += 0.1;
    msg2.linear_acceleration.z = 9.81f;
    msg2.angular_velocity.x += 0.01;
    msg2.angular_velocity.y += 0.01;
    msg2.angular_velocity.z += 0.01;
    msg2.header.stamp.sec += 1;
    msg2.header.stamp.nanosec += 1000;
    msg2.orientation_covariance[0] = -1;
    RCSOFTCHECK(rcl_publish(&publisher2, &msg2, NULL));
  }
}

//timer3 callback
void timer3_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher3, &msg3, NULL));
    static int cnt = 0;
    msg3.linear.x = 0.2;                            //const linear.x
    msg3.angular.z = 1.0 - 0.001 * cnt;             //variable angular.z
    cnt++;
  }
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher1
  RCCHECK(rclc_publisher_init_default(
            &publisher1,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "micro_ros_arduino_node_publisher1"));

  // create publisher2
  RCCHECK(rclc_publisher_init_default(
            &publisher2,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "micro_ros_arduino_node_publisher2"));

  // create publisher3
  RCCHECK(rclc_publisher_init_default(
            &publisher3,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "turtle1/cmd_vel"));

  // create timer1,
  const unsigned int timer1_timeout = 100;  //发布频率10Hz
  RCCHECK(rclc_timer_init_default(
            &timer1,
            &support,
            RCL_MS_TO_NS(timer1_timeout),
            timer1_callback));

  // create timer2,
  const unsigned int timer2_timeout = 1000; //发布频率1Hz
  RCCHECK(rclc_timer_init_default(
            &timer2,
            &support,
            RCL_MS_TO_NS(timer2_timeout),
            timer2_callback));

  // create timer3,
  const unsigned int timer3_timeout = 500;  //发布频率2Hz
  RCCHECK(rclc_timer_init_default(
            &timer3,
            &support,
            RCL_MS_TO_NS(timer3_timeout),
            timer3_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator)); 
   /*3个timer，故第三个参数为3*/

  RCCHECK(rclc_executor_add_timer(&executor, &timer1));   //添加timer1
  RCCHECK(rclc_executor_add_timer(&executor, &timer2));   //添加timer2
  RCCHECK(rclc_executor_add_timer(&executor, &timer3));   //添加timer3

  //   msg1初始化
  msg1.data = 0;

  //   msg2初始化
  msg2.header.frame_id.data = "IMUXX";
  msg2.header.frame_id.size = 5;

  //   msg3初始化
  msg3.linear.x = 0;
  msg3.linear.y = 0;
  msg3.linear.z = 0;
  msg3.angular.x = 0;
  msg3.angular.y = 0;
  msg3.angular.z = 0;
}

void loop() {
//  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}