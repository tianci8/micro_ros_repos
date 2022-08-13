#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>                //msg1类型对应的头文件,imu

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;

rcl_publisher_t publisher1;                     //第2个publisher

sensor_msgs__msg__Imu msg1;                     //msg2：imu类型


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// define timer
rcl_timer_t timer1;

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
    /*在这里添加IMU的采集代码*/
    /*我用数据自加or自减的方式模拟*/
    /*ros2 interface show sensor_msgs/msg/Imu 查看IMU数据类型的详情*/


    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    msg1.linear_acceleration.x =a.acceleration.x;
    msg1.linear_acceleration.y =a.acceleration.y;
    msg1.linear_acceleration.z = a.acceleration.z;
    msg1.angular_velocity.x =g.gyro.x;
    msg1.angular_velocity.y =g.gyro.y;
    msg1.angular_velocity.z =g.gyro.z;
    msg1.header.stamp.sec += 01;
    msg1.header.stamp.nanosec += 100;
    msg1.orientation_covariance[0] = -1;
    RCSOFTCHECK(rcl_publish(&publisher1, &msg1, NULL));
  }
}


void setup() {
  set_microros_transports();
  delay(2000);
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    delay(100);

  }
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

  // create publisher1
  RCCHECK(rclc_publisher_init_default(
            &publisher1,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "imu6050"));

  // create timer1,
  const unsigned int timer1_timeout = 100; //发布频率10Hz
  RCCHECK(rclc_timer_init_default(
            &timer1,
            &support,
            RCL_MS_TO_NS(timer1_timeout),
            timer1_callback));


  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  RCCHECK(rclc_executor_add_timer(&executor, &timer1));   //添加timer2


  //   msg1初始化
  msg1.header.frame_id.data = "IMU6050";
  msg1.header.frame_id.size = 10;
  msg1.linear_acceleration.x = 0;
  msg1.linear_acceleration.y = 0;
  msg1.linear_acceleration.z = 0;
  msg1.angular_velocity.x = 0;
  msg1.angular_velocity.y = 0;
  msg1.angular_velocity.z = 0;
}

void loop() {
  //  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
