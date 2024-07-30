#include <Arduino.h>

#include <ESP32Servo.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_srvs/srv/set_bool.h>

rcl_allocator_t allocator;
rclc_executor_t executor;
rclc_support_t support;

rcl_node_t node;
rcl_service_t servo_service;
std_srvs__srv__SetBool_Request req;
std_srvs__srv__SetBool_Response res;
rcl_node_options_t node_ops = rcl_node_get_default_options();
// node_ops.domain_id = 10;
// RCCHECK(rclc_node_init_with_options(&node, "my_node_name", "", &support, &node_ops));

Servo myservo; // Declare a variable to control the servo

void servo_callback(const void * req, void * res)
{
  const std_srvs__srv__SetBool_Request * request = (const std_srvs__srv__SetBool_Request *)req;
  std_srvs__srv__SetBool_Response * response = (std_srvs__srv__SetBool_Response *)res;

  if (request->data) {
    myservo.writeMicroseconds(2000); // Command servo to rotate left
    delay(650); // Wait for 700ms
    myservo.writeMicroseconds(1500); // Command servo to stop
    delay(1000); // Wait for 1000ms
    myservo.writeMicroseconds(1000); // Command servo to rotate right
    delay(650); // Wait for 700ms
    myservo.writeMicroseconds(1500); // Command servo to stop
    delay(1000); // Wait for 1000ms
    response->message.data = (char *)"Servo start.";
  } else {
    myservo.writeMicroseconds(1500); // Command servo to stop
    response->message.data = (char *)"Servo stopped.";
  }
  response->success = true;
}

void setup() {
  set_microros_transports();

  myservo.attach(17); // Attach the servo to pin 12

  allocator = rcl_get_default_allocator();

  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);
  // Create node
  rclc_node_init_default(&node, "servo_node", "", &support);

  // Create service
  rclc_service_init_default(
    &servo_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
    "servo_service");

  // Create executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_service(&executor, &servo_service, &req, &res, &servo_callback);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}