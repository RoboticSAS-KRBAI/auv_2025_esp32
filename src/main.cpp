#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_INA219.h>

#define SCREEN_WIDTH 128 // display display width, in pixels
#define SCREEN_HEIGHT 64 // display display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define display_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, display_RESET);
Adafruit_INA219 ina219;

/////////////////////
/// For Micro ROS ///
/////////////////////
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

// Message headers
#include <std_msgs/msg/string.h>
#include <auv_interfaces/msg/object_difference.h>
#include <auv_interfaces/msg/sensor.h>

/*
 * Helper functions to help reconnect
*/
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Declare rcl object
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

// Declare Publishers
rcl_publisher_t pub_pwm, pub_error, pub_sensor, pub_set_point, pub_pid, pub_status, pub_boost;

// Declare Subscribers
rcl_subscription_t sub_status, sub_sensor, sub_object_difference;

// Declare Messages
std_msgs__msg__String status_msg;
auv_interfaces__msg__ObjectDifference object_difference_msg;
auv_interfaces__msg__Sensor sensor_msg;

// Variable sensor and logic
float yaw = 0.0, busVoltage = 0.0;
String Status = "stop", Object_type = "None", prevStatus = "", prevObjectDiff = "";
unsigned long lastSensorUpdate = 0;
const unsigned long sensorUpdateInterval = 100;


// Callback functions
bool receive_status = false;
void status_callback(const void *msgin) {
  const std_msgs__msg__String *status_msg = (const std_msgs__msg__String *)msgin;
  
  // receive message
  String received_status = String(status_msg->data.data);
  Status = received_status;  // Menyimpan data status yang diterima
  receive_status = true;
}

void object_difference_callback(const void *msgin) {
  const auv_interfaces__msg__ObjectDifference *object_difference_msg = (const auv_interfaces__msg__ObjectDifference *)msgin;
  
  Object_type = object_difference_msg->object_type.data;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    
    if(receive_status) {
      // Publish
      rcl_publish(&pub_status, &status_msg, NULL);

      receive_status = false;
    }

    // if (Status != prevStatus) {
    //   display.setTextSize(1);
    //   display.setCursor(0, 0);
    //   display.print("                ");
    //   display.setCursor(0, 0);
    //   display.print("S: "); display.print(Status);
    //   prevStatus = Status;
    //   display.display();
    // }

  }
}

// Micro-ROS functions
bool create_entities() {
  const char * node_name = "teensy_node";
  const char * ns = "";
  const int domain_id = 0;

  // Initialize node

  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, domain_id);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, node_name, ns, &support);

  // Initialize publishers
    rclc_publisher_init(
        &pub_status,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "status_msg", &rmw_qos_profile_default);

    // Initialize subscribers
    rclc_subscription_init(
        &sub_status,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "status", &rmw_qos_profile_default);
    
    // rclc_subscription_init(
    //     &sub_sensor,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    //     "sensor", &rmw_qos_profile_default);
    
    // rclc_subscription_init(
    //     &sub_object_difference,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    //     "object_different", &rmw_qos_profile_default);

    /*
   * Init timer_callback
   * TODO : change timer_timeout
   * 50ms : 20Hz
   * 20ms : 50Hz
   * 10ms : 100Hz
   */
  const unsigned int timer_timeout = 50;
  rclc_timer_init_default(&timer,&support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  /*
   * Init Executor
   * TODO : make sure the num_handles is correct
   * num_handles = total_of_subscriber + timer
   * publisher is not counted
   * 
   * TODO : make sure the name of sub msg and callback are correct
   */
  unsigned int num_handles = 4;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_add_subscription(&executor, &sub_status, &status_msg, &status_callback, ON_NEW_DATA);
  // rclc_executor_add_subscription(&executor, &sub_sensor, &sensor_msg, &status_callback, ON_NEW_DATA);
  // rclc_executor_add_subscription(&executor, &sub_object_difference, &object_difference_msg, &status_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // Clean up all the created objects
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_init_options_fini(&init_options);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  // Destroy publishers
  rcl_publisher_fini(&pub_status, &node);

  // Destroy subscribers
  rcl_subscription_fini(&sub_status, &node);
  // rcl_subscription_fini(&sub_sensor, &node);
  // rcl_subscription_fini(&sub_object_difference, &node);
}


void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Wire.begin();
  set_microros_serial_transports(Serial);


  // Initialize messages
  std_msgs__msg__String__init(&status_msg);
  // auv_interfaces__msg__Sensor__init(&sensor_msg);
  // auv_interfaces__msg__ObjectDifference__init(&object_difference_msg);

  status_msg.data.data = (char*) malloc(50);
  status_msg.data.capacity = 50;
  status_msg.data.size = 0;

  // if (!ina219.begin()) {
  //   Serial.println("INA219 ERROR");
  //   display.println("INA219 ERROR");
  //   // while (1);
  // }

  // ina219.setCalibration_32V_1A();
  
  // Initialize state
  state = WAITING_AGENT;
  Serial.println("Setup completed");
}

void run_control_loop() {
  static unsigned long last_display = 0;
  if (millis() - last_display > 100) {
    if (Status != prevStatus) {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.print("                ");
      display.setCursor(0, 0);
      display.print("S: "); display.print(Status);
      prevStatus = Status;
      display.display();
      last_display = millis();
    }
  }


  // unsigned long now = millis();

  // if (now - lastSensorUpdate >= sensorUpdateInterval) {
  //   lastSensorUpdate = now;

  //   // Ambil data dari INA219
  //   float shuntVoltage = ina219.getShuntVoltage_mV();
  //   busVoltage = ina219.getBusVoltage_V();
  //   float current_mA = ina219.getCurrent_mA();
  //   float power_mW = ina219.getPower_mW();
  //   float loadVoltage = busVoltage + (shuntVoltage / 1000);

  //   // Tampilkan hanya Bus Voltage, Suhu, dan Yaw di display
  //   display.setCursor(0, 0);
  //   display.setTextSize(1);
  //   display.setTextColor(SSD1306_WHITE);
  //   display.print("V: "); display.print(busVoltage, 2); display.println("V");
  //   display.print("Y: "); display.println(yaw, 2);
  //   display.display();
  // }

  // if (Status != prevStatus) {
  //   display.setTextSize(1);
  //   display.setCursor(0, 0);
  //   display.print("                ");
  //   display.setCursor(0, 0);
  //   display.print("S: "); display.print(Status);
  //   prevStatus = Status;
  //   display.display();
  // }

  // if (Object_type != prevObjectDiff) {
  //   display.setTextSize(1);
  //   display.setCursor(0, 32);
  //   display.print("                ");
  //   display.setCursor(0, 32);

  //   if(Object_type == "Orange_Flare") {
  //     display.print("O: "); display.println("OF");
  //   } else if(Object_type == "Red_Flare") {
  //     display.print("O: "); display.println("RF");
  //   } else if(Object_type == "Blue_Flare") {
  //     display.print("O: "); display.println("BF");
  //   } else if(Object_type == "Yellow_Flare") {
  //     display.print("O: "); display.println("YF");
  //   } else {
  //     display.print("O: "); display.println(Object_type);
  //   }

  //   prevObjectDiff = Object_type;
  //   display.display();
  // }
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
        Serial.println("Executor running...");
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        run_control_loop();
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  // if(state == AGENT_CONNECTED) {
  //   run_control_loop();
  // }
}