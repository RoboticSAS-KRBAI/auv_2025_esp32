// /*
//  * This is a simple template to use microros with ESP32-Arduino
//  * This sketch has sample of publisher to publish from timer_callback
//  * and subscrption to listen of new data from other ROS node.
//  * 
//  * Some of the codes below are gathered from github and forums
//  * 
//  * Made by Rasheed Kittinanthapanya
//  * 
// */


// /*
//  * TODO : Include your necessary header here
// */


// /////////////////////
// /// For Micro ROS ///
// /////////////////////
// #include <micro_ros_arduino.h>
// #include <stdio.h>
// #include <rcl/rcl.h>
// #include <rcl/error_handling.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <rmw_microros/rmw_microros.h>
// /*
//  * TODO : include your desired msg header file
// */
// #include <std_msgs/msg/bool.h>
// #include <Wire.h>
// #include <INA226.h>
// #include <auv_interfaces/msg/sensor.h>

// /*
//  * Optional,
//  * LED pin to check connection
//  * between micro-ros-agent and ESP32
// */
// // #define LED_PIN 23
// // #define LED_PIN_TEST 18 // subscription LED
// // bool led_test_state = false;

// // Pin Configuration
// #define SDA_PIN 18
// #define SCL_PIN 17

// // WSEN-PADS Configuration
// #define PADS_ADDRESS 0x5D
// #define PADS_DEVICE_ID_REG 0x0F
// #define PADS_CTRL_1_REG 0x10
// #define PADS_CTRL_2_REG 0x11
// #define PADS_STATUS_REG 0x27
// #define PADS_DATA_P_XL_REG 0x28
// #define PADS_DATA_T_L_REG 0x2B
// #define PADS_DEVICE_ID_VALUE 0xB3

// // WSEN-HIDS Configuration
// #define HIDS_ADDRESS 0x44
// #define HIDS_MEASURE_HPM 0xFD
// #define HIDS_SOFT_RESET 0x94
// #define HIDS_MEASURE_SERIAL_NUMBER 0x89
// #define HIDS_MEASURE_DELAY_HPM 20  // Increased from 15 to 20ms for reliability
// #define CRC8_POLYNOMIAL 0x31
// #define CRC8_INIT 0xFF

// // INA226 Configuration
// #define INA1_ADDR 0x45
// #define INA2_ADDR 0x41

// // Display Configuration
// #define DISPLAY_WITH_LABELS true
// #define UPDATE_INTERVAL 2000
// #define I2C_TIMEOUT 100
// #define SERIAL_BUFFER_SIZE 512

// INA226 ina1(INA1_ADDR, &Wire);
// INA226 ina2(INA2_ADDR, &Wire);

// // Timing variables for HIDS
// unsigned long hidsStartTime = 0;
// bool hidsWaiting = false;
// bool hidsDataValid = false;

// unsigned long lastDisplayTime = 0;

// // Sensor data structure with mutex protection
// struct SensorData {
//   float pressure;
//   float tempHIDS;
//   float tempPADS;
//   float tempFusion;
//   float humidity;
//   float voltage1;
//   float current1;
//   float power1;
//   float voltage2;
//   float current2;
//   float power2;
//   unsigned long timestamp;
//   bool dataReady;
//   bool hidsValid;  // Flag untuk validasi HIDS
// } sensorData;

// // Mutex for thread-safe data access
// SemaphoreHandle_t dataMutex;
// TaskHandle_t sensorTaskHandle = NULL;

// // Function Prototypes
// uint8_t PADS_readRegister(uint8_t reg);
// void PADS_writeRegister(uint8_t reg, uint8_t value);
// int32_t PADS_readPressure();
// int16_t PADS_readTemperature();
// bool PADS_init();

// uint8_t calculateCRC8(uint8_t *data, uint8_t len);
// bool verifyCRC8(uint8_t *data, uint8_t len, uint8_t crc);
// bool HIDS_sendCommand(uint8_t command);
// bool HIDS_readData(uint8_t *data, uint8_t len);
// bool HIDS_init();
// bool HIDS_softReset();
// bool HIDS_readSerialNumber(uint32_t *serialNumber);
// bool HIDS_measureBlocking(float *temperature, float *humidity);  // Changed to blocking

// // Task functions
// void sensorReadTask(void *parameter);
// void displaySensorData();


// /*
//  * Helper functions to help reconnect
// */
// #define EXECUTE_EVERY_N_MS(MS, X)  do { \
//     static volatile int64_t init = -1; \
//     if (init == -1) { init = uxr_millis();} \
//     if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
//   } while (0)

// enum states {
//   WAITING_AGENT,
//   AGENT_AVAILABLE,
//   AGENT_CONNECTED,
//   AGENT_DISCONNECTED
// } state;

// /*
//  * Declare rcl object
// */
// rclc_support_t support;
// rcl_init_options_t init_options;
// rcl_node_t node;
// rcl_timer_t timer;
// rclc_executor_t executor;
// rcl_allocator_t allocator;

// /*
//  * TODO : Declare your 
//  * publisher & subscription objects below
// */
// rcl_publisher_t pub_sensor;
// rcl_subscription_t sub_dropper;
// /*
//  * TODO : Define your necessary Msg
//  * that you want to work with below.
// */
// std_msgs__msg__Bool drop_msg;
// auv_interfaces__msg__Sensor sensor_msg;


// /*
//  * TODO : Define your subscription callbacks here
//  * leave the last one as timer_callback()
// */
// void dropper_callback(const void *msgin) {

//   std_msgs__msg__Bool * drop_msg = (std_msgs__msg__Bool *)msgin;
//   /*
//    * Do something with your receive message
//    */


// }


// void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
// {
//   (void) last_call_time;
//   if (timer != NULL) {

//     /*
//        TODO : Publish anything inside here
       
//        For example, we are going to echo back
//        the int16array_sub data to int16array_pub data,
//        so we could see the data reflect each other.

//        And also keep incrementing the int16_pub
//     */

//    rcl_publish(&pub_sensor, &sensor_msg, NULL);

//   }
// }

// /*
//    Create object (Initialization)
// */
// bool create_entities()
// {
//   /*
//      TODO : Define your
//      - ROS node name
//      - namespace
//      - ROS_DOMAIN_ID
//   */
//   const char * node_name = "esp32_node";
//   const char * ns = "";
//   const int domain_id = 0;
  
//   /*
//    * Initialize node
//    */
//   allocator = rcl_get_default_allocator();
//   init_options = rcl_get_zero_initialized_init_options();
//   rcl_init_options_init(&init_options, allocator);
//   rcl_init_options_set_domain_id(&init_options, domain_id);
//   rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
//   rclc_node_init_default(&node, node_name, ns, &support);

  
//   /*
//    * TODO : Init your publisher and subscriber 
//    */
//   rclc_publisher_init(
//     &int16array_pub,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
//     "/esp/int16array_pub", &rmw_qos_profile_default);

//   rclc_publisher_init(
//     &int16_pub,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
//     "/esp/int_pub", &rmw_qos_profile_default);

//   rclc_subscription_init(
//     &int16array_sub,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
//     "/esp/int16array_sub", &rmw_qos_profile_default);

//   rclc_subscription_init(
//     &led_sub,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
//     "/esp/led", &rmw_qos_profile_default);

//   /*
//    * Init timer_callback
//    * TODO : change timer_timeout
//    * 50ms : 20Hz
//    * 20ms : 50Hz
//    * 10ms : 100Hz
//    */
//   const unsigned int timer_timeout = 50;
//   rclc_timer_init_default(&timer,&support, RCL_MS_TO_NS(timer_timeout), timer_callback);

//   /*
//    * Init Executor
//    * TODO : make sure the num_handles is correct
//    * num_handles = total_of_subscriber + timer
//    * publisher is not counted
//    * 
//    * TODO : make sure the name of sub msg and callback are correct
//    */
//   unsigned int num_handles = 3;
//   executor = rclc_executor_get_zero_initialized_executor();
//   rclc_executor_init(&executor, &support.context, num_handles, &allocator);
//   rclc_executor_add_subscription(&executor, &int16array_sub, &int16array_recv_msg, &int16array_callback, ON_NEW_DATA);
//   rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA);
//   rclc_executor_add_timer(&executor, &timer);

//   return true;
// }
// /*
//  * Clean up all the created objects
//  */
// void destroy_entities()
// {
//   rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
//   (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

//   rcl_timer_fini(&timer);
//   rclc_executor_fini(&executor);
//   rcl_init_options_fini(&init_options);
//   rcl_node_fini(&node);
//   rclc_support_fini(&support);
//   /*
//    * TODO : Make sue the name of publisher and subscriber are correct
//    */
//   rcl_publisher_fini(&int16array_pub, &node);
//   rcl_publisher_fini(&int16_pub, &node);
//   rcl_subscription_fini(&int16array_sub, &node);
//   rcl_subscription_fini(&led_sub, &node);
  
// }

// void setup() {
//   /*
//    * TODO : select either of USB or WiFi 
//    * comment the one that not use
//    */
//   set_microros_transports();
//   //set_microros_wifi_transports("WIFI-SSID", "WIFI-PW", "HOST_IP", 8888);

//   /*
//    * Optional, setup output pin for LEDs
//    */
//   pinMode(LED_PIN, OUTPUT);
//   pinMode(LED_PIN_TEST, OUTPUT);

//   /*
//    * TODO : Initialze the message data variable
//    */
//   int16_t array_data[2] = {0, 0};
//   int16array_send_msg.data.capacity = 2; // number of array length
//   int16array_send_msg.data.size = 2;     // number of array length
//   int16array_send_msg.data.data = array_data;

//   int16array_recv_msg.data.capacity = 2; // number of array length
//   int16array_recv_msg.data.size = 2;     // number of array length
//   int16array_recv_msg.data.data = array_data;

//   int16_msg.data = 0;

//   led_msg.data = false;

//   /*
//    * Setup first state
//    */
//   state = WAITING_AGENT;

// }

// void loop() {
//   /*
//    * Try ping the micro-ros-agent (HOST PC), then switch the state 
//    * from the example
//    * https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
//    * 
//    */
//   switch (state) {
//     case WAITING_AGENT:
//       EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
//       break;
//     case AGENT_AVAILABLE:
//       state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
//       if (state == WAITING_AGENT) {
//         destroy_entities();
//       };
//       break;
//     case AGENT_CONNECTED:
//       EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
//       if (state == AGENT_CONNECTED) {
//         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//       }
//       break;
//     case AGENT_DISCONNECTED:
//       destroy_entities();
//       state = WAITING_AGENT;
//       break;
//     default:
//       break;
//   }
//   /*
//    * Output LED when in AGENT_CONNECTED state
//    */
//   if (state == AGENT_CONNECTED) {
//     digitalWrite(LED_PIN, 1);
//   } else {
//     digitalWrite(LED_PIN, 0);
//   }

//   /*
//    * TODO : 
//    * Do anything else you want to do here,
//    * like read sensor data,  
//    * calculate something, etc.
//    */

// }