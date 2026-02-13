#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <ArduinoJson.h>

#include <can/core.hpp>

// Hardware Pins
#define HOME_SW_1_PIN 34
#define HOME_SW_2_PIN 35

// State
bool is_homed_1 = false;
bool is_homed_2 = false;
bool is_homing_1_in_progress = false;
bool is_homing_2_in_progress = false;

// CAN
can::CanCommunicator* can_comm;

// ROS 2
rcl_subscription_t robot_control_sub;
rcl_publisher_t robot_status_pub;
std_msgs__msg__String robot_control_msg;
std_msgs__msg__String robot_status_msg;
char rx_json_buffer[1024];
char tx_json_buffer[512];

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop() { while(1) delay(100); }

// ホーミング開始
void start_homing() {
  is_homing_1_in_progress = true;
  is_homing_2_in_progress = true;
  can::CanTxMessageBuilder builder;
  // Move both lifts down slowly (0x00 for Lift 1, 0x10 for Lift 2)
  can_comm->transmit(builder.set_id(can::CanId(0x300)).set_command(0x00).build());
  can_comm->transmit(builder.set_id(can::CanId(0x300)).set_command(0x10).build());
}

// CAN -> ROS フィードバック処理
void register_can_listeners() {
  can_comm->add_receive_event_listener(
    {0x10, 0x30, 0x31, 0x40, 0x41, 0x4A, 0x4B, 0x4C, 0x4D},
    [&](const can::CanId id, const std::array<uint8_t, 8> data) {
      StaticJsonDocument<256> status_doc;
      status_doc["id"] = (uint32_t)id;
      if (id == 0x10) {
        float voltage = (float)((data[1] << 8) | data[2]) / 100.0f;
        status_doc["battery_12v"] = voltage;
      } else {
        status_doc["val"] = data[1];
      }
      serializeJson(status_doc, tx_json_buffer);
      robot_status_msg.data.data = tx_json_buffer;
      robot_status_msg.data.size = strlen(tx_json_buffer);
      rcl_publish(&robot_status_pub, &robot_status_msg, NULL);
    }
  );
}

// ROS -> CAN 制御コマンド処理
void robot_control_callback(const void* msgin) {
  if (!is_homed_1 || !is_homed_2) return; // Command gating until homed

  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
  StaticJsonDocument<1024> doc;
  if (deserializeJson(doc, msg->data.data)) return;

  can::CanTxMessageBuilder builder;

  if (doc.containsKey("yagura")) {
    auto y = doc["yagura"];
    for (int i = 1; i <= 2; i++) {
      char p_key[8], s_key[8];
      sprintf(p_key, "%d_pos", i); sprintf(s_key, "%d_state", i);
      if (y.containsKey(p_key)) {
        const char* p = y[p_key]; uint8_t cmd = 0x02;
        if (strcmp(p, "up") == 0) cmd = 0x01; else if (strcmp(p, "down") == 0) cmd = 0x00;
        can_comm->transmit(builder.set_id(can::CanId(0x300)).set_command(i == 1 ? cmd : cmd + 0x10).build());
      }
      if (y.containsKey(s_key)) {
        const char* s = y[s_key]; uint8_t cmd = 0x02;
        if (strcmp(s, "open") == 0) cmd = 0x01; else if (strcmp(s, "closed") == 0) cmd = 0x00;
        can_comm->transmit(builder.set_id(can::CanId(0x400)).set_command(i == 1 ? cmd : cmd + 0x10).build());
      }
    }
  }

  if (doc.containsKey("ring")) {
    auto r = doc["ring"];
    for (int i = 1; i <= 2; i++) {
      char p_key[8], s_key[8];
      sprintf(p_key, "%d_pos", i); sprintf(s_key, "%d_state", i);
      if (r.containsKey(p_key)) {
        const char* p = r[p_key]; uint8_t v = 0x00;
        if (strcmp(p, "yagura") == 0) v = 0x01; else if (strcmp(p, "honmaru") == 0) v = 0x02;
        can_comm->transmit(builder.set_id(can::CanId(0x401)).set_command(i == 1 ? 0x00 : 0x20).set_value((uint32_t)v).build());
      }
      if (r.containsKey(s_key)) {
        const char* s = r[s_key]; uint8_t cmd = 0x12;
        if (strcmp(s, "open") == 0) cmd = 0x11; else if (strcmp(s, "closed") == 0) cmd = 0x10;
        can_comm->transmit(builder.set_id(can::CanId(0x401)).set_command(i == 1 ? cmd : cmd + 0x20).build());
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(HOME_SW_1_PIN, INPUT_PULLUP);
  pinMode(HOME_SW_2_PIN, INPUT_PULLUP);

  can_comm = new can::CanCommunicator();
  can_comm->setup();

  can::CanTxMessageBuilder pwr;
  can_comm->transmit(pwr.set_id(can::CanId(0x100)).set_command(0x00).set_omake({0x01, 0x01, 0x01}).build());

  set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "hwmc_node", "", &support));

  robot_control_msg.data.data = rx_json_buffer;
  robot_control_msg.data.capacity = 1024;
  robot_status_msg.data.data = tx_json_buffer;
  robot_status_msg.data.capacity = 512;

  RCCHECK(rclc_subscription_init_default(&robot_control_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/robot_control"));
  RCCHECK(rclc_publisher_init_default(&robot_status_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/robot_status"));

  register_can_listeners();
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &robot_control_sub, &robot_control_msg, &robot_control_callback, ON_NEW_DATA));

  start_homing();
}

void loop() {
  can_comm->process_received_messages();
  
  if (is_homing_1_in_progress && digitalRead(HOME_SW_1_PIN) == LOW) {
    can::CanTxMessageBuilder b; can_comm->transmit(b.set_id(can::CanId(0x300)).set_command(0x02).build());
    is_homed_1 = true; is_homing_1_in_progress = false; Serial.println("Lift 1 Homed!");
  }
  if (is_homing_2_in_progress && digitalRead(HOME_SW_2_PIN) == LOW) {
    can::CanTxMessageBuilder b; can_comm->transmit(b.set_id(can::CanId(0x300)).set_command(0x12).build());
    is_homed_2 = true; is_homing_2_in_progress = false; Serial.println("Lift 2 Homed!");
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(1);
}
