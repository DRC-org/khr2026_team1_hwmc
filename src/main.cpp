#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <robot_msgs/msg/robot_feedback.h>
#include <std_msgs/msg/string.h>

#include <can/core.hpp>

// WiFi / micro-ROS agent の設定
#define WIFI_SSID "your_ssid"
#define WIFI_PASS "your_password"
#define AGENT_IP IPAddress(192, 168, 1, 100)
#define AGENT_PORT 8888

#define RCCHECK(fn)         \
  {                         \
    rcl_ret_t rc = (fn);    \
    if (rc != RCL_RET_OK) { \
      error_loop();         \
    }                       \
  }
#define RCSOFTCHECK(fn)  \
  {                      \
    rcl_ret_t rc = (fn); \
    (void)rc;            \
  }

// micro-ROS オブジェクト
rcl_publisher_t pub_feedback;
rcl_subscription_t sub_control;
rcl_timer_t timer_feedback;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// メッセージバッファ
robot_msgs__msg__RobotFeedback feedback_msg;
std_msgs__msg__String control_msg;
static char control_buf[512];

void error_loop() {
  while (true) {
    delay(100);
  }
}

// robot_control トピックのコールバック（JSON 文字列を受信）
void on_control_command(const void* msg_in) {
  const auto* cmd = static_cast<const std_msgs__msg__String*>(msg_in);

  JsonDocument doc;
  DeserializationError err =
      deserializeJson(doc, cmd->data.data, cmd->data.size);
  if (err) return;

  if (doc["m3508_rpms"].is<JsonObject>()) {
    float fl = doc["m3508_rpms"]["fl"] | 0.0f;
    float fr = doc["m3508_rpms"]["fr"] | 0.0f;
    float rl = doc["m3508_rpms"]["rl"] | 0.0f;
    float rr = doc["m3508_rpms"]["rr"] | 0.0f;
    // TODO: CAN バスで各 M3508 に目標 RPM を送信
    (void)fl;
    (void)fr;
    (void)rl;
    (void)rr;
  }

  if (doc["pid_gains"].is<JsonObject>()) {
    float kp = doc["pid_gains"]["kp"] | 0.5f;
    float ki = doc["pid_gains"]["ki"] | 0.05f;
    float kd = doc["pid_gains"]["kd"] | 0.0f;
    // TODO: PID ゲインを更新
    (void)kp;
    (void)ki;
    (void)kd;
  }
}

// 100 ms ごとにフィードバックを送信するタイマーコールバック
void timer_feedback_callback(rcl_timer_t* timer, int64_t /*last_call_time*/) {
  if (timer == nullptr) return;

  // TODO: CAN バスから実際の RPM を読み取り feedback_msg に格納する
  // 例:
  //   feedback_msg.m3508_rpms.fl = can_get_rpm(CAN_M3508_FL);
  //   feedback_msg.m3508_rpms.fr = can_get_rpm(CAN_M3508_FR);
  //   feedback_msg.m3508_rpms.rl = can_get_rpm(CAN_M3508_RL);
  //   feedback_msg.m3508_rpms.rr = can_get_rpm(CAN_M3508_RR);

  // TODO: 各機構の状態を取得して格納する
  // 例:
  //   feedback_msg.yagura_1.pos   = ROBOT_MSGS__MSG__YAGURA_MECHANISM__POS_UP;
  //   feedback_msg.yagura_1.state =
  //   ROBOT_MSGS__MSG__YAGURA_MECHANISM__STATE_OPEN;

  RCSOFTCHECK(rcl_publish(&pub_feedback, &feedback_msg, nullptr));
}

void setup() {
  Serial.begin(115200);

  // micro-ROS WiFi トランスポートの初期化（内部で WiFi 接続も行う）
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));
  RCCHECK(rclc_node_init_default(&node, "hwmc_node", "", &support));

  // Publisher: robot_feedback (RobotFeedback 型)
  RCCHECK(rclc_publisher_init_default(
      &pub_feedback, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(robot_msgs, msg, RobotFeedback),
      "robot_feedback"));

  // Subscriber: robot_control (String 型, JSON)
  control_msg.data.data = control_buf;
  control_msg.data.size = 0;
  control_msg.data.capacity = sizeof(control_buf);
  RCCHECK(rclc_subscription_init_default(
      &sub_control, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "robot_control"));

  // タイマー: 100 ms ごとにフィードバック送信
  RCCHECK(rclc_timer_init_default(&timer_feedback, &support, RCL_MS_TO_NS(100),
                                  timer_feedback_callback));

  // エグゼキュータ: サブスクライバ 1 + タイマー 1
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_control, &control_msg,
                                         &on_control_command, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_feedback));
}

void loop() { rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); }
