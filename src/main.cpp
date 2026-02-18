#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <robot_msgs/msg/hand_message.h>

#include <can/core.hpp>

TaskHandle_t MicroROSTaskHandle = NULL;
TaskHandle_t ControlTaskHandle = NULL;
SemaphoreHandle_t DataMutex = NULL;

// 開発時に WiFi 接続する用
#if (MICRO_ROS_TRANSPORT_ARDUINO_WIFI == 1)
char ssid[] = "DRC";
char psk[] = "28228455";
IPAddress agent_ip(192, 168, 0, 101);
size_t agent_port = 8888;
#endif

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

can::CanCommunicator* can_comm;

// micro-ROS オブジェクト
rcl_publisher_t pub_feedback;
rcl_subscription_t sub_control;
rcl_timer_t timer_feedback;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// メッセージバッファ
robot_msgs__msg__HandMessage feedback_msg;
robot_msgs__msg__HandMessage control_msg;

void error_loop() {
  while (true) {
    delay(100);
  }
}

// hand_control topic の callback
IRAM_ATTR void on_control_command(const void* msg_in) {
  const auto* cmd = static_cast<const robot_msgs__msg__HandMessage*>(msg_in);

  // TODO: cmd をもとに CAN 通信する
}

// 100 ms ごとにフィードバックを送信する
IRAM_ATTR void timer_feedback_callback(rcl_timer_t* timer,
                                       int64_t /*last_call_time*/) {
  if (timer == nullptr) return;

  // TODO: CAN で取得した値を入れる

  RCSOFTCHECK(rcl_publish(&pub_feedback, &feedback_msg, nullptr));
}

void setup_micro_ros() {
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL == 1)
  set_microros_serial_transports(Serial);
#elif (MICRO_ROS_TRANSPORT_ARDUINO_WIFI == 1)
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
#endif

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));
  RCCHECK(rclc_node_init_default(&node, "hwmc_node", "", &support));

  // Publisher: hand_feedback (HandMessage 型)
  RCCHECK(rclc_publisher_init_default(
      &pub_feedback, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(robot_msgs, msg, HandMessage),
      "hand_feedback"));

  // Subscriber: hand_control (HandMessage 型)
  RCCHECK(rclc_subscription_init_default(
      &sub_control, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(robot_msgs, msg, HandMessage),
      "hand_control"));

  // Timer: 50 ms ごとにフィードバック送信
  RCCHECK(rclc_timer_init_default(&timer_feedback, &support, RCL_MS_TO_NS(50),
                                  timer_feedback_callback));

  // Message の初期化
  robot_msgs__msg__HandMessage__init(&control_msg);
  robot_msgs__msg__HandMessage__init(&feedback_msg);

  // Executor: Sub 1 + Timer 1
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_control, &control_msg,
                                         &on_control_command, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_feedback));
}

// Control Task: Core 1, 最高優先度
void ControlTask(void* pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(3);
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    // CAN 受信処理（キューを 1 つずつ処理）
    can_comm->process_received_messages();

    // TODO: いろいろやる

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Micro-ROS Task: Core 0, 標準優先度
void MicroROSTask(void* pvParameters) {
  setup_micro_ros();

  while (1) {
    // タイムアウト 10ms でタスク切り替えの余地を与える
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    vTaskDelay(10);
  }
}

void setup() {
  Serial.begin(115200);

  DataMutex = xSemaphoreCreateMutex();

  // CAN 通信の初期化（フィルタなし = 全受信）
  can_comm = new can::CanCommunicator();
  can_comm->setup();
  Serial.println("CAN setup complete");

  // Core 1 (Application Core) で制御タスクを実行
  xTaskCreatePinnedToCore(ControlTask, "ControlTask", 4096, NULL,
                          configMAX_PRIORITIES - 1, &ControlTaskHandle, 1);

  // Core 0 (Protocol Core) で Micro-ROS を実行（Wi-Fi も Core 0
  // で処理されるため）
  xTaskCreatePinnedToCore(MicroROSTask, "MicroROSTask", 8192, NULL, 2,
                          &MicroROSTaskHandle, 0);

  Serial.println("Tasks started");
}

void loop() { vTaskDelay(portMAX_DELAY); }
