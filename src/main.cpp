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

// 各アクチュエータの状態
struct ActuatorState {
  uint8_t pos =
      0;  // yagura: UP=0,DOWN=1,STOPPED=2,UP_DONE=3,DOWN_DONE=4
          // ring:
          // YAGURA=0,HONMARU=1,STOPPED=2,PICKUP_DONE=3,YAGURA_DONE=4,HONMARU_DONE=5
  uint8_t state = 0;  // OPEN=0,CLOSE=1,STOPPED=2,OPEN_DONE=3,CLOSE_DONE=4
};

struct MechanismStates {
  ActuatorState yagura_1;
  ActuatorState yagura_2;
  ActuatorState ring_1;
  ActuatorState ring_2;
};

MechanismStates target{};   // 目標値
MechanismStates current{};  // 現在値

void error_loop() {
  while (true) {
    delay(100);
  }
}

// hand_control topic の callback
IRAM_ATTR void on_control_command(const void* msg_in) {
  const auto* cmd = static_cast<const robot_msgs__msg__HandMessage*>(msg_in);

  if (xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
    target.yagura_1 = {cmd->yagura_1.pos, cmd->yagura_1.state};
    target.yagura_2 = {cmd->yagura_2.pos, cmd->yagura_2.state};
    target.ring_1 = {cmd->ring_1.pos, cmd->ring_1.state};
    target.ring_2 = {cmd->ring_2.pos, cmd->ring_2.state};

    xSemaphoreGive(DataMutex);
  }

  for (int i = 0; i < 2; i++) {
    const ActuatorState& tgt = (i == 0) ? target.yagura_1 : target.yagura_2;
    const ActuatorState& cur = (i == 0) ? current.yagura_1 : current.yagura_2;

    if (tgt.pos == robot_msgs__msg__YaguraMechanism__POS_UP &&
        cur.pos != robot_msgs__msg__YaguraMechanism__POS_UP &&
        cur.pos != robot_msgs__msg__YaguraMechanism__POS_UP_DONE) {
      // TODO: 櫓ハンドを UP
    } else if (tgt.pos == robot_msgs__msg__YaguraMechanism__POS_DOWN &&
               cur.pos != robot_msgs__msg__YaguraMechanism__POS_DOWN &&
               cur.pos != robot_msgs__msg__YaguraMechanism__POS_DOWN_DONE) {
      // TODO: 櫓ハンドを DOWN
    } else if (tgt.pos == robot_msgs__msg__YaguraMechanism__POS_STOPPED &&
               cur.pos != robot_msgs__msg__YaguraMechanism__POS_STOPPED) {
      // TODO: 櫓ハンドを STOP
    }

    if (tgt.state == robot_msgs__msg__YaguraMechanism__STATE_OPEN &&
        cur.state != robot_msgs__msg__YaguraMechanism__STATE_OPEN &&
        cur.state != robot_msgs__msg__YaguraMechanism__STATE_OPEN_DONE) {
      // TODO: 櫓ハンドを OPEN
    } else if (tgt.state == robot_msgs__msg__YaguraMechanism__STATE_CLOSE &&
               cur.state != robot_msgs__msg__YaguraMechanism__STATE_CLOSE &&
               cur.state !=
                   robot_msgs__msg__YaguraMechanism__STATE_CLOSE_DONE) {
      // TODO: 櫓ハンドを CLOSE
    } else if (tgt.state == robot_msgs__msg__YaguraMechanism__STATE_STOPPED &&
               cur.state != robot_msgs__msg__YaguraMechanism__STATE_STOPPED) {
      // TODO: 櫓ハンドを STOP
    }
  }

  for (int i = 0; i < 2; i++) {
    const ActuatorState& tgt = (i == 0) ? target.ring_1 : target.ring_2;
    const ActuatorState& cur = (i == 0) ? current.ring_1 : current.ring_2;

    if (tgt.pos == robot_msgs__msg__RingMechanism__POS_PICKUP &&
        cur.pos != robot_msgs__msg__RingMechanism__POS_PICKUP &&
        cur.pos != robot_msgs__msg__RingMechanism__POS_PICKUP_DONE) {
      // TODO: リングハンドを PICKUP ポジションへ
    } else if (tgt.pos == robot_msgs__msg__RingMechanism__POS_YAGURA &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_YAGURA &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_YAGURA_DONE) {
      // TODO: リングハンドを YAGURA ポジションへ
    } else if (tgt.pos == robot_msgs__msg__RingMechanism__POS_HONMARU &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_HONMARU &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_HONMARU_DONE) {
      // TODO: リングハンドを HONMARU ポジションへ
    } else if (tgt.pos == robot_msgs__msg__RingMechanism__POS_STOPPED &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_STOPPED) {
      // TODO: リングハンドを STOP
    }

    if (tgt.state == robot_msgs__msg__RingMechanism__STATE_OPEN &&
        cur.state != robot_msgs__msg__RingMechanism__STATE_OPEN &&
        cur.state != robot_msgs__msg__RingMechanism__STATE_OPEN_DONE) {
      // TODO: リングハンドを OPEN
    } else if (tgt.state == robot_msgs__msg__RingMechanism__STATE_CLOSE &&
               cur.state != robot_msgs__msg__RingMechanism__STATE_CLOSE &&
               cur.state != robot_msgs__msg__RingMechanism__STATE_CLOSE_DONE) {
      // TODO: リングハンドを CLOSE
    } else if (tgt.state == robot_msgs__msg__RingMechanism__STATE_STOPPED &&
               cur.state != robot_msgs__msg__RingMechanism__STATE_STOPPED) {
      // TODO: リングハンドを STOP
    }
  }
}

// 100 ms ごとにフィードバックを送信する
IRAM_ATTR void timer_feedback_callback(rcl_timer_t* timer,
                                       int64_t /*last_call_time*/) {
  if (timer == nullptr) return;

  if (xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
    feedback_msg.yagura_1 = {current.yagura_1.pos, current.yagura_1.state};
    feedback_msg.yagura_2 = {current.yagura_2.pos, current.yagura_2.state};
    feedback_msg.ring_1 = {current.ring_1.pos, current.ring_1.state};
    feedback_msg.ring_2 = {current.ring_2.pos, current.ring_2.state};

    xSemaphoreGive(DataMutex);
  }

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
