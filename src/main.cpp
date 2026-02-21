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
#include <can/peripheral.hpp>

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

#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      (void)temp_rc;               \
    }                              \
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
  uint8_t
      pos;  // yagura: UP=0,DOWN=1,STOPPED=2,UP_DONE=3,DOWN_DONE=4
            // ring:
            // PICKUP=0,YAGURA=1,HONMARU=2,STOPPED=3,PICKUP_DONE=4,YAGURA_DONE=5,HONMARU_DONE=6
  uint8_t state;  // OPEN=0,CLOSE=1,STOPPED=2,OPEN_DONE=3,CLOSE_DONE=4
};

struct MechanismStates {
  ActuatorState yagura_1 = {2, 4};
  ActuatorState yagura_2 = {2, 4};
  ActuatorState ring_1 = {3, 2};
  ActuatorState ring_2 = {3, 2};
};

MechanismStates target{};   // 目標値
MechanismStates current{};  // 現在値

enum class AgentState { WAITING, CONNECTED, DISCONNECTED };

void register_can_event_handlers() {
  // M3508 のフィードバック RPM を受け取る
  can_comm->add_receive_event_listener(
      {0x000}, [&](const can::CanId /*id*/, const std::array<uint8_t, 8> data) {
        int8_t identifier = data[0];
        int8_t target = data[1];

        // volatile int16_t への書き込みは ESP32 でアトミック。
        // Mutex を使うと Core 0 の micro-ROS タスクとの競合で
        // フィードバックがドロップされるため、ここでは使わない。
        switch (identifier) {
          case 0x30:
            switch (target) {
              case 0x00:
                current.yagura_1.pos =
                    robot_msgs__msg__YaguraMechanism__POS_DOWN_DONE;
                break;
              case 0x01:
                current.yagura_1.pos =
                    robot_msgs__msg__YaguraMechanism__POS_UP_DONE;
                break;
            }
            break;
          case 0x40:
            switch (target) {
              case 0x00:
                current.yagura_2.state =
                    robot_msgs__msg__YaguraMechanism__STATE_CLOSE_DONE;
                break;
              case 0x01:
                current.yagura_2.state =
                    robot_msgs__msg__YaguraMechanism__STATE_OPEN_DONE;
                break;
            }
            break;
        }
      });
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

  // TODO: CanTxMessageBuilder の仕様が今年と異なるので修正する

  for (int i = 0; i < 2; i++) {
    const ActuatorState& tgt = (i == 0) ? target.yagura_1 : target.yagura_2;
    const ActuatorState& cur = (i == 0) ? current.yagura_1 : current.yagura_2;

    if (tgt.pos == robot_msgs__msg__YaguraMechanism__POS_UP &&
        cur.pos != robot_msgs__msg__YaguraMechanism__POS_UP &&
        cur.pos != robot_msgs__msg__YaguraMechanism__POS_UP_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(can::CanDest::dc_lift)
                             .set_command((i == 0) ? 0x01 : 0x11)
                             .build());
      if (i == 0) {
        current.yagura_1.pos = robot_msgs__msg__YaguraMechanism__POS_UP;
      }
    } else if (tgt.pos == robot_msgs__msg__YaguraMechanism__POS_DOWN &&
               cur.pos != robot_msgs__msg__YaguraMechanism__POS_DOWN &&
               cur.pos != robot_msgs__msg__YaguraMechanism__POS_DOWN_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(can::CanDest::dc_lift)
                             .set_command((i == 0) ? 0x00 : 0x10)
                             .build());
      if (i == 0) {
        current.yagura_1.pos = robot_msgs__msg__YaguraMechanism__POS_DOWN;
      }
    } else if (tgt.pos == robot_msgs__msg__YaguraMechanism__POS_STOPPED &&
               cur.pos != robot_msgs__msg__YaguraMechanism__POS_STOPPED) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(can::CanDest::dc_lift)
                             .set_command((i == 0) ? 0x02 : 0x12)
                             .build());
      if (i == 0) {
        current.yagura_1.pos = robot_msgs__msg__YaguraMechanism__POS_STOPPED;
      }
    }

    if (tgt.state == robot_msgs__msg__YaguraMechanism__STATE_OPEN &&
        cur.state != robot_msgs__msg__YaguraMechanism__STATE_OPEN &&
        cur.state != robot_msgs__msg__YaguraMechanism__STATE_OPEN_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(can::CanDest::servo_yagura)
                             .set_command((i == 0) ? 0x01 : 0x11)
                             .build());
      if (i == 0) {
        Serial.println("OPEN");
        current.yagura_1.state = robot_msgs__msg__YaguraMechanism__STATE_OPEN;
      }
    } else if (tgt.state == robot_msgs__msg__YaguraMechanism__STATE_CLOSE &&
               cur.state != robot_msgs__msg__YaguraMechanism__STATE_CLOSE &&
               cur.state !=
                   robot_msgs__msg__YaguraMechanism__STATE_CLOSE_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(can::CanDest::servo_yagura)
                             .set_command((i == 0) ? 0x00 : 0x10)
                             .build());
      if (i == 0) {
        Serial.println("CLOSE");
        current.yagura_1.state = robot_msgs__msg__YaguraMechanism__STATE_CLOSE;
      }
    } else if (tgt.state == robot_msgs__msg__YaguraMechanism__STATE_STOPPED &&
               cur.state != robot_msgs__msg__YaguraMechanism__STATE_STOPPED) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(can::CanDest::servo_yagura)
                             .set_command((i == 0) ? 0x02 : 0x12)
                             .build());
      if (i == 0) {
        Serial.println("STOPPED");
        current.yagura_1.state =
            robot_msgs__msg__YaguraMechanism__STATE_STOPPED;
      }
    }
  }

  for (int i = 0; i < 2; i++) {
    const ActuatorState& tgt = (i == 0) ? target.ring_1 : target.ring_2;
    const ActuatorState& cur = (i == 0) ? current.ring_1 : current.ring_2;

    if (tgt.pos == robot_msgs__msg__RingMechanism__POS_PICKUP &&
        cur.pos != robot_msgs__msg__RingMechanism__POS_PICKUP &&
        cur.pos != robot_msgs__msg__RingMechanism__POS_PICKUP_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(can::CanDest::servo_ring)
                             .set_command((i == 0) ? 0x00 : 0x20)
                             .set_value(0x00)
                             .build());
    } else if (tgt.pos == robot_msgs__msg__RingMechanism__POS_YAGURA &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_YAGURA &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_YAGURA_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(can::CanDest::servo_ring)
                             .set_command((i == 0) ? 0x00 : 0x20)
                             .set_value(0x01)
                             .build());
    } else if (tgt.pos == robot_msgs__msg__RingMechanism__POS_HONMARU &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_HONMARU &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_HONMARU_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(can::CanDest::servo_ring)
                             .set_command((i == 0) ? 0x00 : 0x20)
                             .set_value(0x02)
                             .build());
    } else if (tgt.pos == robot_msgs__msg__RingMechanism__POS_STOPPED &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_STOPPED) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(can::CanDest::servo_ring)
                             .set_command((i == 0) ? 0x01 : 0x21)
                             .build());
    }

    if (tgt.state == robot_msgs__msg__RingMechanism__STATE_OPEN &&
        cur.state != robot_msgs__msg__RingMechanism__STATE_OPEN &&
        cur.state != robot_msgs__msg__RingMechanism__STATE_OPEN_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(can::CanDest::servo_ring)
                             .set_command((i == 0) ? 0x11 : 0x31)
                             .build());
    } else if (tgt.state == robot_msgs__msg__RingMechanism__STATE_CLOSE &&
               cur.state != robot_msgs__msg__RingMechanism__STATE_CLOSE &&
               cur.state != robot_msgs__msg__RingMechanism__STATE_CLOSE_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(can::CanDest::servo_ring)
                             .set_command((i == 0) ? 0x10 : 0x30)
                             .build());
    } else if (tgt.state == robot_msgs__msg__RingMechanism__STATE_STOPPED &&
               cur.state != robot_msgs__msg__RingMechanism__STATE_STOPPED) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(can::CanDest::servo_ring)
                             .set_command((i == 0) ? 0x12 : 0x32)
                             .build());
    }
  }
}

// 50 ms ごとにフィードバックを送信する
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

/* void setup_micro_ros() {
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
} */

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK)
    return false;
  if (rclc_node_init_default(&node, "hwmc_node", "", &support) != RCL_RET_OK)
    return false;

  if (rclc_publisher_init_default(
          &pub_feedback, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(robot_msgs, msg, HandMessage),
          "hand_feedback") != RCL_RET_OK)
    return false;

  if (rclc_subscription_init_default(
          &sub_control, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(robot_msgs, msg, HandMessage),
          "hand_control") != RCL_RET_OK)
    return false;

  if (rclc_timer_init_default(&timer_feedback, &support, RCL_MS_TO_NS(50),
                              timer_feedback_callback) != RCL_RET_OK)
    return false;

  robot_msgs__msg__HandMessage__init(&control_msg);
  robot_msgs__msg__HandMessage__init(&feedback_msg);

  if (rclc_executor_init(&executor, &support.context, 2, &allocator) !=
      RCL_RET_OK)
    return false;
  if (rclc_executor_add_subscription(&executor, &sub_control, &control_msg,
                                     &on_control_command,
                                     ON_NEW_DATA) != RCL_RET_OK)
    return false;
  if (rclc_executor_add_timer(&executor, &timer_feedback) != RCL_RET_OK)
    return false;

  return true;
}

static void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&executor);
  rcl_timer_fini(&timer_feedback);
  rcl_subscription_fini(&sub_control, &node);
  rcl_publisher_fini(&pub_feedback, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  robot_msgs__msg__HandMessage__fini(&control_msg);
  robot_msgs__msg__HandMessage__fini(&feedback_msg);
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
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL == 1)
  set_microros_serial_transports(Serial);
#elif (MICRO_ROS_TRANSPORT_ARDUINO_WIFI == 1)
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
#endif

  AgentState agent_state = AgentState::WAITING;
  uint32_t ping_counter = 0;

  while (1) {
    switch (agent_state) {
      case AgentState::WAITING:
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          if (create_entities()) {
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL != 1)
            Serial.println("micro-ROS: connected");
#endif
            ping_counter = 0;
            agent_state = AgentState::CONNECTED;
          } else {
            destroy_entities();
          }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        break;

      case AgentState::CONNECTED:
        // 約 1 秒ごとに agent の死活確認
        if (++ping_counter >= 100) {
          ping_counter = 0;
          if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL != 1)
            Serial.println("micro-ROS: agent disconnected");
#endif
            agent_state = AgentState::DISCONNECTED;
            break;
          }
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
        break;

      case AgentState::DISCONNECTED:
        // TODO: 切断時にモーターを安全停止
        // if (xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
        //   target.fl = 0;
        //   target.fr = 0;
        //   target.rl = 0;
        //   target.rr = 0;
        //   xSemaphoreGive(DataMutex);
        // }
        destroy_entities();
        agent_state = AgentState::WAITING;
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);

  DataMutex = xSemaphoreCreateMutex();

  // CAN 通信の初期化（フィルタなし = 全受信）
  can_comm = new can::CanCommunicator();
  can_comm->setup();
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL != 1)
  Serial.println("CAN setup complete");
#endif

  // Core 1 (Application Core) で制御タスクを実行
  xTaskCreatePinnedToCore(ControlTask, "ControlTask", 4096, NULL,
                          configMAX_PRIORITIES - 1, &ControlTaskHandle, 1);

  // Core 0 (Protocol Core) で Micro-ROS を実行（Wi-Fi も Core 0
  // で処理されるため）
  xTaskCreatePinnedToCore(MicroROSTask, "MicroROSTask", 8192, NULL, 2,
                          &MicroROSTaskHandle, 0);

#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL != 1)
  Serial.println("Tasks started");
#endif
}

void loop() { vTaskDelay(portMAX_DELAY); }
