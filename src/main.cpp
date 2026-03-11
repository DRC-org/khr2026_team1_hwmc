#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_system.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <robot_msgs/msg/hand_message.h>

#include <std_msgs/msg/bool.h>

#include <can/core.hpp>
#include <can/peripheral.hpp>

TaskHandle_t MicroROSTaskHandle = NULL;
TaskHandle_t ControlTaskHandle = NULL;
SemaphoreHandle_t DataMutex = NULL;

// 開発時に WiFi 接続する用
#if (MICRO_ROS_TRANSPORT_ARDUINO_WIFI == 1)
char ssid[] = "DRC";
char psk[] = "kumachan";
IPAddress agent_ip(192, 168, 1, 103);
size_t agent_port = 8889;
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

// 動的ヘルスチェック
rcl_subscription_t sub_health_check;
std_msgs__msg__Bool hc_control_msg;
volatile bool hc_active = false;
volatile uint8_t hc_step = 0;
volatile uint32_t hc_time = 0;

// サーボ基板ヘルスチェック状態
struct ServoHealthState {
  volatile bool alive = false;
  volatile uint32_t last_response_ms = 0;
  volatile uint8_t ring_hand_state = 0;
  volatile uint8_t yagura_state = 0;
  volatile uint8_t touch_sensor = 0;
};

ServoHealthState servo_health[2];  // [0]=front(0x400), [1]=rear(0x401)

struct DcLiftHealthState {
  volatile bool alive = false;
  volatile uint32_t last_response_ms = 0;
};

DcLiftHealthState dc_lift_health[2];  // [0]=front(0x300), [1]=rear(0x301)

struct CrabHealthState {
  volatile bool alive = false;
  volatile uint32_t last_response_ms = 0;
};

CrabHealthState crab_health;

uint32_t last_health_request_ms = 0;
constexpr uint32_t HEALTH_REQUEST_INTERVAL_MS = 1000;
constexpr uint32_t HEALTH_TIMEOUT_MS = 2000;

// 各アクチュエータの状態
struct ActuatorState {
  uint8_t
      pos;  // yagura: UP=0,DOWN=1,STOPPED=2,UP_DONE=3,DOWN_DONE=4
            // ring:
            // PICKUP=0,YAGURA=1,HONMARU=2,STOPPED=3,PICKUP_DONE=4,YAGURA_DONE=5,HONMARU_DONE=6
  uint8_t state;  // OPEN=0,CLOSE=1,STOPPED=2,OPEN_DONE=3,CLOSE_DONE=4
};

struct CrabLedState {
  bool vgoal_led = false;
  bool error_led = false;
};

struct MechanismStates {
  ActuatorState yagura_1 = {2, 2};
  ActuatorState yagura_2 = {2, 2};
  ActuatorState ring_1 = {3, 2};
  ActuatorState ring_2 = {3, 2};
  CrabLedState crab = {};
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
          // 古いフィードバックによる再送ループ防止:
          // コマンド方向と一致する完了通知のみ受理する
          case 0x30:  // 昇降 1 動作完了
            if (target == 0x00 &&
                current.yagura_1.pos ==
                    robot_msgs__msg__YaguraMechanism__POS_DOWN) {
              current.yagura_1.pos =
                  robot_msgs__msg__YaguraMechanism__POS_DOWN_DONE;
            } else if (target == 0x01 &&
                       current.yagura_1.pos ==
                           robot_msgs__msg__YaguraMechanism__POS_UP) {
              current.yagura_1.pos =
                  robot_msgs__msg__YaguraMechanism__POS_UP_DONE;
            }
            break;
          case 0x31:  // 昇降 2 動作完了
            if (target == 0x00 &&
                current.yagura_2.pos ==
                    robot_msgs__msg__YaguraMechanism__POS_DOWN) {
              current.yagura_2.pos =
                  robot_msgs__msg__YaguraMechanism__POS_DOWN_DONE;
            } else if (target == 0x01 &&
                       current.yagura_2.pos ==
                           robot_msgs__msg__YaguraMechanism__POS_UP) {
              current.yagura_2.pos =
                  robot_msgs__msg__YaguraMechanism__POS_UP_DONE;
            }
            break;
          case 0x40:  // リングハンド 1 動作完了（開閉）
            if (target == 0x00 &&
                current.ring_1.state ==
                    robot_msgs__msg__RingMechanism__STATE_CLOSE) {
              current.ring_1.state =
                  robot_msgs__msg__RingMechanism__STATE_CLOSE_DONE;
            } else if (target == 0x01 &&
                       current.ring_1.state ==
                           robot_msgs__msg__RingMechanism__STATE_OPEN) {
              current.ring_1.state =
                  robot_msgs__msg__RingMechanism__STATE_OPEN_DONE;
            }
            break;
          case 0x41:  // リングハンド 1 移動完了（位置）
            if (target == 0x00 &&
                current.ring_1.pos ==
                    robot_msgs__msg__RingMechanism__POS_PICKUP) {
              current.ring_1.pos =
                  robot_msgs__msg__RingMechanism__POS_PICKUP_DONE;
            } else if (target == 0x01 &&
                       current.ring_1.pos ==
                           robot_msgs__msg__RingMechanism__POS_YAGURA) {
              current.ring_1.pos =
                  robot_msgs__msg__RingMechanism__POS_YAGURA_DONE;
            } else if (target == 0x02 &&
                       current.ring_1.pos ==
                           robot_msgs__msg__RingMechanism__POS_HONMARU) {
              current.ring_1.pos =
                  robot_msgs__msg__RingMechanism__POS_HONMARU_DONE;
            }
            break;
          case 0x42:  // リングハンド 1 把持失敗
            // OPEN/OPEN_DONE 中は上書きしない（OPEN コマンド送信後の遅延 GRIP_FAIL を無視）
            if (current.ring_1.state !=
                    robot_msgs__msg__RingMechanism__STATE_OPEN &&
                current.ring_1.state !=
                    robot_msgs__msg__RingMechanism__STATE_OPEN_DONE) {
              current.ring_1.state =
                  robot_msgs__msg__RingMechanism__STATE_GRIP_FAIL;
            }
            break;
          case 0x43:  // 櫓ハンド 1 動作完了
            if (target == 0x00 &&
                current.yagura_1.state ==
                    robot_msgs__msg__YaguraMechanism__STATE_CLOSE) {
              current.yagura_1.state =
                  robot_msgs__msg__YaguraMechanism__STATE_CLOSE_DONE;
            } else if (target == 0x01 &&
                       current.yagura_1.state ==
                           robot_msgs__msg__YaguraMechanism__STATE_OPEN) {
              current.yagura_1.state =
                  robot_msgs__msg__YaguraMechanism__STATE_OPEN_DONE;
            }
            break;
          case 0x4A:  // リングハンド 2 動作完了（開閉）
            if (target == 0x00 &&
                current.ring_2.state ==
                    robot_msgs__msg__RingMechanism__STATE_CLOSE) {
              current.ring_2.state =
                  robot_msgs__msg__RingMechanism__STATE_CLOSE_DONE;
            } else if (target == 0x01 &&
                       current.ring_2.state ==
                           robot_msgs__msg__RingMechanism__STATE_OPEN) {
              current.ring_2.state =
                  robot_msgs__msg__RingMechanism__STATE_OPEN_DONE;
            }
            break;
          case 0x4B:  // リングハンド 2 移動完了（位置）
            if (target == 0x00 &&
                current.ring_2.pos ==
                    robot_msgs__msg__RingMechanism__POS_PICKUP) {
              current.ring_2.pos =
                  robot_msgs__msg__RingMechanism__POS_PICKUP_DONE;
            } else if (target == 0x01 &&
                       current.ring_2.pos ==
                           robot_msgs__msg__RingMechanism__POS_YAGURA) {
              current.ring_2.pos =
                  robot_msgs__msg__RingMechanism__POS_YAGURA_DONE;
            } else if (target == 0x02 &&
                       current.ring_2.pos ==
                           robot_msgs__msg__RingMechanism__POS_HONMARU) {
              current.ring_2.pos =
                  robot_msgs__msg__RingMechanism__POS_HONMARU_DONE;
            }
            break;
          case 0x4C:  // リングハンド 2 把持失敗
            // OPEN/OPEN_DONE 中は上書きしない（OPEN コマンド送信後の遅延 GRIP_FAIL を無視）
            if (current.ring_2.state !=
                    robot_msgs__msg__RingMechanism__STATE_OPEN &&
                current.ring_2.state !=
                    robot_msgs__msg__RingMechanism__STATE_OPEN_DONE) {
              current.ring_2.state =
                  robot_msgs__msg__RingMechanism__STATE_GRIP_FAIL;
            }
            break;
          case 0x4D:  // 櫓ハンド 2 動作完了
            if (target == 0x00 &&
                current.yagura_2.state ==
                    robot_msgs__msg__YaguraMechanism__STATE_CLOSE) {
              current.yagura_2.state =
                  robot_msgs__msg__YaguraMechanism__STATE_CLOSE_DONE;
            } else if (target == 0x01 &&
                       current.yagura_2.state ==
                           robot_msgs__msg__YaguraMechanism__STATE_OPEN) {
              current.yagura_2.state =
                  robot_msgs__msg__YaguraMechanism__STATE_OPEN_DONE;
            }
            break;
          case 0x44: {  // サーボ基板 front ヘルスチェック応答
            servo_health[0].alive = true;
            servo_health[0].last_response_ms = (uint32_t)millis();
            servo_health[0].ring_hand_state = data[1];
            servo_health[0].yagura_state = data[2];
            servo_health[0].touch_sensor = data[3];
            break;
          }
          case 0x4E: {  // サーボ基板 rear ヘルスチェック応答
            servo_health[1].alive = true;
            servo_health[1].last_response_ms = (uint32_t)millis();
            servo_health[1].ring_hand_state = data[1];
            servo_health[1].yagura_state = data[2];
            servo_health[1].touch_sensor = data[3];
            break;
          }
          case 0x3A: {  // DC リフト front ヘルスチェック応答
            dc_lift_health[0].alive = true;
            dc_lift_health[0].last_response_ms = (uint32_t)millis();
            break;
          }
          case 0x3B: {  // DC リフト rear ヘルスチェック応答
            dc_lift_health[1].alive = true;
            dc_lift_health[1].last_response_ms = (uint32_t)millis();
            break;
          }
          case 0x32: {  // crub_igniter ヘルスチェック応答
            crab_health.alive = true;
            crab_health.last_response_ms = (uint32_t)millis();
            break;
          }
        }
      });
}

// hand_control topic の callback
IRAM_ATTR void on_control_command(const void* msg_in) {
  if (hc_active) return;  // ヘルスチェック中は通常コマンドを無視

  const auto* cmd = static_cast<const robot_msgs__msg__HandMessage*>(msg_in);

  if (xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
    target.yagura_1 = {cmd->yagura_1.pos, cmd->yagura_1.state};
    target.yagura_2 = {cmd->yagura_2.pos, cmd->yagura_2.state};
    target.ring_1 = {cmd->ring_1.pos, cmd->ring_1.state};
    target.ring_2 = {cmd->ring_2.pos, cmd->ring_2.state};
    target.crab = {cmd->crab.vgoal_led, cmd->crab.error_led};

    xSemaphoreGive(DataMutex);
  }

  // カニ機構LED: 状態が変わったときだけCAN送信
  if (target.crab.vgoal_led != current.crab.vgoal_led) {
    can_comm->transmit(can::CanTxMessageBuilder()
                           .set_dest(can::CanDest::crab_led)
                           .set_command(target.crab.vgoal_led ? 0x01 : 0x00)
                           .build());
    current.crab.vgoal_led = target.crab.vgoal_led;
  }
  if (target.crab.error_led != current.crab.error_led) {
    can_comm->transmit(can::CanTxMessageBuilder()
                           .set_dest(can::CanDest::crab_led)
                           .set_command(target.crab.error_led ? 0x11 : 0x10)
                           .build());
    current.crab.error_led = target.crab.error_led;
  }

  // TODO: CanTxMessageBuilder の仕様が今年と異なるので修正する

  for (int i = 0; i < 2; i++) {
    const ActuatorState& tgt = (i == 0) ? target.yagura_1 : target.yagura_2;
    const ActuatorState& cur = (i == 0) ? current.yagura_1 : current.yagura_2;

    ActuatorState& cur_mut = (i == 0) ? current.yagura_1 : current.yagura_2;

    auto dc_dest = (i == 0) ? can::CanDest::dc_lift_front : can::CanDest::dc_lift_rear;
    auto sv_dest = (i == 0) ? can::CanDest::servo_front : can::CanDest::servo_rear;

    if (tgt.pos == robot_msgs__msg__YaguraMechanism__POS_UP &&
        cur.pos != robot_msgs__msg__YaguraMechanism__POS_UP &&
        cur.pos != robot_msgs__msg__YaguraMechanism__POS_UP_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(dc_dest)
                             .set_command(0x01)
                             .build());
      cur_mut.pos = robot_msgs__msg__YaguraMechanism__POS_UP;
    } else if (tgt.pos == robot_msgs__msg__YaguraMechanism__POS_DOWN &&
               cur.pos != robot_msgs__msg__YaguraMechanism__POS_DOWN &&
               cur.pos != robot_msgs__msg__YaguraMechanism__POS_DOWN_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(dc_dest)
                             .set_command(0x00)
                             .build());
      cur_mut.pos = robot_msgs__msg__YaguraMechanism__POS_DOWN;
    } else if (tgt.pos == robot_msgs__msg__YaguraMechanism__POS_STOPPED &&
               cur.pos != robot_msgs__msg__YaguraMechanism__POS_STOPPED) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(dc_dest)
                             .set_command(0x02)
                             .build());
      cur_mut.pos = robot_msgs__msg__YaguraMechanism__POS_STOPPED;
    }

    if (tgt.state == robot_msgs__msg__YaguraMechanism__STATE_OPEN &&
        cur.state != robot_msgs__msg__YaguraMechanism__STATE_OPEN &&
        cur.state != robot_msgs__msg__YaguraMechanism__STATE_OPEN_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(sv_dest)
                             .set_command(0x21)
                             .build());
      cur_mut.state = robot_msgs__msg__YaguraMechanism__STATE_OPEN;
    } else if (tgt.state == robot_msgs__msg__YaguraMechanism__STATE_CLOSE &&
               cur.state != robot_msgs__msg__YaguraMechanism__STATE_CLOSE &&
               cur.state !=
                   robot_msgs__msg__YaguraMechanism__STATE_CLOSE_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(sv_dest)
                             .set_command(0x20)
                             .build());
      cur_mut.state = robot_msgs__msg__YaguraMechanism__STATE_CLOSE;
    } else if (tgt.state == robot_msgs__msg__YaguraMechanism__STATE_STOPPED &&
               cur.state != robot_msgs__msg__YaguraMechanism__STATE_STOPPED) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(sv_dest)
                             .set_command(0x22)
                             .build());
      cur_mut.state = robot_msgs__msg__YaguraMechanism__STATE_STOPPED;
    }
  }

  for (int i = 0; i < 2; i++) {
    const ActuatorState& tgt = (i == 0) ? target.ring_1 : target.ring_2;
    const ActuatorState& cur = (i == 0) ? current.ring_1 : current.ring_2;

    ActuatorState& cur_mut = (i == 0) ? current.ring_1 : current.ring_2;

    auto sv_dest = (i == 0) ? can::CanDest::servo_front : can::CanDest::servo_rear;

    if (tgt.pos == robot_msgs__msg__RingMechanism__POS_PICKUP &&
        cur.pos != robot_msgs__msg__RingMechanism__POS_PICKUP &&
        cur.pos != robot_msgs__msg__RingMechanism__POS_PICKUP_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(sv_dest)
                             .set_command(0x00)
                             .set_value(0x00)
                             .build());
      cur_mut.pos = robot_msgs__msg__RingMechanism__POS_PICKUP;
    } else if (tgt.pos == robot_msgs__msg__RingMechanism__POS_YAGURA &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_YAGURA &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_YAGURA_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(sv_dest)
                             .set_command(0x00)
                             .set_value(0x01)
                             .build());
      cur_mut.pos = robot_msgs__msg__RingMechanism__POS_YAGURA;
    } else if (tgt.pos == robot_msgs__msg__RingMechanism__POS_HONMARU &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_HONMARU &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_HONMARU_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(sv_dest)
                             .set_command(0x00)
                             .set_value(0x02)
                             .build());
      cur_mut.pos = robot_msgs__msg__RingMechanism__POS_HONMARU;
    } else if (tgt.pos == robot_msgs__msg__RingMechanism__POS_STOPPED &&
               cur.pos != robot_msgs__msg__RingMechanism__POS_STOPPED) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(sv_dest)
                             .set_command(0x01)
                             .build());
      cur_mut.pos = robot_msgs__msg__RingMechanism__POS_STOPPED;
    }

    if (tgt.state == robot_msgs__msg__RingMechanism__STATE_OPEN &&
        cur.state != robot_msgs__msg__RingMechanism__STATE_OPEN &&
        cur.state != robot_msgs__msg__RingMechanism__STATE_OPEN_DONE &&
        cur.state != robot_msgs__msg__RingMechanism__STATE_GRIP_FAIL) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(sv_dest)
                             .set_command(0x11)
                             .build());
      cur_mut.state = robot_msgs__msg__RingMechanism__STATE_OPEN;
    } else if (tgt.state == robot_msgs__msg__RingMechanism__STATE_CLOSE &&
               cur.state != robot_msgs__msg__RingMechanism__STATE_CLOSE &&
               cur.state != robot_msgs__msg__RingMechanism__STATE_CLOSE_DONE) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(sv_dest)
                             .set_command(0x10)
                             .build());
      cur_mut.state = robot_msgs__msg__RingMechanism__STATE_CLOSE;
    } else if (tgt.state == robot_msgs__msg__RingMechanism__STATE_STOPPED &&
               cur.state != robot_msgs__msg__RingMechanism__STATE_STOPPED) {
      can_comm->transmit(can::CanTxMessageBuilder()
                             .set_dest(sv_dest)
                             .set_command(0x12)
                             .build());
      cur_mut.state = robot_msgs__msg__RingMechanism__STATE_STOPPED;
    }
  }
}

// health_check topic の callback
IRAM_ATTR void on_health_check_command(const void* msg_in) {
  const auto* cmd = static_cast<const std_msgs__msg__Bool*>(msg_in);
  if (cmd->data && !hc_active) {
    hc_active = true;
    hc_step   = 1;
    hc_time   = (uint32_t)millis();
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
    feedback_msg.crab.vgoal_led = current.crab.vgoal_led;
    feedback_msg.crab.error_led = current.crab.error_led;
    feedback_msg.crab.crab_alive = crab_health.alive;
    feedback_msg.servo_front_alive = servo_health[0].alive;
    feedback_msg.servo_rear_alive = servo_health[1].alive;
    feedback_msg.dc_lift_front_alive = dc_lift_health[0].alive;
    feedback_msg.dc_lift_rear_alive = dc_lift_health[1].alive;

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

  if (rclc_subscription_init_default(
          &sub_health_check, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
          "health_check") != RCL_RET_OK)
    return false;

  if (rclc_timer_init_default(&timer_feedback, &support, RCL_MS_TO_NS(50),
                              timer_feedback_callback) != RCL_RET_OK)
    return false;

  robot_msgs__msg__HandMessage__init(&control_msg);
  robot_msgs__msg__HandMessage__init(&feedback_msg);
  std_msgs__msg__Bool__init(&hc_control_msg);

  if (rclc_executor_init(&executor, &support.context, 3, &allocator) !=
      RCL_RET_OK)
    return false;
  if (rclc_executor_add_subscription(&executor, &sub_control, &control_msg,
                                     &on_control_command,
                                     ON_NEW_DATA) != RCL_RET_OK)
    return false;
  if (rclc_executor_add_timer(&executor, &timer_feedback) != RCL_RET_OK)
    return false;
  if (rclc_executor_add_subscription(&executor, &sub_health_check,
                                     &hc_control_msg, &on_health_check_command,
                                     ON_NEW_DATA) != RCL_RET_OK)
    return false;

  return true;
}

static void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&executor);
  rcl_timer_fini(&timer_feedback);
  rcl_subscription_fini(&sub_control, &node);
  rcl_subscription_fini(&sub_health_check, &node);
  rcl_publisher_fini(&pub_feedback, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  robot_msgs__msg__HandMessage__fini(&control_msg);
  robot_msgs__msg__HandMessage__fini(&feedback_msg);
  std_msgs__msg__Bool__fini(&hc_control_msg);
}

// Control Task: Core 1, 最高優先度
void ControlTask(void* pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(3);
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    // CAN 受信処理（キューを 1 つずつ処理）
    can_comm->process_received_messages();

    // サーボ基板へ定期ヘルスチェック要求
    {
      uint32_t now_ms = (uint32_t)millis();
      if (now_ms - last_health_request_ms >= HEALTH_REQUEST_INTERVAL_MS) {
        last_health_request_ms = now_ms;
        for (auto dest : {can::CanDest::servo_front, can::CanDest::servo_rear,
                          can::CanDest::dc_lift_front, can::CanDest::dc_lift_rear,
                          can::CanDest::crab_led}) {
          can_comm->transmit(can::CanTxMessageBuilder()
                                 .set_dest(dest)
                                 .set_command(0xFF)
                                 .build());
        }
        // サーボ基板タイムアウト判定
        const char* servo_names[] = {"front", "rear"};
        for (int i = 0; i < 2; i++) {
          bool was_alive = servo_health[i].alive;
          if (servo_health[i].last_response_ms != 0 &&
              now_ms - servo_health[i].last_response_ms >= HEALTH_TIMEOUT_MS) {
            servo_health[i].alive = false;
          }
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL != 1)
          if (was_alive && !servo_health[i].alive) {
            Serial.printf("servo %s: timeout\n", servo_names[i]);
          } else if (!was_alive && servo_health[i].alive) {
            Serial.printf("servo %s: alive\n", servo_names[i]);
          }
#endif
        }
        // DC リフト基板タイムアウト判定
        const char* dc_names[] = {"front", "rear"};
        for (int i = 0; i < 2; i++) {
          bool was_alive = dc_lift_health[i].alive;
          if (dc_lift_health[i].last_response_ms != 0 &&
              now_ms - dc_lift_health[i].last_response_ms >= HEALTH_TIMEOUT_MS) {
            dc_lift_health[i].alive = false;
          }
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL != 1)
          if (was_alive && !dc_lift_health[i].alive) {
            Serial.printf("dc_lift %s: timeout\n", dc_names[i]);
          } else if (!was_alive && dc_lift_health[i].alive) {
            Serial.printf("dc_lift %s: alive\n", dc_names[i]);
          }
#endif
        }
        // crub_igniter タイムアウト判定
        {
          bool was_alive = crab_health.alive;
          if (crab_health.last_response_ms != 0 &&
              now_ms - crab_health.last_response_ms >= HEALTH_TIMEOUT_MS) {
            crab_health.alive = false;
          }
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL != 1)
          if (was_alive && !crab_health.alive) {
            Serial.println("crab: timeout");
          } else if (!was_alive && crab_health.alive) {
            Serial.println("crab: alive");
          }
#endif
        }
      }
    }

    // 動的ヘルスチェック状態機械
    if (hc_active) {
      uint32_t now_ms = (uint32_t)millis();
      uint32_t elapsed = now_ms - hc_time;
      switch (hc_step) {
        case 1:  // リングハンド 閉じる
          for (auto dest : {can::CanDest::servo_front, can::CanDest::servo_rear}) {
            can_comm->transmit(can::CanTxMessageBuilder().set_dest(dest).set_command(0x10).build());
          }
          hc_step = 2; hc_time = now_ms;
          break;
        case 2:  // 500ms後: リングハンド 開く
          if (elapsed >= 500) {
            for (auto dest : {can::CanDest::servo_front, can::CanDest::servo_rear}) {
              can_comm->transmit(can::CanTxMessageBuilder().set_dest(dest).set_command(0x11).build());
            }
            hc_step = 3; hc_time = now_ms;
          }
          break;
        case 3:  // 500ms後: 櫓ハンド 閉じる
          if (elapsed >= 500) {
            for (auto dest : {can::CanDest::servo_front, can::CanDest::servo_rear}) {
              can_comm->transmit(can::CanTxMessageBuilder().set_dest(dest).set_command(0x20).build());
            }
            hc_step = 4; hc_time = now_ms;
          }
          break;
        case 4:  // 1500ms後: 櫓ハンド 開く
          if (elapsed >= 1500) {
            for (auto dest : {can::CanDest::servo_front, can::CanDest::servo_rear}) {
              can_comm->transmit(can::CanTxMessageBuilder().set_dest(dest).set_command(0x21).build());
            }
            hc_step = 5; hc_time = now_ms;
          }
          break;
        case 5:  // 1500ms後: リフト 上げる
          if (elapsed >= 1500) {
            for (auto dest : {can::CanDest::dc_lift_front, can::CanDest::dc_lift_rear}) {
              can_comm->transmit(can::CanTxMessageBuilder().set_dest(dest).set_command(0x01).build());
            }
            hc_step = 6; hc_time = now_ms;
          }
          break;
        case 6:  // 1200ms後: リフト 下げる
          if (elapsed >= 1200) {
            for (auto dest : {can::CanDest::dc_lift_front, can::CanDest::dc_lift_rear}) {
              can_comm->transmit(can::CanTxMessageBuilder().set_dest(dest).set_command(0x00).build());
            }
            hc_step = 7; hc_time = now_ms;
          }
          break;
        case 7:  // 1200ms後: リフト 停止
          if (elapsed >= 1200) {
            for (auto dest : {can::CanDest::dc_lift_front, can::CanDest::dc_lift_rear}) {
              can_comm->transmit(can::CanTxMessageBuilder().set_dest(dest).set_command(0x02).build());
            }
            hc_step = 8; hc_time = now_ms;
          }
          break;
        case 8:  // vgoal_led + error_led ON
          can_comm->transmit(can::CanTxMessageBuilder().set_dest(can::CanDest::crab_led).set_command(0x01).build());
          can_comm->transmit(can::CanTxMessageBuilder().set_dest(can::CanDest::crab_led).set_command(0x11).build());
          hc_step = 9; hc_time = now_ms;
          break;
        case 9:  // 500ms後: vgoal_led + error_led OFF → 完了
          if (elapsed >= 500) {
            can_comm->transmit(can::CanTxMessageBuilder().set_dest(can::CanDest::crab_led).set_command(0x00).build());
            can_comm->transmit(can::CanTxMessageBuilder().set_dest(can::CanDest::crab_led).set_command(0x10).build());
            hc_active = false; hc_step = 0;
          }
          break;
        default: break;
      }
    }

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
  uint32_t waiting_ticks = 0;

  while (1) {
    switch (agent_state) {
      case AgentState::WAITING:
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          if (create_entities()) {
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL != 1)
            Serial.println("micro-ROS: connected");
#endif
            ping_counter = 0;
            waiting_ticks = 0;
            agent_state = AgentState::CONNECTED;
          } else {
            destroy_entities();
          }
        } else {
          // 30 秒間再接続できなければ transport 層 stale 等からの回復のためリセット
          if (++waiting_ticks >= 60) {
            esp_restart();
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
        waiting_ticks = 0;
        destroy_entities();
        agent_state = AgentState::WAITING;
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.setSleep(false);  // WiFi 省電力モードをオフ

  DataMutex = xSemaphoreCreateMutex();

  // CAN 通信の初期化（フィルタなし = 全受信）
  can_comm = new can::CanCommunicator();
  can_comm->setup();
  register_can_event_handlers();
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
