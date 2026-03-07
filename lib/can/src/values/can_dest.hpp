#pragma once

#include "can_id.hpp"

namespace can {
/**
 * @brief CAN 1 & 2 Modules (Based on can.md)
 */
enum class CanDest : can::CanId {
  // CAN 1: Mechanism System
  central_mech = 0x000,
  power_12v = 0x100,
  dc_lift_front = 0x300,  // DC モータ制御基板（前）: 昇降 1
  dc_lift_rear = 0x301,   // DC モータ制御基板（後）: 昇降 2
  servo_front = 0x400,    // サーボ制御基板（前）: ring_1 + yagura_1
  servo_rear = 0x401,     // サーボ制御基板（後）: ring_2 + yagura_2

  // CAN 2: Drive System
  central_drive = 0x001,
  power_24v = 0x101,
  m3508_ctrl = 0x200,
  m3508_1_fb = 0x201,
  m3508_2_fb = 0x202,
  m3508_3_fb = 0x203,
  m3508_4_fb = 0x204,
};

constexpr bool operator==(const can::CanId id, const can::CanDest dest) {
  return id == static_cast<can::CanId>(dest);
}

constexpr bool operator==(const can::CanDest dest, const can::CanId id) {
  return id == static_cast<can::CanId>(dest);
}
}  // namespace can
