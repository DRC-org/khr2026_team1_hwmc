#include "can_tx_message_builder.hpp"

namespace can {
CanTxMessageBuilder::CanTxMessageBuilder()
    : id(0), command(0), value(0), omake({0, 0, 0}) {}

CanTxMessageBuilder& CanTxMessageBuilder::set_id(const can::CanId id) {
  this->id = id;
  return *this;
}

CanTxMessageBuilder& CanTxMessageBuilder::set_dest(const CanDest dest) {
  this->id = static_cast<can::CanId>(dest);
  return *this;
}

CanTxMessageBuilder& CanTxMessageBuilder::set_command(const uint8_t command) {
  this->command = command;
  return *this;
}

CanTxMessageBuilder& CanTxMessageBuilder::set_value(const uint8_t value) {
  this->value = value;
  return *this;
}

CanTxMessageBuilder& CanTxMessageBuilder::set_omake(
    const std::array<uint8_t, 3> omake) {
  this->omake = omake;
  return *this;
}

CanTxMessageBuilder& CanTxMessageBuilder::set_omake_0(const uint8_t omake) {
  this->omake[0] = omake;
  return *this;
}

CanTxMessageBuilder& CanTxMessageBuilder::set_omake_1(const uint8_t omake) {
  this->omake[1] = omake;
  return *this;
}

CanTxMessageBuilder& CanTxMessageBuilder::set_omake_2(const uint8_t omake) {
  this->omake[2] = omake;
  return *this;
}

CanTxMessage CanTxMessageBuilder::build() const {
  return CanTxMessage(id, {command, value, omake[0], omake[1], omake[2], 0, 0, 0});
}
}  // namespace can