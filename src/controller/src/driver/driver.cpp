#include "driver.hpp"
#include "packer.hpp"

Driver::Driver(CAN& can)
: can_(can) {}

// ─── TX -- matches motor_command_callback ───────────────────────────
// mode bits (byte 0): bit0=positionmode, bit1=speedmode
static constexpr uint8_t MODE_POSITION = 0x01;
static constexpr uint8_t MODE_SPEED    = 0x02;
static constexpr uint8_t MODE_STOP     = 0x08;

CANError Driver::send_speed(uint16_t can_id, float speed)
{
    uint8_t data[8] {};
    data[0] = MODE_SPEED;
    Packer::pack_f32(data + 2, speed);   // goal at bytes 2-5
    return can_.send_frame(can_id, data, 8);
}

CANError Driver::send_position(uint16_t can_id, float position)
{
    uint8_t data[8] {};
    data[0] = MODE_POSITION;
    Packer::pack_f32(data + 2, position);   // goal at bytes 2-5
    return can_.send_frame(can_id, data, 8);
}

CANError Driver::send_stop(uint16_t can_id)
{
    uint8_t data[8] {};
    data[0] = MODE_STOP;
    // goal left at 0.0
    return can_.send_frame(can_id, data, 8);
}

// ─── RX -- matches on_message_received, 100 <= id < 200 branch ──────
// feedback id = can_id + 100
// bytes 0-3 = position, bytes 4-7 = speed, both float32 LE

bool Driver::read_position(uint16_t can_id, float& out_position)
{
    uint16_t id {}; uint8_t data[8] {}; uint8_t len {};
    if (can_.receive_frame(id, data, len) != CANError::OK) return false;
    if (id != can_id + 100U || len < 8) return false;
    out_position = Packer::unpack_f32(data + 0);
    return true;
}

bool Driver::read_speed(uint16_t can_id, float& out_speed)
{
    uint16_t id {}; uint8_t data[8] {}; uint8_t len {};
    if (can_.receive_frame(id, data, len) != CANError::OK) return false;
    if (id != can_id + 100U || len < 8) return false;
    out_speed = Packer::unpack_f32(data + 4);
    return true;
}

bool Driver::read_speed_and_position(uint16_t can_id, float& out_speed, float& out_position)
{
    uint16_t id {}; uint8_t data[8] {}; uint8_t len {};
    if (can_.receive_frame(id, data, len) != CANError::OK) return false;
    if (id != can_id + 100U || len < 8) return false;
    out_position = Packer::unpack_f32(data + 0);
    out_speed    = Packer::unpack_f32(data + 4);
    return true;
}

bool Driver::read_encoder_raw(uint16_t abs_can_id, float& out_position, float& out_speed)
{
    uint16_t id {}; uint8_t data[8] {}; uint8_t len {};
    if (can_.receive_frame(id, data, len) != CANError::OK) return false;
    if (id != abs_can_id || len < 8) return false;   // exact id, no offset
    out_position = Packer::unpack_f32(data + 0);
    out_speed    = Packer::unpack_f32(data + 4);
    return true;
}