#ifndef DRIVER_HPP
#define DRIVER_HPP

#include "can.hpp"
#include <cstdint>

class Driver {
public:
    explicit Driver(CAN& can);

    // TX: matches motor_command_callback exactly.
    //   byte 0   = mode flags: bit0=positionmode, bit1=speedmode,
    //              bit2=voltagemode, bit3=stop, bit4=reset
    //   byte 1   = unused (reserved / ramp-rate on some firmware versions)
    //   bytes 2-5 = goal, float32 LE
    //   bytes 6-7 = unused
    // A single frame can only be EITHER speed OR position mode -- the
    // mode bit tells the motor how to interpret the goal float. This is
    // why speed and steering live on separate CAN ids/motors.
    CANError send_speed(uint16_t can_id, float speed);
    CANError send_position(uint16_t can_id, float position);

    // sends a stop-mode frame (mode bit3=stop, goal=0). Use on shutdown
    // so the motor doesn't keep running its last command indefinitely.
    CANError send_stop(uint16_t can_id);

    // RX: matches on_message_received's "100 <= arbitration_id < 200"
    // (EncoderFeedback) branch exactly.
    //   feedback id = can_id + 100
    //   bytes 0-3 = position, float32 LE
    //   bytes 4-7 = speed,    float32 LE
    // Both fields arrive together on one feedback frame regardless of
    // which mode the corresponding TX command used.
    bool read_position(uint16_t can_id, float& out_position);
    bool read_speed(uint16_t can_id, float& out_speed);
    bool read_speed_and_position(uint16_t can_id, float& out_speed, float& out_position);

    // reads a raw encoder feedback frame at an ABSOLUTE can id, no
    // offset -- for passive/dead-wheel sensors that have their own
    // fixed id and aren't paired with any TX command (unlike the
    // swerve motors above). Same layout: bytes 0-3=position,
    // bytes 4-7=speed, both float32 LE. Matches can_driver.py's
    // generic "100 <= arbitration_id < 200" EncoderFeedback branch.
    bool read_encoder_raw(uint16_t abs_can_id, float& out_position, float& out_speed);

private:
    CAN& can_;
};

#endif // DRIVER_HPP