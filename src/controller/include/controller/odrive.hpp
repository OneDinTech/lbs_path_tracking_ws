#ifndef ODRIVE_HPP
#define ODRIVE_HPP

#include "can.hpp"
#include "packer.hpp"
#include <cstdint>

enum class ControlMode : uint32_t {
    VOLTAGE  = 0,
    TORQUE   = 1,
    VELOCITY = 2,
    POSITION = 3,
};

enum class AxisState : uint32_t {
    UNDEFINED            = 0,
    IDLE                 = 1,
    STARTUP_SEQUENCE     = 2,
    FULL_CALIBRATION     = 3,
    MOTOR_CALIBRATION    = 4,
    ENCODER_INDEX_SEARCH = 6,
    ENCODER_OFFSET_CALIB = 7,
    CLOSED_LOOP_CONTROL  = 8,
    LOCKIN_SPIN          = 9,
    HOMING               = 11,
};

enum class InputMode : uint32_t {
    INACTIVE    = 0,
    PASSTHROUGH = 1,
    VEL_RAMP    = 2,
    POS_FILTER  = 3,
    TRAP_TRAJ   = 5,
    TORQUE_RAMP = 6,
};

// CANSimple command IDs
static constexpr uint8_t CMD_SET_AXIS_STATE   = 0x007;
static constexpr uint8_t CMD_SET_CTRL_MODE    = 0x00B;
static constexpr uint8_t CMD_SET_LIMITS       = 0x00F;
static constexpr uint8_t CMD_SET_ABSOLUTE_POS = 0x019;
static constexpr uint8_t CMD_SET_POS_GAIN     = 0x01A;
static constexpr uint8_t CMD_SET_VEL_GAINS    = 0x01B;
static constexpr uint8_t CMD_VEL_CONTROL      = 0X00D;

class ODrive {
public:
    explicit ODrive(CAN& can);

    CANError odrive_init(
        uint8_t     node_id,
        InputMode   inputmode,
        ControlMode controlmode,
        AxisState   axisstate,
        float       vel_limit,
        float       current_limit,
        float       pos_gain,
        float       vel_gain,
        float       vel_integrator_gain,
        bool        arm,
        bool        zero_position
    );

    CANError send_speed(uint8_t node_id, float vel);
    bool receive_speed(uint8_t node_id, float& r_vel);

private:

    uint16_t make_can_id(uint8_t node_id, uint8_t cmd_id);

    CAN&    can_;
    uint8_t node_id_;
};

#endif // ODRIVE_HPP