#ifndef VESC_HPP
#define VESC_HPP

#include "can.hpp"
#include "packer.hpp"
#include <cstdint>
#include <cmath>
#include <unordered_map>

class VESC
{
public:
    explicit VESC(CAN& can);

    // TX — unchanged
    CANError set_duty(uint8_t vesc_id, float duty);
    CANError set_current(uint8_t vesc_id, float amps);
    CANError set_current_brake(uint8_t vesc_id, float amps);
    CANError send_speed(uint8_t vesc_id, float mech_rpm, uint8_t pole_pairs);
    CANError send_position(uint8_t vesc_id, float degrees);

    // RX — same names, same signatures, now internally lossless
    bool read_speed(uint8_t vesc_id, uint8_t pole_pairs, float& out_mech_rpm);
    bool read_current(uint8_t vesc_id, float& out_current);
    bool read_duty(uint8_t vesc_id, float& out_duty);
    bool read_fet_temp(uint8_t vesc_id, float& out_temp);
    bool read_motor_temp(uint8_t vesc_id, float& out_temp);
    bool read_input_current(uint8_t vesc_id, float& out_current);
    bool read_pid_position(uint8_t vesc_id, float& out_position);
    bool read_amp_hours(uint8_t vesc_id, float& out_ah);
    bool read_watt_hours(uint8_t vesc_id, float& out_wh);

private:
    struct VescCache {
        float rpm_erpm      = 0.0f;
        float current       = 0.0f;
        float duty          = 0.0f;
        float fet_temp       = 0.0f;
        float motor_temp     = 0.0f;
        float input_current  = 0.0f;
        float pid_position   = 0.0f;
        float amp_hours      = 0.0f;
        float watt_hours     = 0.0f;
        bool  has_data       = false;
    };

    // drains ALL frames currently sitting in the kernel socket buffer
    // called automatically at the start of every read_* call
    void drain_all();

    CAN& can_;
    std::unordered_map<uint8_t, VescCache> cache_;
};

#endif // VESC_HPP