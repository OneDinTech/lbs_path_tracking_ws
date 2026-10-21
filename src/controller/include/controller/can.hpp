#ifndef SOCKET_CAN_HPP
#define SOCKET_CAN_HPP

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdint>
#include <string>

enum class CANError {
    OK = 0,
    NOT_OPEN,
    BUFFER_FULL,
    DISCONNECTED,
    TIMEOUT,
    INVALID_FRAME,
    UNKNOWN
};

class CAN {
public:
    explicit CAN(const std::string& interface = "can0",
                 int bitrate = 1'000'000,
                 int tx_queue_len = 1000);
    ~CAN();

    bool enable(bool configure_link = true);
    bool disable(bool bring_link_down = false);

    // ---- standard 11-bit ID (swerve, existing Driver class) ----
    CANError send_frame(uint16_t id, const uint8_t* data, uint8_t len);
    CANError receive_frame(uint16_t& id, uint8_t* data, uint8_t& len, int timeout_ms = 50);

    // ---- extended 29-bit ID (VESC) ----
    CANError send_frame(uint32_t id, const uint8_t* data, uint8_t len, bool extended);
    CANError receive_frame(uint32_t& id, uint8_t* data, uint8_t& len, bool& extended, int timeout_ms = 50);

    void flush_receive_buffer();
    bool set_new_bitrate(int new_bitrate);
    bool is_interface_up() const;

    uint32_t tx_count()    const { return tx_count_; }
    uint32_t rx_count()    const { return rx_count_; }
    uint32_t drop_count()  const { return drop_count_; }
    void reset_stats() { tx_count_ = rx_count_ = drop_count_ = 0; }

private:
    bool run_ip(const std::string& cmd);
    bool set_socket_buffer_size();

    std::string iface_;
    int bitrate_;
    int tx_queue_len_;
    int sock_ = -1;

    uint32_t tx_count_   = 0;
    uint32_t rx_count_   = 0;
    uint32_t drop_count_ = 0;
};

#endif