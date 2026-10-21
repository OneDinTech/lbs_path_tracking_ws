#include "can.hpp"
#include <cerrno>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <sys/socket.h>
#include <sys/select.h>
#include <ctime>

CAN::CAN(const std::string& interface, int bitrate, int tx_queue_len)
: iface_(interface), bitrate_(bitrate), tx_queue_len_(tx_queue_len) {}

CAN::~CAN()
{
    disable(false);
}

bool CAN::run_ip(const std::string& ip_cmd)
{
    const char* IP = "/sbin/ip";
    std::string base = std::string((access(IP, X_OK) == 0) ? IP : "ip") + " " + ip_cmd;
    std::string cmd  = (geteuid() == 0) ? base : "sudo " + base;
    int rc = std::system(cmd.c_str());
    if (rc == -1) return false;
#ifdef WEXITSTATUS
    return (WEXITSTATUS(rc) == 0);
#else
    return (rc == 0);
#endif
}

bool CAN::is_interface_up() const
{
    FILE* f = popen(("ip link show " + iface_).c_str(), "r");
    if (!f) return false;
    char buf[256];
    std::string out;
    while (fgets(buf, sizeof(buf), f)) out += buf;
    pclose(f);
    return out.find("state UP") != std::string::npos;
}

bool CAN::set_socket_buffer_size()
{
    const int sndbuf = 131072 * 4;
    const int rcvbuf = 131072 * 4;
    if (setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
        std::cerr << "[CAN] Warning: could not set SO_SNDBUF: " << strerror(errno) << "\n";
    }
    if (setsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
        std::cerr << "[CAN] Warning: could not set SO_RCVBUF: " << strerror(errno) << "\n";
    }
    return true;
}

bool CAN::enable(bool configure_link)
{
    if (sock_ != -1)
        return true;

    if (configure_link) {
        // always bring down first — overrides any auto-up from the OS
        (void)run_ip("link set " + iface_ + " down");

        if (!run_ip("link set " + iface_ +
                    " type can bitrate " + std::to_string(bitrate_)))
            return false;

        (void)run_ip("link set " + iface_ +
                     " txqueuelen " + std::to_string(tx_queue_len_));

        if (!run_ip("link set " + iface_ + " up"))
            return false;
    }

    sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        perror("[CAN] socket");
        return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        perror("[CAN] ioctl SIOCGIFINDEX");
        ::close(sock_); sock_ = -1;
        return false;
    }

    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("[CAN] bind");
        ::close(sock_); sock_ = -1;
        return false;
    }

    set_socket_buffer_size();

    std::cout << "[CAN] " << iface_ << " open at "
              << bitrate_ << " bps, txqueuelen=" << tx_queue_len_ << "\n";
    return true;
}

bool CAN::disable(bool bring_link_down)
{
    if (sock_ != -1) {
        ::close(sock_);
        sock_ = -1;
    }
    if (bring_link_down)
        (void)run_ip("link set " + iface_ + " down");
    return true;
}

// ─── standard 11-bit ID ─────────────────────────────────────

CANError CAN::send_frame(uint16_t id, const uint8_t* data, uint8_t len)
{
    if (sock_ < 0)  return CANError::NOT_OPEN;
    if (len > 8)    return CANError::INVALID_FRAME;

    struct can_frame frame {};
    frame.can_id  = id & CAN_SFF_MASK;
    frame.can_dlc = len;
    if (len) std::memcpy(frame.data, data, len);

    for (int attempt = 0; attempt < 2; ++attempt) {
        ssize_t rc = ::write(sock_, &frame, sizeof(frame));
        if (rc == static_cast<ssize_t>(sizeof(frame))) {
            ++tx_count_;
            return CANError::OK;
        }
        if (errno == ENOBUFS) {
            if (attempt == 0) {
                struct timespec ts {0, 1'000'000};
                nanosleep(&ts, nullptr);
                continue;
            }
            ++drop_count_;
            return CANError::BUFFER_FULL;
        }
        ++drop_count_;
        if (errno == ENETDOWN || errno == ENODEV) return CANError::DISCONNECTED;
        return CANError::UNKNOWN;
    }
    return CANError::UNKNOWN;
}

CANError CAN::receive_frame(uint16_t& id, uint8_t* data, uint8_t& len, int timeout_ms)
{
    if (sock_ < 0) return CANError::NOT_OPEN;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(sock_, &read_fds);

    struct timeval tv {};
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int rc = ::select(sock_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (rc < 0)  return CANError::UNKNOWN;
    if (rc == 0) return CANError::TIMEOUT;

    struct can_frame frame {};
    if (::read(sock_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame)))
        return CANError::UNKNOWN;

    if (frame.can_id & CAN_ERR_FLAG)
        return CANError::UNKNOWN;

    // skip extended frames here — this overload is for standard IDs only
    if (frame.can_id & CAN_EFF_FLAG)
        return CANError::TIMEOUT;   // treat as "nothing relevant" for this overload

    id  = static_cast<uint16_t>(frame.can_id & CAN_SFF_MASK);
    len = frame.can_dlc;
    if (len) std::memcpy(data, frame.data, len);

    ++rx_count_;
    return CANError::OK;
}

// ─── extended 29-bit ID (VESC) ──────────────────────────────

CANError CAN::send_frame(uint32_t id, const uint8_t* data, uint8_t len, bool extended)
{
    if (sock_ < 0)  return CANError::NOT_OPEN;
    if (len > 8)    return CANError::INVALID_FRAME;

    struct can_frame frame {};
    frame.can_id = extended ? ((id & CAN_EFF_MASK) | CAN_EFF_FLAG)
                             : (id & CAN_SFF_MASK);
    frame.can_dlc = len;
    if (len) std::memcpy(frame.data, data, len);

    for (int attempt = 0; attempt < 2; ++attempt) {
        ssize_t rc = ::write(sock_, &frame, sizeof(frame));
        if (rc == static_cast<ssize_t>(sizeof(frame))) {
            ++tx_count_;
            return CANError::OK;
        }
        if (errno == ENOBUFS) {
            if (attempt == 0) {
                struct timespec ts {0, 1'000'000};
                nanosleep(&ts, nullptr);
                continue;
            }
            ++drop_count_;
            return CANError::BUFFER_FULL;
        }
        ++drop_count_;
        if (errno == ENETDOWN || errno == ENODEV) return CANError::DISCONNECTED;
        return CANError::UNKNOWN;
    }
    return CANError::UNKNOWN;
}

CANError CAN::receive_frame(uint32_t& id, uint8_t* data, uint8_t& len, bool& extended, int timeout_ms)
{
    if (sock_ < 0) return CANError::NOT_OPEN;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(sock_, &read_fds);

    struct timeval tv {};
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int rc = ::select(sock_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (rc < 0)  return CANError::UNKNOWN;
    if (rc == 0) return CANError::TIMEOUT;

    struct can_frame frame {};
    if (::read(sock_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame)))
        return CANError::UNKNOWN;

    if (frame.can_id & CAN_ERR_FLAG)
        return CANError::UNKNOWN;

    extended = (frame.can_id & CAN_EFF_FLAG) != 0;
    id  = extended ? (frame.can_id & CAN_EFF_MASK) : (frame.can_id & CAN_SFF_MASK);
    len = frame.can_dlc;
    if (len) std::memcpy(data, frame.data, len);

    ++rx_count_;
    return CANError::OK;
}

void CAN::flush_receive_buffer()
{
    if (sock_ < 0) return;
    struct can_frame frame {};
    while (::recv(sock_, &frame, sizeof(frame), MSG_DONTWAIT)
           == static_cast<ssize_t>(sizeof(frame))) {}
}

bool CAN::set_new_bitrate(int new_bitrate)
{
    std::cout << "[CAN] Changing bitrate " << bitrate_
              << " → " << new_bitrate << "\n";
    disable(true);
    if (!run_ip("link set " + iface_ + " down") ||
        !run_ip("link set " + iface_ +
                " type can bitrate " + std::to_string(new_bitrate)) ||
        !run_ip("link set " + iface_ + " up")) {
        std::cerr << "[CAN] Failed to reconfigure bitrate\n";
        return false;
    }
    bitrate_ = new_bitrate;
    if (!enable(false)) {
        std::cerr << "[CAN] Failed to reopen socket\n";
        return false;
    }
    std::cout << "[CAN] Bitrate changed OK\n";
    return true;
}