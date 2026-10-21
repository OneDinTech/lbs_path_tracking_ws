#include "packer.hpp"
#include <cstring>

namespace Packer {

// ─── pack ───────────────────────────────────────────────────

void pack_f32(uint8_t* data, float value)
{
    uint32_t raw = 0;
    std::memcpy(&raw, &value, sizeof(raw));
    data[0] = static_cast<uint8_t>(raw         & 0xFFu);
    data[1] = static_cast<uint8_t>((raw >>  8) & 0xFFu);
    data[2] = static_cast<uint8_t>((raw >> 16) & 0xFFu);
    data[3] = static_cast<uint8_t>((raw >> 24) & 0xFFu);
}

void pack_i16(uint8_t* data, int16_t value)
{
    const uint16_t raw = static_cast<uint16_t>(value);
    data[0] = static_cast<uint8_t>((raw >> 8) & 0xFFu);
    data[1] = static_cast<uint8_t>(raw        & 0xFFu);
}

void pack_i32(uint8_t* data, int32_t value)
{
    const uint32_t raw = static_cast<uint32_t>(value);
    data[0] = static_cast<uint8_t>((raw >> 24) & 0xFFu);
    data[1] = static_cast<uint8_t>((raw >> 16) & 0xFFu);
    data[2] = static_cast<uint8_t>((raw >>  8) & 0xFFu);
    data[3] = static_cast<uint8_t>(raw         & 0xFFu);
}

// ─── unpack ─────────────────────────────────────────────────

float unpack_f32(const uint8_t* data)
{
    const uint32_t raw =
        static_cast<uint32_t>(data[0])         |
        (static_cast<uint32_t>(data[1]) <<  8) |
        (static_cast<uint32_t>(data[2]) << 16) |
        (static_cast<uint32_t>(data[3]) << 24);
    float value = 0.0f;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

int16_t unpack_i16(const uint8_t* data)
{
    const uint16_t raw =
        (static_cast<uint16_t>(data[0]) << 8) |
         static_cast<uint16_t>(data[1]);
    return static_cast<int16_t>(raw);
}

int32_t unpack_i32(const uint8_t* data)
{
    const uint32_t raw =
        (static_cast<uint32_t>(data[0]) << 24) |
        (static_cast<uint32_t>(data[1]) << 16) |
        (static_cast<uint32_t>(data[2]) <<  8) |
         static_cast<uint32_t>(data[3]);
    return static_cast<int32_t>(raw);
}

} // namespace Packer