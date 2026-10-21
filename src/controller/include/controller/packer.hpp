#ifndef PACKER_HPP
#define PACKER_HPP

#include <cstdint>

namespace Packer {

// pack
void pack_f32(uint8_t* data, float value);
void pack_i16(uint8_t* data, int16_t value);
void pack_i32(uint8_t* data, int32_t value);

// unpack
float   unpack_f32(const uint8_t* data);
int16_t unpack_i16(const uint8_t* data);
int32_t unpack_i32(const uint8_t* data);

} // namespace Packer

#endif // PACKER_HPP