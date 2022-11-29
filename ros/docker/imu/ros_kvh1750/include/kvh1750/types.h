/**
 * Basic types for KVH 1750 interfacing.
 * \author Jason Ziglar <jpz@vt.edu>
 * \date 02/16/2015
 */
#ifndef _KVH_1750_TYPES_H_
#define _KVH_1750_TYPES_H_

#include "bit_message.h"
#include "iomodule.h"

//Make C++11x support not required
#if __cplusplus >= 201103L
#include <array>
//C++11 macro for casting ints to chars, to avoid narrowing
namespace util
{
template <typename... A>
constexpr std::array<uint8_t, sizeof...(A)> byte_array(A... v)
{ return std::array<uint8_t, sizeof...(A)>{{static_cast<uint8_t>(v)...}}; }

}

#define MAKE_BYTE_ARRAY(...) util::byte_array(__VA_ARGS__)

#define ARRAY_TYPE std::array

#else

#include <tr1/array>

#define MAKE_BYTE_ARRAY(...) {__VA_ARGS__}
#define ARRAY_TYPE std::tr1::array

#endif

#include <string>
#include <stdint.h>

namespace kvh
{

//! Constant for convering accelerations to floating point values
const float Gravity = 9.80665;

const size_t HeaderSize = 4;
typedef ARRAY_TYPE<uint8_t, HeaderSize> HeaderType;
//! IMU Message Header
const HeaderType IMUHeader = MAKE_BYTE_ARRAY(0xFE, 0x81, 0xFF, 0x55);
//! BIT Message Header
const HeaderType BITHeader = MAKE_BYTE_ARRAY(0xFE, 0x81, 0x00, 0xAA);
//! Max operating temperature of the KVH 1750
const int16_t MaxTemp_C = 75;

/**
 * Enumeration defining which bits in the status field correspond
 * to various sub-components of the system.
 */
typedef enum
{
  GYRO_X = 1,
  GYRO_Y = 1 << 1,
  GYRO_Z = 1 << 2,
  ACCEL_X = 1 << 4,
  ACCEL_Y = 1 << 5,
  ACCEL_Z = 1 << 6
} StatusBits;

typedef enum
{
  X = 0,
  Y,
  Z,
  NUM_FIELDS
} FieldOrder;

//! CRC Computation values from KVH
namespace crc
{
//! Width of the CRC
const size_t Width = 32;
//! Starting Polynomial value
const uint64_t Poly = 0x04C11DB7;
//! Mask for XOr Input operation
const uint32_t XOr_In = 0xFFFFFFFF;
//! Mask XOr Output Operation
const uint32_t XOr_Out = 0x0u;
//! Reflect In Flag
const bool Reflect_In = false;
//! Reflect Out Flag
const bool Reflect_Out = false;
}

#pragma pack(push, 1)
/**
 * Raw message format from KVH, straight off the wire
 */
struct RawMessage
{
  uint8_t header[HeaderSize];
  int32_t rots[NUM_FIELDS];
  int32_t accels[NUM_FIELDS];
  uint8_t status;
  uint8_t seq;
  int16_t temp;
  uint32_t crc;
};

#pragma pack(pop)

/**
 * Usable form of KVH message. Importantly, data is in valid units.
 */
class Message
{
public:
  Message();
  Message(const RawMessage& raw, uint32_t secs, uint32_t nsecs,
    bool is_c, bool is_da);
  ~Message();

  bool from_raw(const RawMessage& raw, uint32_t secs, uint32_t nsecs,
    bool is_c, bool is_da);

  float gyro_x() const;
  float gyro_y() const;
  float gyro_z() const;
  float accel_x() const;
  float accel_y() const;
  float accel_z() const;

  int16_t temp() const;
  int16_t temp(bool& is_c) const;

  void time(uint32_t& secs, uint32_t& nsecs) const;
  uint8_t sequence_number() const;

  bool valid() const;
  bool valid_gyro_x() const;
  bool valid_gyro_y() const;
  bool valid_gyro_z() const;

  bool valid_accel_x() const;
  bool valid_accel_y() const;
  bool valid_accel_z() const;
  bool is_celsius() const;
  bool is_delta_angle() const;

  void to_celsius();
  void to_farenheit();
protected:
  float _ang_vel[NUM_FIELDS];
  float _lin_accel[NUM_FIELDS];
  uint32_t _secs;
  uint32_t _nsecs;
  int16_t _temp;
  uint8_t _status;
  uint8_t _seq;
  bool _is_da;
  bool _is_c;
};

int16_t to_c(int16_t temp);
int16_t to_f(int16_t temp);

bool valid_checksum(const RawMessage& msg);
uint32_t compute_checksum(const char* buff, size_t len);

}

#endif