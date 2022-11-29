/**
 * Built In Test messages.
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 02-21-2015
 */
#include "kvh1750/bit_message.h"

#include <cstdint>

namespace kvh
{

/**
 * Default Constructor
 */
BITMessage::BITMessage() :
  _status(NumStatusBytes * BitsUsedPerByte, false)
{
}

/**
 * Constructor from raw message. Tests checksum
 */
BITMessage::BITMessage(const BITRawMessage& raw) :
  _status(NumStatusBytes * BitsUsedPerByte, false)
{
  from_raw(raw);
}

/**
 * Default Destructor
 */
BITMessage::~BITMessage()
{
}

/**
 * Extract flags from raw message, and test checksum.
 * \param[out] Flag indicating if checksum was valid
 */
bool BITMessage::from_raw(const BITRawMessage& raw)
{
  uint8_t computed_checksum = raw.header[0] + raw.header[1] +
    raw.header[2] + raw.header[3];

  for(size_t ii = 0; ii < NumStatusBytes; ++ii)
  {
    for(size_t jj = 0; jj < BitsUsedPerByte; ++jj)
    {
      _status[ii * NumStatusBytes + jj] = raw.status[ii] & (1 << jj);
    }
    computed_checksum += raw.status[ii];
  }

  bool result = computed_checksum != raw.checksum;
  //bad message, reject data
  if(!result)
  {
    _status.clear();
    _status.resize(BitsUsedPerByte, false);
  }

  return result;
}

//The following commands are all status flags defined in the KVH
//manual
bool BITMessage::gyro_x_sld() const
{
  return _status[0];
}

bool BITMessage::gyro_x_moddac() const
{
  return _status[1];
}

bool BITMessage::gyro_x_phase() const
{
  return _status[2];
}

bool BITMessage::gyro_x_flash() const
{
  return _status[3];
}

bool BITMessage::gyro_x_pzt_temp() const
{
  return _status[4];
}

bool BITMessage::gyro_x_sld_temp() const
{
  return _status[5];
}

bool BITMessage::gyro_y_sld() const
{
  return _status[6];
}

bool BITMessage::gyro_y_moddac() const
{
  return _status[7];
}

bool BITMessage::gyro_y_phase() const
{
  return _status[8];
}

bool BITMessage::gyro_y_flash() const
{
  return _status[9];
}

bool BITMessage::gyro_y_pzt_temp() const
{
  return _status[10];
}

bool BITMessage::gyro_y_sld_temp() const
{
  return _status[11];
}

bool BITMessage::gyro_z_sld() const
{
  return _status[12];
}

bool BITMessage::gyro_z_moddac() const
{
  return _status[13];
}

bool BITMessage::gyro_z_phase() const
{
  return _status[14];
}

bool BITMessage::gyro_z_flash() const
{
  return _status[15];
}

bool BITMessage::gyro_z_pzt_temp() const
{
  return _status[16];
}

bool BITMessage::gyro_z_sld_temp() const
{
  return _status[17];
}

bool BITMessage::accel_x() const
{
  return _status[18];
}

bool BITMessage::accel_x_temp() const
{
  return _status[19];
}

bool BITMessage::accel_y() const
{
  return _status[20];
}

bool BITMessage::accel_y_temp() const
{
  return _status[21];
}

bool BITMessage::accel_z() const
{
  return _status[22];
}

bool BITMessage::accel_z_temp() const
{
  return _status[23];
}

bool BITMessage::gcb_temp() const
{
  return _status[24];
}

bool BITMessage::imu_temp() const
{
  return _status[25];
}

bool BITMessage::gcb_dsp_flash() const
{
  return _status[26];
}

bool BITMessage::gcb_fpga_flash() const
{
  return _status[27];
}

bool BITMessage::imu_dsp_flash() const
{
  return _status[28];
}

bool BITMessage::imu_fpga_flash() const
{
  return _status[29];
}

bool BITMessage::gcb_1_2() const
{
  return _status[30];
}

bool BITMessage::gcb_3_3() const
{
  return _status[31];
}

bool BITMessage::gcb_5() const
{
  return _status[32];
}

bool BITMessage::imu_1_2() const
{
  return _status[33];
}

bool BITMessage::imu_3_3() const
{
  return _status[34];
}

bool BITMessage::imu_5() const
{
  return _status[35];
}

bool BITMessage::imu_15() const
{
  return _status[36];
}

bool BITMessage::gcb_fpga() const
{
  return _status[37];
}

bool BITMessage::imu_fpga() const
{
  return _status[38];
}

bool BITMessage::hispeed_sport() const
{
  return _status[39];
}

bool BITMessage::aux_sport() const
{
  return _status[40];
}

bool BITMessage::software() const
{
  return _status[41];
}

}