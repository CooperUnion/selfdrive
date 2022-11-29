/**
 * Built In Test Message from KVH 1750
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 02/21/2015
 */
#ifndef _KVH1750_BIT_MESSAGE_H_
#define _KVH1750_BIT_MESSAGE_H_

#include <vector>
#include <cstring>

namespace kvh
{

//! Number of status bytes in the message
const size_t NumStatusBytes = 6;
//! Number of bits used in each byte
const size_t BitsUsedPerByte = 7;

#pragma pack(push, 1)

/**
 * Raw format of the KVH Built In Test message, used for
 * accessing straight from a byte array.
 */
struct BITRawMessage
{
  char header[4];
  char status[NumStatusBytes];
  char checksum;
};

#pragma pack(pop)

/**
 * Higher level form of BIT message
 */
class BITMessage
{
public:
  BITMessage();
  BITMessage(const BITRawMessage& raw);
  ~BITMessage();

  bool from_raw(const BITRawMessage& raw);

  //gyro
  bool gyro_x_sld() const;
  bool gyro_x_moddac() const;
  bool gyro_x_phase() const;
  bool gyro_x_flash() const;
  bool gyro_x_pzt_temp() const;
  bool gyro_x_sld_temp() const;
  bool gyro_y_sld() const;
  bool gyro_y_moddac() const;
  bool gyro_y_phase() const;
  bool gyro_y_flash() const;
  bool gyro_y_pzt_temp() const;
  bool gyro_y_sld_temp() const;
  bool gyro_z_sld() const;
  bool gyro_z_moddac() const;
  bool gyro_z_phase() const;
  bool gyro_z_flash() const;
  bool gyro_z_pzt_temp() const;
  bool gyro_z_sld_temp() const;

  //accelerometers
  bool accel_x() const;
  bool accel_x_temp() const;
  bool accel_y() const;
  bool accel_y_temp() const;
  bool accel_z() const;
  bool accel_z_temp() const;

  //other
  bool gcb_temp() const;
  bool imu_temp() const;
  bool gcb_dsp_flash() const;
  bool gcb_fpga_flash() const;
  bool imu_dsp_flash() const;
  bool imu_fpga_flash() const;
  bool gcb_1_2() const;
  bool gcb_3_3() const;
  bool gcb_5() const;
  bool imu_1_2() const;
  bool imu_3_3() const;
  bool imu_5() const;
  bool imu_15() const;
  bool gcb_fpga() const;
  bool imu_fpga() const;
  bool hispeed_sport() const;
  bool aux_sport() const;
  bool software() const;
private:
  std::vector<bool> _status;
};

}

#endif