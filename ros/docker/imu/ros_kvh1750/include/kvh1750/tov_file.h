/**
 * Interfacing with KVH 1750 over RSS-422.
 * \author Jason Ziglar <jpz@vt.edu>, based on code form Eric L. Hahn <erichahn@vt.edu>
 * \date 02/15/2015
 * Copyright 2015, Virginia Tech. All Rights Reserved.
 */
#ifndef __KVH_1750_IMU_h__
#define __KVH_1750_IMU_h__

#include "imu.h"

#include <serial/serial.h>

#include <chrono>

#include <vector>

#include <memory>

namespace kvh
{

/**
 * Interface to a KVH1750 over RS-422, with support for the TOV signal
 * via a file descriptor.
 */
class TOVFile : public IOModule
{
public:
  TOVFile(const std::string& addr = "/dev/ttyS4", uint32_t baud = 921600,
    uint32_t tm = 1, const std::string& tov_addr = "");
  virtual ~TOVFile();

  virtual bool read(uint8_t* buff, size_t max_bytes, size_t& bytes, bool tov);
  virtual bool write(const uint8_t* buff, size_t bytes);
  virtual void flush_buffers();
  virtual void time(uint32_t& secs, uint32_t& nsecs);
  virtual void reset_time();

protected:
  typedef std::chrono::high_resolution_clock::duration duration_t;
  std::shared_ptr<serial::Serial> _data;
  std::shared_ptr<serial::Serial> _tov;
  duration_t _tm; //! Time when message was read
  bool _valid_tm; //! Flag indicating if time is valid
};

};

#endif