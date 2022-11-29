/**
 * Module defining IO interface to an IMU.
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 02/22/2015
 */
#ifndef _IOMODULE_h_
#define _IOMODULE_h_

#include <cstring>
#include <stdint.h>

namespace kvh
{

/**
 * Interface to IO for reading IMU data.
 */
class IOModule
{
public:
  IOModule();
  virtual ~IOModule();

  virtual bool read(uint8_t* buff, size_t max_bytes, size_t& bytes, bool tov = true) = 0;
  virtual bool write(const uint8_t* buff, size_t bytes) = 0;
  virtual void flush_buffers() = 0;
  virtual void time(uint32_t& secs, uint32_t& nsecs) = 0;
  virtual void reset_time() = 0;
};

}

#endif
