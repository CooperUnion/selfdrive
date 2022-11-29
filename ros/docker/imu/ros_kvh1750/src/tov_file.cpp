/*
 * KVH 1750 IMU
 * Eric L. Hahn <erichahn@vt.edu>
 * 12/5/2014
 * Copyright 2014. All Rights Reserved.
 */
#include "kvh1750/tov_file.h"

#include <algorithm>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <byteswap.h>

namespace kvh
{

/**
 * Default constructor.
 * \param[in] addr File location to read data from
 * \param[in] tm Number of milliseconds to wait. -1 blocks, 0 is nonblock
 */
TOVFile::TOVFile(const std::string& addr, uint32_t baud, uint32_t tm,
  const std::string& tov_addr)
  :IOModule(),
  _data(std::make_shared<serial::Serial>(addr, baud, serial::Timeout::simpleTimeout(tm))),
  _tov(),
  _tm(),
  _valid_tm(false)
{
  if(!tov_addr.empty())
  {
    _tov = std::make_shared<serial::Serial>(tov_addr, baud,
      serial::Timeout::simpleTimeout(tm));
  }
}

/**
 * Default destructor
 */
TOVFile::~TOVFile()
{
}

/**
 * Low level read interface for A POSIX style file descriptor.
 * \param[in,out] buff Buffer to store data
 * \param[in,out] bytes Number of bytes read
 * \param[out] Flag indicating if read was successful
 */
bool TOVFile::read(uint8_t* buff, size_t max_bytes, size_t& bytes, bool use_tov)
{
  duration_t wait;
  if(_tov && use_tov)
  {
    std::vector<uint8_t> wait_buff(1, 0);
    this->flush_buffers();
    size_t bytes_read = _tov->read(wait_buff, wait_buff.size());
    wait = std::chrono::high_resolution_clock::now().time_since_epoch();
    if(bytes_read == 0)
    {
      return false;
    }
  }
  else
  {
    if(!_data->waitReadable())
    {
      return false;
    }
    wait = std::chrono::high_resolution_clock::now().time_since_epoch();
  }

  //only update time if at start of new message
  if(!_valid_tm)
  {
    _tm = wait;
    _valid_tm = true;
  }

  bytes = _data->read(buff, max_bytes);

  return true;
}

/**
 * Writes data to the device.
 * \param[out] Flag indicating if bytes are written
 */
bool TOVFile::write(const uint8_t* buff, size_t bytes)
{
  size_t res = _data->write(buff, bytes);
  return (res == bytes);
}

/**
 * Flush buffers to clear up any cached data
 */
void TOVFile::flush_buffers()
{
  _data->flush();
  if(_tov)
  {
    _tov->flush();
  }
}

/**
 * Reports time when current message was read.
 * \param[in,out] secs Seconds since epoch
 * \param[in,out] nsecs Nanoseconds since the Seconds field
 */
void TOVFile::time(uint32_t& secs, uint32_t& nsecs)
{
  //TODO: Unhack this disaster
  secs = _tm.count() / 1e9;
  nsecs = _tm.count() - secs * 1e9;
}

/**
 * Resets time to set for next read.
 */
void TOVFile::reset_time()
{
  _valid_tm = false;
  _tm = std::chrono::high_resolution_clock::duration::zero();
}

}
