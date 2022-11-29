/**
 * KVH 1750 Base Interface
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 02/15/2015
 * Copyright 2015, Virginia Tech. All Rights Reserved.
 */
#include "kvh1750/imu.h"

#include <algorithm>

#include <sstream>

namespace kvh
{

namespace
{
  const std::string RFCmd = "ROTFMT";
  const std::string DRCmd = "DR";
  const std::string TUCmd = "TEMPUNITS";
  const std::string ConfigCmd = "CONFIG";
}

const std::vector<int> IMU1750::DataRates {1, 5, 10, 25, 50, 100, 250, 500, 750, 1000};

/**
 * Default constructor
 */
IMU1750::IMU1750(std::shared_ptr<IOModule> mod)
  :_io(mod),
  _buff(sizeof(kvh::RawMessage), 0),
  _bytes_read(0),
  _rate(IMU1750::DataRates.back()),
  _max_temp(kvh::MaxTemp_C),
  _is_config(true),
  _is_c(true),
  _is_da(true)
{
  set_mode(false);
  query_temp_units(_is_c);
  query_angle_units(_is_da);
  query_data_rate(_rate);
}

/**
 * Default destructor
 */
IMU1750::~IMU1750()
{
}

/**
 * Sets temperature limit over which over temp is reported
 */
void IMU1750::set_temp_limit(int max_temp)
{
  _max_temp = max_temp;
}

/**
 * Public read interface, which attempts to read a new KVH message
 * \param[in] msg Storage location for new message.
 * \param[out] size_t Flag indicating read success.
 */
typename IMU1750::ParseResults IMU1750::read(kvh::Message& msg)
{
  if(!base_read())
  {
    return BAD_READ;
  }
  std::vector<uint8_t>::iterator match = _buff.end();
  bool header_start = find_header(true, match);

  IMU1750::ParseResults result = VALID;

  if(header_start && _bytes_read >= sizeof(kvh::RawMessage))
  {
    //get timestamp from reading
    uint32_t secs = 0;
    uint32_t nsecs = 0;
    _io->time(secs, nsecs);
    kvh::RawMessage* raw = reinterpret_cast<kvh::RawMessage*>(&_buff.front());
    result = (msg.from_raw(*raw, secs, nsecs, _is_c, _is_da) ? VALID : BAD_CRC);
    //cleanup from successful read
    reset_buffer();

    bool celsius;
    int temp = msg.temp(celsius);
    if(!celsius)
    {
      kvh::to_c(temp);
    }
    if(temp >= _max_temp)
    {
      result = OVER_TEMP;
    }
  }
  else
  {
    if(match != _buff.begin())
    {
      reset_partial_buffer(match);
    }
    result = PARTIAL_READ;
  }

  return result;
}

/**
 * Reports last read IMU read rate for general use. This does not
 * stop data from the IMU.
 * \param[out] Rate in Hz
 */
int IMU1750::imu_rate() const
{
  return _rate;
}

/**
 * Reports last read temperature units from IMU. This does not stop
 * data from the IMU.
 * \param[out] Flag indicating if temperatures are in Celsius
 */
bool IMU1750::is_celsius() const
{
  return _is_c;
}

/**
 * Queries the IMU to determine the temperature units.
 * \param[in,out] is_celsius Flag set to indicate if temperature is in C
 * \param[out] Flag indicating if the query came back valid
 */
bool IMU1750::query_temp_units(bool& celsius)
{
  std::string cmd = build_command(TUCmd, "", true);
  set_mode(true);
  cmd_write(cmd);

  bool status = cmd_read();
  if(status)
  {
    std::vector<uint8_t>::iterator match;
    status = find_response(TUCmd, match) && parse_temp_units(match);
    if(status)
    {
      celsius = _is_c;
    }
  }

  set_mode(false);
  return status;
}

/**
 * Queries IMU to determine the rotational units.
 * \param[in,out] Flag indicating if angles are raw (delta angles) or filtered
 * to be an angular rate
 * \param[out] Flag indicating if query was processed.
 */
bool IMU1750::query_angle_units(bool& is_da)
{
  std::string cmd = build_command(RFCmd, "", true);
  set_mode(true);
  cmd_write(cmd);

  bool status = cmd_read();
  if(status)
  {
    std::vector<uint8_t>::iterator match;
    status = find_response(RFCmd, match);
    if(status)
    {
      status = parse_angle_units(match);
    }
  }

  set_mode(false);
  return status;
}

/**
 * Query IMU for current reporting data rate.
 * \param[in,out] rate_hz Reporting rate in Hz.
 * \param[out] Flag indicating if query succeeded.
 */
bool IMU1750::query_data_rate(int& rate_hz)
{
  std::string cmd = build_command(DRCmd, "", true);
  set_mode(true);
  cmd_write(cmd);
  bool status = cmd_read();
  if(status)
  {
    std::vector<uint8_t>::iterator match;
    status = find_response(DRCmd, match) &&
      parse_data_rate(match, rate_hz);
  }
  set_mode(false);
  return status;
}

/**
 * Set temperature units (C or F)
 * \param[in] Flag indicating if temperature is in C
 * \param[out] Flag indicating if command succeeded
 */
bool IMU1750::set_temp_units(bool celsius)
{
  const std::string val = (celsius ? "C" : "F");
  std::string cmd = build_command(TUCmd, val, false);
  set_mode(true);
  cmd_write(cmd);

  bool status = cmd_read();
  if(status)
  {
    std::vector<uint8_t>::iterator match;
    status = find_response(TUCmd, match) && parse_temp_units(match);
  }
  set_mode(false);
  return status;
}

/**
 * Set reporting data rate for IMU.
 * \param[in] Rate in Hz. Must be one of the options in IMU1750::DataRates
 * \param[out] Flag indicating if command succeeded
 */
bool IMU1750::set_data_rate(int rate_hz)
{
  std::vector<int>::const_iterator dr_match = std::find(IMU1750::DataRates.begin(),
    IMU1750::DataRates.end(), rate_hz);
  if(dr_match == IMU1750::DataRates.end())
  {
    return false;
  }

  const std::string val = std::to_string(rate_hz);
  std::string cmd = build_command(DRCmd, val, false);

  set_mode(true);
  cmd_write(cmd);

  std::vector<uint8_t>::iterator match;
  bool status = cmd_read() && find_response(DRCmd, match);

  set_mode(false);
  return status;
}

/**
 * Set angle units for IMU.
 * \param[in] Flag indicating if units are raw (delta angles) or not
 * \param[out] Flag indicating if command succeeded.
 */
bool IMU1750::set_angle_units(bool is_da)
{
  const std::string val = (is_da ? "DELTA" : "RATE");
  std::string cmd = build_command(RFCmd, val, false);
  set_mode(true);
  cmd_write(cmd);

  bool status = cmd_read();
  if(status)
  {
    std::vector<uint8_t>::iterator match;
    status = find_response(RFCmd, match);
    if(status)
    {
      status = parse_angle_units(match);
    }
  }
  set_mode(false);
  return status;
}

/**
 * Sets the IMU mode to either normal or config.
 * \param[in] Flag indicating if IMU should enter config mode.
 */
void IMU1750::set_mode(bool enter_config)
{
  if(enter_config == _is_config)
  {
    return;
  }

  std::string val = (enter_config ? "1" : "0");
  std::string cmd = build_command(ConfigCmd, val, false);
  cmd_write(cmd);

  //normal mode never responds with confirmation
  if(enter_config)
  {
    reset_buffer();
    cmd_read();
  }

  _is_config = enter_config;
}

/**
 * Parse buffer for angle units.
 * \param[in] Iterator pointing at start of response parameters
 * \param[out] Flag indicating if response was correctly parsed
 */
bool IMU1750::parse_angle_units(std::vector<uint8_t>::iterator match)
{
  bool status = false;
  const std::string Delta = "DELTA";
  const std::string Rate = "RATE";
  if(std::equal(Delta.begin(), Delta.end(), match))
  {
    _is_da = true;
    status = true;
  }
  else if(std::equal(Rate.begin(), Rate.end(), match))
  {
    _is_da = false;
    status = true;
  }

  return status;
}

/**
 * Parse buffer for temperature units
 * \param[in] Iterator pointing at start of response parameters
 * \param[out] Flag indicating if response was correctly parsed
 */
bool IMU1750::parse_temp_units(std::vector<uint8_t>::iterator match)
{
  bool status = true;
  switch(*match)
  {
    case 'C':
      _is_c = true;
      break;
    case 'F':
      _is_c = false;
      break;
    default:
      status = false;
      break;
  }

  return status;
}

/**
 * Parse buffer for data rate
 * \param[in] Iterator pointing at start of response parameters
 * \param[in,out] Rate value parsed
 * \param[out] Flag indicating if response was correctly parsed
 */
bool IMU1750::parse_data_rate(std::vector<uint8_t>::iterator match, int& rate)
{
  rate = std::strtol(reinterpret_cast<char*>(&*match), NULL, 10);
  return true;
}

/**
 * Internal read function which performs read and maintains internal state.
 * \param[in] use_tov Flag indicating if the timing signal should be used.
 * \param[out] Flag indicating if read was valid.
 */
bool IMU1750::base_read(bool use_tov)
{
  size_t bytes = 0;
  bool status = _io->read(&_buff.front() + _bytes_read, bytes_remaining(), bytes, use_tov);
  if(!status)
  {
    reset_buffer();
  }
  else
  {
    _bytes_read += bytes;
  }
  return status;
}

/**
 * Attempt to read response from command.
 * \param[out] Flag indicating if buffer now contains the response
 */
bool IMU1750::cmd_read()
{
  const size_t MaxReadAttempts = 20;

  bool response = false;
  for(size_t ii = 0; !response && ii < MaxReadAttempts; ++ii)
  {
    base_read(false);

    if(_buff[_bytes_read - 1] == '\n')
    {
      response = true;
    }
    else if(bytes_remaining() == 0)
    {
      reset_buffer();
    }
  }

  return response;
}

/**
 * Write command to IMU.
 */
bool IMU1750::cmd_write(const std::string& cmd)
{
  _io->flush_buffers();
  reset_buffer();
  return _io->write(reinterpret_cast<const uint8_t*>(cmd.c_str()), cmd.length());
}

/**
 * Function to search the memory buffer for the header sequence.
 * \param[in,out] iterator pointing to location of header
 * \param[out] Flag indicating if header is at the start of the header.
 */
bool IMU1750::find_header(bool is_imu, std::vector<uint8_t>::iterator& match)
{
  HeaderType::const_iterator header_start = (is_imu ? IMUHeader.begin() : BITHeader.begin());
  HeaderType::const_iterator header_end = (is_imu ? IMUHeader.end() : BITHeader.end());
  match = std::search(_buff.begin(), _buff.end(), header_start, header_end);

  return match == _buff.begin();
}

/**
 * Clears and resets the memory buffer used to store the raw message being
 * processed.
 */
void IMU1750::reset_buffer()
{
  _buff.clear();
  _buff.resize(sizeof(kvh::RawMessage), 0);
  _bytes_read = 0;
  _io->reset_time();
}

/**
 * Stores a partially read message at the start of the buffer
 * to simplify processing.
 * \param[in] match Iterator pointing to start of header.
 */
void IMU1750::reset_partial_buffer(const std::vector<uint8_t>::iterator& match)
{
  std::vector<uint8_t>::iterator buffer_end = _buff.begin() + _bytes_read;
  //start at either the match, or the longest missable substring
  std::vector<uint8_t>::iterator start = std::min(match, std::max(_buff.begin(),
    buffer_end - (HeaderSize - 1)));
  _backup_buff.clear();
  _backup_buff.insert(_backup_buff.end(), start, buffer_end);
  _backup_buff.resize(sizeof(kvh::RawMessage), 0); //force size to be the same
  _bytes_read = buffer_end - start;
  _buff.swap(_backup_buff);
}

/**
 * Number of bytes available for writing in the buffer.
 */
size_t IMU1750::bytes_remaining() const
{
  return _buff.size() - _bytes_read;
}

/**
 * Sets the buffer size in case larger messages need to be read.
 * \param[in] len Length of internal buffer in bytes
 */
void IMU1750::set_buffer_size(size_t len)
{
  std::vector<uint8_t>::reverse_iterator start = std::max(_buff.rend(),
    _buff.rbegin() + len);
  std::vector<uint8_t> new_buffer(start, _buff.rend()); //copy last len bytes
  new_buffer.resize(len, 0); //make sure size is correct
  _buff.swap(new_buffer);
}

/**
 * Utility function for building an IMU commands in a consistent fashion.
 * \param[in] Command type.
 * \param[in] Command arguments, if any are needed.
 * \param[in] Flag indicating if command is a query or not.
 * \param[out] String containing the fully built command
 */
std::string IMU1750::build_command(const std::string& type, const std::string& val,
  bool is_query)
{
  std::stringstream ss;
  ss << (is_query ? "?" : "=") << type;
  if(!is_query)
  {
    ss << "," << val;
  }
  ss << "\n";
  return ss.str();
}

/**
 * Utility function to attempt to find a valid response in the internal buffer.
 * \param[in] Command type to search for
 * \param[in,out] Iterator pointing at start of response arguments
 * \pram[out] Flag indicating if response was found
 */
bool IMU1750::find_response(const std::string& type,
  std::vector<uint8_t>::iterator& match)
{
  std::string FullMatch = type + ",";
  match = std::search(_buff.begin(), _buff.end(),
    FullMatch.begin(), FullMatch.end());

  bool status = (match != _buff.end());
  match += FullMatch.length();
  return status;
}

}