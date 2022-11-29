/**
 * Unit testing KVH Interfaces.
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 02/19/2015
 */
#include "kvh1750/imu.h"

#include <gtest/gtest.h>

#include <boost/algorithm/string.hpp>

#include <chrono>
#include <memory>

#include <map>

//Make C++11x support not required
#if __cplusplus >= 201103L
#include <functional>

#define FUNCTION std::function
#define BIND std::bind
#else
#include <tr1/functional>

#define FUNCTION std::tr1::function
#define BIND std::tr1::bind

#endif

typedef std::vector<uint8_t> ByteBuffer;
typedef std::map<std::string, ByteBuffer> TestCases;

struct DataCase
{
  ByteBuffer data;
  std::vector<size_t> reads;
};

typedef FUNCTION<std::string(const std::string& name, const std::string& args)> CommandProcessor;

/**
 * Stub class for unit testing data
 */
class IOStub : public kvh::IOModule
{
public:
  IOStub();
  virtual ~IOStub();

  bool set_test_case(const std::string& name);
  std::vector<std::string> cases() const;
protected:
  virtual bool read(uint8_t* buff, size_t max_bytes, size_t& bytes, bool tov);
  virtual bool write(const uint8_t* buff, size_t bytes);
  virtual void flush_buffers();
  virtual void time(uint64_t& secs, uint64_t& nsecs);
  virtual void reset_time();
protected:
  void add_case(const std::string& name, uint8_t* start, uint8_t* end,
    const std::vector<size_t>& reads);
  bool offset_read(const std::vector<uint8_t>& data, size_t& offset, uint8_t* buff,
    size_t max_bytes, size_t& bytes_read);

  std::string processTemperature(const std::string& name, const std::string& args);
  std::string processRotFmt(const std::string& name, const std::string& args);
  std::string processConfig(const std::string& name, const std::string& args);
protected:
  typedef std::chrono::high_resolution_clock::duration duration_t;
  std::map<std::string, DataCase> _cases;
  std::vector<uint8_t> _response_data;
  std::vector<uint8_t> _input;
  std::map<std::string, CommandProcessor> _processors;
  std::string _case;
  size_t _offset;
  size_t _response_offset;
  size_t _counter;
  duration_t _tm;
  bool _valid_tm;
  bool _is_config;
  bool _is_c;
  bool _is_da;
};

/**
 * Default constructor
 */
IOStub::IOStub() :
  _cases(),
  _response_data(),
  _input(),
  _processors(),
  _case(),
  _offset(0),
  _response_offset(0),
  _counter(0),
  _tm(),
  _valid_tm(false),
  _is_config(false),
  _is_c(true)
{
  //accelerometers only
  std::array<uint8_t, sizeof(kvh::RawMessage)> valid =
    MAKE_BYTE_ARRAY(0xFE, 0x81, 0xFF, 0x55, 0x3B, 0x04, 0x1F, 0x78, 0xB9, 0xB9,
                    0x66, 0x13, 0xBA, 0x23, 0x4B, 0x38, 0xBB, 0x7C, 0x04, 0x10,
                    0xB8, 0xDE, 0xF4, 0x80, 0x3F, 0x7F, 0x4B, 0x7B, 0x70, 0x1F,
                    0x00, 0x1D, 0xF1, 0x2F, 0xB6, 0xEE
                   );
  std::vector<size_t> reads(1, sizeof(kvh::RawMessage));
  add_case("early", valid.begin(), valid.end(), reads);
  //fully valid
  valid = MAKE_BYTE_ARRAY(0xFE, 0x81, 0xFF, 0x55, 0xB3, 0xD0, 0xBB, 0xE7, 0x32,
                          0x19, 0x3E, 0xF4, 0x30, 0xAC, 0x19, 0xA5, 0xBA, 0x8F, 0x78, 0xD6, 0x3A, 0x29,
                          0x05, 0x94, 0x3F, 0x7E, 0xCB, 0xD4, 0x77, 0x68, 0x00, 0x1D, 0xAD, 0xC5, 0xF6,
                          0x51);
  add_case("basic", valid.begin(), valid.end(), reads);

  std::vector<size_t> multi_read;
  multi_read.push_back(5);
  multi_read.push_back(valid.size() - 5);
  add_case("simple_split", valid.begin(), valid.end(), multi_read);
  //bad checksum
  valid = MAKE_BYTE_ARRAY(0xFE, 0x81, 0xFF, 0x55, 0xB4, 0x6B, 0x86, 0xC6, 0xB4,
                          0x4F, 0x5B, 0x83, 0xB4, 0x10, 0x6D, 0x09, 0xBB, 0x71, 0x3C, 0xB7, 0xBB,
                          0xAB, 0xD9, 0x14, 0x3F, 0x7E, 0x31, 0x9A, 0x77, 0x69, 0x00, 0x1D, 0x95,
                          0xBB, 0xDB, 0x4F);
  add_case("bad_crc", valid.begin(), valid.end(), reads);

  using namespace std::placeholders;
  _processors["CONFIG"] = BIND(&IOStub::processConfig, this, _1, _2);
  _processors["TEMPUNITS"] = BIND(&IOStub::processTemperature, this, _1, _2);
  _processors["ROTFMT"] = BIND(&IOStub::processRotFmt, this, _1, _2);
}

IOStub::~IOStub()
{
}

bool IOStub::set_test_case(const std::string& name)
{
  bool valid = (_cases.find(name) != _cases.end());
  if(valid)
  {
    _case = name;
    _counter = 0;
    _offset = 0;
    reset_time();
  }

  return valid;
}

std::vector<std::string> IOStub::cases() const
{
  std::vector<std::string> case_list;

  for(const auto& ii : _cases)
  {
    case_list.push_back(ii.first);
  }

  return case_list;
}

/**
 * Simplified read which draws from canned data.
 */
bool IOStub::read(uint8_t* buff, size_t max_bytes, size_t& bytes, bool tov)
{
  if(!_response_data.empty())
  {
    bool status = offset_read(_response_data, _response_offset, buff, max_bytes,
      bytes);
    if(_response_offset == _response_data.size())
    {
      _response_data.clear();
      _response_offset = 0;
    }

    return status;
  }
  const auto& match = _cases.find(_case);
  if(match == _cases.end())
  {
    return false;
  }
  const DataCase& data = match->second;
  //bounds check the read attempts
  if(_counter >= data.reads.size())
  {
    return false;
  }

  size_t read_size = std::min(max_bytes, data.reads[_counter++]);
  bool result = offset_read(data.data, _offset, buff, read_size, bytes);

  return result;
}

bool IOStub::write(const uint8_t* buff, size_t bytes)
{
  if(bytes == 0)
  {
    return true;
  }

  size_t last_index = _input.size();
  _input.insert(_input.end(), buff, buff + bytes);

  auto start = _input.begin() + last_index;
  auto match = std::find(start, _input.end(), '\n');

  if(match == _input.end())
  {
    return true;
  }

  std::string data = std::string(start, match);
  _input.clear();

  bool is_query = false;
  switch(data[0])
  {
    case '?':
      is_query = true;
    break;
    case '=':
      is_query = false;
    break;
    default:
      return false;
  }
  std::string cmd;
  std::string args;

  if(is_query)
  {
    cmd = data.substr(1);
  }
  else
  {
    size_t end = data.find(',');
    cmd = data.substr(1, end - 1);
    //apparent fencepost error here is to remove the newline
    args = data.substr(end + 1);
  }

  boost::to_upper(cmd);
  boost::to_upper(args);

  const auto& processor = _processors.find(cmd);

  std::string resp = "";
  if(processor != _processors.end())
  {
    resp = processor->second(processor->first, args);
  }

  if(!resp.empty())
  {
    _response_data.assign(resp.begin(), resp.end());
  }
  return true;
}

void IOStub::flush_buffers()
{
}

void IOStub::time(uint64_t& secs, uint64_t& nsecs)
{
  duration_t d_secs = std::chrono::duration_cast<std::chrono::seconds>(_tm);
  duration_t diff_t = _tm - d_secs;
  duration_t d_nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(diff_t);
  secs = d_secs.count();
  nsecs = d_nsecs.count();
}

void IOStub::reset_time()
{
  _valid_tm = false;
  _tm = std::chrono::high_resolution_clock::duration::zero();
}

void IOStub::add_case(const std::string& name, uint8_t* start, uint8_t* end,
  const std::vector<size_t>& reads)
{
  _cases[name].data.assign(start, end);
  _cases[name].reads = reads;
}

bool IOStub::offset_read(const std::vector<uint8_t>& data, size_t& offset, uint8_t* buff,
  size_t max_bytes, size_t& bytes_read)
{
  if(!_valid_tm)
  {
    _tm = std::chrono::high_resolution_clock::now().time_since_epoch();
    _valid_tm = true;
  }
  bytes_read = std::min(max_bytes, data.size() - offset);
  memcpy(buff, &data[offset], bytes_read);
  offset += bytes_read;

  return true;
}

std::string IOStub::processConfig(const std::string& name, const std::string& args)
{
  const std::string Response = name + ",1\n";
  if(args.empty() && _is_config)
  {
    return Response;
  }

  std::string resp = "";
  if(args == "1") //on
  {
    _is_config = true;
    resp = Response;
  }
  else if(args == "0")
  {
    _is_config = false;
  }
  else
  {
    resp = "INVALID";
  }

  return resp;
}

std::string IOStub::processTemperature(const std::string& name, const std::string& args)
{
  std::string resp;
  if(!args.empty())
  {
    if(args == "C")
    {
      _is_c = true;
    }
    else if(args == "F")
    {
      _is_c = false;
    }
    else
    {
      resp = "INVALID";
    }
  }

  if(resp.empty())
  {
    resp = name + "," + (_is_c ? "C" : "F") + "\n";
  }

  return resp;
}

std::string IOStub::processRotFmt(const std::string& name, const std::string& args)
{
  std::string resp;
  if(!args.empty())
  {
    if(args == "DELTA")
    {
      _is_da = true;
    }
    else if(args == "RATE")
    {
      _is_da = false;
    }
    else if(args == "RESET")
    {
      //TODO: Implement reset statement
    }
    else
    {
      resp = "INVALID";
    }
  }

  if(resp.empty())
  {
    resp = name + "," + (_is_da ? "DELTA" : "RATE") + "\n";
  }

  return resp;
}

namespace
{
  std::shared_ptr<IOStub> stub = std::make_shared<IOStub>();
  std::shared_ptr<kvh::IOModule> mod =
    std::dynamic_pointer_cast<kvh::IOModule>(stub);
  kvh::IMU1750 imu(mod);
}

TEST(TestSuite, earlyMessageTest)
{
  //Need to exploit polymorphism
  kvh::Message msg;
  stub->set_test_case("early");
  EXPECT_EQ(kvh::IMU1750::VALID, imu.read(msg));
  EXPECT_EQ(true, msg.valid());
  EXPECT_EQ(false, msg.valid_gyro_x());
  EXPECT_EQ(false, msg.valid_gyro_y());
  EXPECT_EQ(false, msg.valid_gyro_z());
  EXPECT_EQ(true, msg.valid_accel_x());
  EXPECT_EQ(true, msg.valid_accel_y());
  EXPECT_EQ(true, msg.valid_accel_z());
  EXPECT_EQ(31, msg.sequence_number());
  EXPECT_EQ(29, msg.temp());
  EXPECT_EQ(true, msg.is_celsius());
  //Test that C->C conversion is idempotent
  msg.to_celsius();
  EXPECT_EQ(29, msg.temp());
  EXPECT_EQ(true, msg.is_celsius());
  //test conversion to farenheit
  msg.to_farenheit();
  EXPECT_EQ(false, msg.is_celsius());
  EXPECT_EQ(84, msg.temp());
}

TEST(TestSuite, validMessageTest)
{
  kvh::Message msg;
  stub->set_test_case("basic");
  EXPECT_EQ(kvh::IMU1750::VALID, imu.read(msg));
  EXPECT_EQ(true, msg.valid());
  EXPECT_EQ(true, msg.valid_gyro_x());
  EXPECT_EQ(true, msg.valid_gyro_y());
  EXPECT_EQ(true, msg.valid_gyro_z());
  EXPECT_EQ(true, msg.valid_accel_x());
  EXPECT_EQ(true, msg.valid_accel_y());
  EXPECT_EQ(true, msg.valid_accel_z());
  EXPECT_EQ(104, msg.sequence_number());
  EXPECT_EQ(29, msg.temp());
  EXPECT_EQ(true, msg.is_celsius());
}

TEST(TestSuite, simpleSplitTest)
{
  kvh::Message msg;
  stub->set_test_case("simple_split");
  EXPECT_EQ(kvh::IMU1750::PARTIAL_READ, imu.read(msg));
  EXPECT_EQ(kvh::IMU1750::VALID, imu.read(msg));
  EXPECT_EQ(true, msg.valid());
  EXPECT_EQ(true, msg.valid_gyro_x());
  EXPECT_EQ(true, msg.valid_gyro_y());
  EXPECT_EQ(true, msg.valid_gyro_z());
  EXPECT_EQ(true, msg.valid_accel_x());
  EXPECT_EQ(true, msg.valid_accel_y());
  EXPECT_EQ(true, msg.valid_accel_z());
  EXPECT_EQ(104, msg.sequence_number());
  EXPECT_EQ(29, msg.temp());
  EXPECT_EQ(true, msg.is_celsius());
}

TEST(TestSuite, invalidMessageTest)
{
  kvh::Message msg;
  stub->set_test_case("bad_crc");
  EXPECT_EQ(kvh::IMU1750::BAD_CRC, imu.read(msg));
  EXPECT_EQ(false, msg.valid());
}

TEST(TestSuite, commandTests)
{
  //note: no test case sent, since the test_case doesn't
  //impact command sequences
  EXPECT_EQ(true, imu.set_temp_units(false));
  bool is_c = true;
  imu.query_temp_units(is_c);
  EXPECT_EQ(false, is_c);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}