/**
 * Plugin interface for adding custom processing for KVH messages.
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 09/29/2015
 */
#ifndef _KVH_PLUGIN_H_
#define _KVH_PLUGIN_H_

#include "kvh1750/types.h"

namespace kvh
{

class MessageProcessorBase
{
public:
  virtual ~MessageProcessorBase() {};

  virtual void process_message(const kvh::Message& msg) = 0;
  virtual void set_link_name(const std::string& link) = 0;
protected:
  MessageProcessorBase() {};
};

}

#endif
