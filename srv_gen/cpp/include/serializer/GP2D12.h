/* Auto-generated by genmsg_cpp for file /home/patrick/Eclipse/ros/pi-robot-ros-stack/serializer/srv/GP2D12.srv */
#ifndef SERIALIZER_SERVICE_GP2D12_H
#define SERIALIZER_SERVICE_GP2D12_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "ros/service_traits.h"




namespace serializer
{
template <class ContainerAllocator>
struct GP2D12Request_ : public ros::Message
{
  typedef GP2D12Request_<ContainerAllocator> Type;

  GP2D12Request_()
  : pin(0)
  , cached(false)
  {
  }

  GP2D12Request_(const ContainerAllocator& _alloc)
  : pin(0)
  , cached(false)
  {
  }

  typedef uint8_t _pin_type;
  uint8_t pin;

  typedef uint8_t _cached_type;
  uint8_t cached;


private:
  static const char* __s_getDataType_() { return "serializer/GP2D12Request"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "53bed398bc7ef4510938f1f4d6a12a22"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "4e943b13b25d974978eb5df74257558f"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "uint8 pin\n\
bool cached\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, pin);
    ros::serialization::serialize(stream, cached);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, pin);
    ros::serialization::deserialize(stream, cached);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(pin);
    size += ros::serialization::serializationLength(cached);
    return size;
  }

  typedef boost::shared_ptr< ::serializer::GP2D12Request_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serializer::GP2D12Request_<ContainerAllocator>  const> ConstPtr;
}; // struct GP2D12Request
typedef  ::serializer::GP2D12Request_<std::allocator<void> > GP2D12Request;

typedef boost::shared_ptr< ::serializer::GP2D12Request> GP2D12RequestPtr;
typedef boost::shared_ptr< ::serializer::GP2D12Request const> GP2D12RequestConstPtr;


template <class ContainerAllocator>
struct GP2D12Response_ : public ros::Message
{
  typedef GP2D12Response_<ContainerAllocator> Type;

  GP2D12Response_()
  : value(0)
  {
  }

  GP2D12Response_(const ContainerAllocator& _alloc)
  : value(0)
  {
  }

  typedef uint8_t _value_type;
  uint8_t value;


private:
  static const char* __s_getDataType_() { return "serializer/GP2D12Response"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "e4da51e86d3bac963ee2bb1f41031407"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "4e943b13b25d974978eb5df74257558f"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "uint8 value\n\
\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, value);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, value);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(value);
    return size;
  }

  typedef boost::shared_ptr< ::serializer::GP2D12Response_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serializer::GP2D12Response_<ContainerAllocator>  const> ConstPtr;
}; // struct GP2D12Response
typedef  ::serializer::GP2D12Response_<std::allocator<void> > GP2D12Response;

typedef boost::shared_ptr< ::serializer::GP2D12Response> GP2D12ResponsePtr;
typedef boost::shared_ptr< ::serializer::GP2D12Response const> GP2D12ResponseConstPtr;

struct GP2D12
{

typedef GP2D12Request Request;
typedef GP2D12Response Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct GP2D12
} // namespace serializer

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::serializer::GP2D12Request_<ContainerAllocator> > {
  static const char* value() 
  {
    return "53bed398bc7ef4510938f1f4d6a12a22";
  }

  static const char* value(const  ::serializer::GP2D12Request_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x53bed398bc7ef451ULL;
  static const uint64_t static_value2 = 0x0938f1f4d6a12a22ULL;
};

template<class ContainerAllocator>
struct DataType< ::serializer::GP2D12Request_<ContainerAllocator> > {
  static const char* value() 
  {
    return "serializer/GP2D12Request";
  }

  static const char* value(const  ::serializer::GP2D12Request_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::serializer::GP2D12Request_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 pin\n\
bool cached\n\
\n\
";
  }

  static const char* value(const  ::serializer::GP2D12Request_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::serializer::GP2D12Request_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::serializer::GP2D12Response_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e4da51e86d3bac963ee2bb1f41031407";
  }

  static const char* value(const  ::serializer::GP2D12Response_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe4da51e86d3bac96ULL;
  static const uint64_t static_value2 = 0x3ee2bb1f41031407ULL;
};

template<class ContainerAllocator>
struct DataType< ::serializer::GP2D12Response_<ContainerAllocator> > {
  static const char* value() 
  {
    return "serializer/GP2D12Response";
  }

  static const char* value(const  ::serializer::GP2D12Response_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::serializer::GP2D12Response_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 value\n\
\n\
\n\
";
  }

  static const char* value(const  ::serializer::GP2D12Response_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::serializer::GP2D12Response_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::serializer::GP2D12Request_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.pin);
    stream.next(m.cached);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GP2D12Request_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::serializer::GP2D12Response_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.value);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GP2D12Response_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<serializer::GP2D12> {
  static const char* value() 
  {
    return "4e943b13b25d974978eb5df74257558f";
  }

  static const char* value(const serializer::GP2D12&) { return value(); } 
};

template<>
struct DataType<serializer::GP2D12> {
  static const char* value() 
  {
    return "serializer/GP2D12";
  }

  static const char* value(const serializer::GP2D12&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<serializer::GP2D12Request_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4e943b13b25d974978eb5df74257558f";
  }

  static const char* value(const serializer::GP2D12Request_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<serializer::GP2D12Request_<ContainerAllocator> > {
  static const char* value() 
  {
    return "serializer/GP2D12";
  }

  static const char* value(const serializer::GP2D12Request_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<serializer::GP2D12Response_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4e943b13b25d974978eb5df74257558f";
  }

  static const char* value(const serializer::GP2D12Response_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<serializer::GP2D12Response_<ContainerAllocator> > {
  static const char* value() 
  {
    return "serializer/GP2D12";
  }

  static const char* value(const serializer::GP2D12Response_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // SERIALIZER_SERVICE_GP2D12_H

