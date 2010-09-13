/* Auto-generated by genmsg_cpp for file /home/patrick/Eclipse/serializer-ros-pkg/serializer/srv/IsMoving.srv */
#ifndef SERIALIZER_SERVICE_ISMOVING_H
#define SERIALIZER_SERVICE_ISMOVING_H
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
struct IsMovingRequest_ : public ros::Message
{
  typedef IsMovingRequest_<ContainerAllocator> Type;

  IsMovingRequest_()
  {
  }

  IsMovingRequest_(const ContainerAllocator& _alloc)
  {
  }


private:
  static const char* __s_getDataType_() { return "serializer/IsMovingRequest"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "d41d8cd98f00b204e9800998ecf8427e"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "9104f1a32b4fbf4d3c8c80d9b9493250"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    return size;
  }

  typedef boost::shared_ptr< ::serializer::IsMovingRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serializer::IsMovingRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct IsMovingRequest
typedef  ::serializer::IsMovingRequest_<std::allocator<void> > IsMovingRequest;

typedef boost::shared_ptr< ::serializer::IsMovingRequest> IsMovingRequestPtr;
typedef boost::shared_ptr< ::serializer::IsMovingRequest const> IsMovingRequestConstPtr;


template <class ContainerAllocator>
struct IsMovingResponse_ : public ros::Message
{
  typedef IsMovingResponse_<ContainerAllocator> Type;

  IsMovingResponse_()
  : moving(false)
  {
  }

  IsMovingResponse_(const ContainerAllocator& _alloc)
  : moving(false)
  {
  }

  typedef uint8_t _moving_type;
  uint8_t moving;


private:
  static const char* __s_getDataType_() { return "serializer/IsMovingResponse"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "9104f1a32b4fbf4d3c8c80d9b9493250"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "9104f1a32b4fbf4d3c8c80d9b9493250"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "bool moving\n\
\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, moving);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, moving);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(moving);
    return size;
  }

  typedef boost::shared_ptr< ::serializer::IsMovingResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serializer::IsMovingResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct IsMovingResponse
typedef  ::serializer::IsMovingResponse_<std::allocator<void> > IsMovingResponse;

typedef boost::shared_ptr< ::serializer::IsMovingResponse> IsMovingResponsePtr;
typedef boost::shared_ptr< ::serializer::IsMovingResponse const> IsMovingResponseConstPtr;

struct IsMoving
{

typedef IsMovingRequest Request;
typedef IsMovingResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct IsMoving
} // namespace serializer

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::serializer::IsMovingRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::serializer::IsMovingRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::serializer::IsMovingRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "serializer/IsMovingRequest";
  }

  static const char* value(const  ::serializer::IsMovingRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::serializer::IsMovingRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
";
  }

  static const char* value(const  ::serializer::IsMovingRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::serializer::IsMovingRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::serializer::IsMovingResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9104f1a32b4fbf4d3c8c80d9b9493250";
  }

  static const char* value(const  ::serializer::IsMovingResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x9104f1a32b4fbf4dULL;
  static const uint64_t static_value2 = 0x3c8c80d9b9493250ULL;
};

template<class ContainerAllocator>
struct DataType< ::serializer::IsMovingResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "serializer/IsMovingResponse";
  }

  static const char* value(const  ::serializer::IsMovingResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::serializer::IsMovingResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool moving\n\
\n\
\n\
";
  }

  static const char* value(const  ::serializer::IsMovingResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::serializer::IsMovingResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::serializer::IsMovingRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct IsMovingRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::serializer::IsMovingResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.moving);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct IsMovingResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<serializer::IsMoving> {
  static const char* value() 
  {
    return "9104f1a32b4fbf4d3c8c80d9b9493250";
  }

  static const char* value(const serializer::IsMoving&) { return value(); } 
};

template<>
struct DataType<serializer::IsMoving> {
  static const char* value() 
  {
    return "serializer/IsMoving";
  }

  static const char* value(const serializer::IsMoving&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<serializer::IsMovingRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9104f1a32b4fbf4d3c8c80d9b9493250";
  }

  static const char* value(const serializer::IsMovingRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<serializer::IsMovingRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "serializer/IsMoving";
  }

  static const char* value(const serializer::IsMovingRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<serializer::IsMovingResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9104f1a32b4fbf4d3c8c80d9b9493250";
  }

  static const char* value(const serializer::IsMovingResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<serializer::IsMovingResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "serializer/IsMoving";
  }

  static const char* value(const serializer::IsMovingResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // SERIALIZER_SERVICE_ISMOVING_H

