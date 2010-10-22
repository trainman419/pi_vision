/* Auto-generated by genmsg_cpp for file /home/patrick/Eclipse/ros/pi-robot-ros-stack/serializer/msg/SensorState.msg */
#ifndef SERIALIZER_MESSAGE_SENSORSTATE_H
#define SERIALIZER_MESSAGE_SENSORSTATE_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "roslib/Header.h"

namespace serializer
{
template <class ContainerAllocator>
struct SensorState_ : public ros::Message
{
  typedef SensorState_<ContainerAllocator> Type;

  SensorState_()
  : header()
  , name()
  , value()
  {
  }

  SensorState_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , name(_alloc)
  , value(_alloc)
  {
  }

  typedef  ::roslib::Header_<ContainerAllocator>  _header_type;
   ::roslib::Header_<ContainerAllocator>  header;

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _name_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  name;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _value_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  value;


  ROSCPP_DEPRECATED uint32_t get_name_size() const { return (uint32_t)name.size(); }
  ROSCPP_DEPRECATED void set_name_size(uint32_t size) { name.resize((size_t)size); }
  ROSCPP_DEPRECATED void get_name_vec(std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other > & vec) const { vec = this->name; }
  ROSCPP_DEPRECATED void set_name_vec(const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other > & vec) { this->name = vec; }
  ROSCPP_DEPRECATED uint32_t get_value_size() const { return (uint32_t)value.size(); }
  ROSCPP_DEPRECATED void set_value_size(uint32_t size) { value.resize((size_t)size); }
  ROSCPP_DEPRECATED void get_value_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->value; }
  ROSCPP_DEPRECATED void set_value_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->value = vec; }
private:
  static const char* __s_getDataType_() { return "serializer/SensorState"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "c775d5ae64f1f355fcb3c88b89468dd0"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "Header header\n\
\n\
string[] name\n\
float32[] value\n\
\n\
================================================================================\n\
MSG: roslib/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, name);
    ros::serialization::serialize(stream, value);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, name);
    ros::serialization::deserialize(stream, value);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(name);
    size += ros::serialization::serializationLength(value);
    return size;
  }

  typedef boost::shared_ptr< ::serializer::SensorState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serializer::SensorState_<ContainerAllocator>  const> ConstPtr;
}; // struct SensorState
typedef  ::serializer::SensorState_<std::allocator<void> > SensorState;

typedef boost::shared_ptr< ::serializer::SensorState> SensorStatePtr;
typedef boost::shared_ptr< ::serializer::SensorState const> SensorStateConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::serializer::SensorState_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::serializer::SensorState_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace serializer

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::serializer::SensorState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c775d5ae64f1f355fcb3c88b89468dd0";
  }

  static const char* value(const  ::serializer::SensorState_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc775d5ae64f1f355ULL;
  static const uint64_t static_value2 = 0xfcb3c88b89468dd0ULL;
};

template<class ContainerAllocator>
struct DataType< ::serializer::SensorState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "serializer/SensorState";
  }

  static const char* value(const  ::serializer::SensorState_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::serializer::SensorState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
\n\
string[] name\n\
float32[] value\n\
\n\
================================================================================\n\
MSG: roslib/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::serializer::SensorState_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::serializer::SensorState_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::serializer::SensorState_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.name);
    stream.next(m.value);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SensorState_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::serializer::SensorState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::serializer::SensorState_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::roslib::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "name[]" << std::endl;
    for (size_t i = 0; i < v.name.size(); ++i)
    {
      s << indent << "  name[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name[i]);
    }
    s << indent << "value[]" << std::endl;
    for (size_t i = 0; i < v.value.size(); ++i)
    {
      s << indent << "  value[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.value[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // SERIALIZER_MESSAGE_SENSORSTATE_H

