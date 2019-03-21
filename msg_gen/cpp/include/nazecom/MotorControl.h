/* Auto-generated by genmsg_cpp for file /home/stsc/work/ros_demo/nazecom/msg/MotorControl.msg */
#ifndef NAZECOM_MESSAGE_MOTORCONTROL_H
#define NAZECOM_MESSAGE_MOTORCONTROL_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace nazecom
{
template <class ContainerAllocator>
struct MotorControl_ {
  typedef MotorControl_<ContainerAllocator> Type;

  MotorControl_()
  : m0(0.0)
  , m1(0.0)
  , m2(0.0)
  , m3(0.0)
  , flags(0)
  {
  }

  MotorControl_(const ContainerAllocator& _alloc)
  : m0(0.0)
  , m1(0.0)
  , m2(0.0)
  , m3(0.0)
  , flags(0)
  {
  }

  typedef float _m0_type;
  float m0;

  typedef float _m1_type;
  float m1;

  typedef float _m2_type;
  float m2;

  typedef float _m3_type;
  float m3;

  typedef uint32_t _flags_type;
  uint32_t flags;


  typedef boost::shared_ptr< ::nazecom::MotorControl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nazecom::MotorControl_<ContainerAllocator>  const> ConstPtr;
}; // struct MotorControl
typedef  ::nazecom::MotorControl_<std::allocator<void> > MotorControl;

typedef boost::shared_ptr< ::nazecom::MotorControl> MotorControlPtr;
typedef boost::shared_ptr< ::nazecom::MotorControl const> MotorControlConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::nazecom::MotorControl_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::nazecom::MotorControl_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace nazecom

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nazecom::MotorControl_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nazecom::MotorControl_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nazecom::MotorControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "683da7fb6ac19e6ab15d9ab72a59046a";
  }

  static const char* value(const  ::nazecom::MotorControl_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x683da7fb6ac19e6aULL;
  static const uint64_t static_value2 = 0xb15d9ab72a59046aULL;
};

template<class ContainerAllocator>
struct DataType< ::nazecom::MotorControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nazecom/MotorControl";
  }

  static const char* value(const  ::nazecom::MotorControl_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nazecom::MotorControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 m0\n\
float32 m1\n\
float32 m2\n\
float32 m3\n\
uint32 flags\n\
\n\
";
  }

  static const char* value(const  ::nazecom::MotorControl_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nazecom::MotorControl_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nazecom::MotorControl_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.m0);
    stream.next(m.m1);
    stream.next(m.m2);
    stream.next(m.m3);
    stream.next(m.flags);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct MotorControl_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nazecom::MotorControl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::nazecom::MotorControl_<ContainerAllocator> & v) 
  {
    s << indent << "m0: ";
    Printer<float>::stream(s, indent + "  ", v.m0);
    s << indent << "m1: ";
    Printer<float>::stream(s, indent + "  ", v.m1);
    s << indent << "m2: ";
    Printer<float>::stream(s, indent + "  ", v.m2);
    s << indent << "m3: ";
    Printer<float>::stream(s, indent + "  ", v.m3);
    s << indent << "flags: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.flags);
  }
};


} // namespace message_operations
} // namespace ros

#endif // NAZECOM_MESSAGE_MOTORCONTROL_H

