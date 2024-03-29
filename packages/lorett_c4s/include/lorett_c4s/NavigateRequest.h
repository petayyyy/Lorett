// Generated by gencpp from file lorett_c4s/NavigateRequest.msg
// DO NOT EDIT!


#ifndef LORETT_C4S_MESSAGE_NAVIGATEREQUEST_H
#define LORETT_C4S_MESSAGE_NAVIGATEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace lorett_c4s
{
template <class ContainerAllocator>
struct NavigateRequest_
{
  typedef NavigateRequest_<ContainerAllocator> Type;

  NavigateRequest_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , yaw(0.0)
    , yaw_rate(0.0)
    , speed(0.0)
    , frame_id()
    , auto_arm(false)  {
    }
  NavigateRequest_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , yaw(0.0)
    , yaw_rate(0.0)
    , speed(0.0)
    , frame_id(_alloc)
    , auto_arm(false)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;

   typedef float _yaw_type;
  _yaw_type yaw;

   typedef float _yaw_rate_type;
  _yaw_rate_type yaw_rate;

   typedef float _speed_type;
  _speed_type speed;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _frame_id_type;
  _frame_id_type frame_id;

   typedef uint8_t _auto_arm_type;
  _auto_arm_type auto_arm;





  typedef boost::shared_ptr< ::lorett_c4s::NavigateRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lorett_c4s::NavigateRequest_<ContainerAllocator> const> ConstPtr;

}; // struct NavigateRequest_

typedef ::lorett_c4s::NavigateRequest_<std::allocator<void> > NavigateRequest;

typedef boost::shared_ptr< ::lorett_c4s::NavigateRequest > NavigateRequestPtr;
typedef boost::shared_ptr< ::lorett_c4s::NavigateRequest const> NavigateRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lorett_c4s::NavigateRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lorett_c4s::NavigateRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lorett_c4s::NavigateRequest_<ContainerAllocator1> & lhs, const ::lorett_c4s::NavigateRequest_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.yaw == rhs.yaw &&
    lhs.yaw_rate == rhs.yaw_rate &&
    lhs.speed == rhs.speed &&
    lhs.frame_id == rhs.frame_id &&
    lhs.auto_arm == rhs.auto_arm;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lorett_c4s::NavigateRequest_<ContainerAllocator1> & lhs, const ::lorett_c4s::NavigateRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lorett_c4s

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::lorett_c4s::NavigateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lorett_c4s::NavigateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lorett_c4s::NavigateRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lorett_c4s::NavigateRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lorett_c4s::NavigateRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lorett_c4s::NavigateRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lorett_c4s::NavigateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7695bfffababf12319f0958d40f3d0d8";
  }

  static const char* value(const ::lorett_c4s::NavigateRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7695bfffababf123ULL;
  static const uint64_t static_value2 = 0x19f0958d40f3d0d8ULL;
};

template<class ContainerAllocator>
struct DataType< ::lorett_c4s::NavigateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lorett_c4s/NavigateRequest";
  }

  static const char* value(const ::lorett_c4s::NavigateRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lorett_c4s::NavigateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
"float32 z\n"
"float32 yaw\n"
"float32 yaw_rate\n"
"float32 speed\n"
"string frame_id\n"
"bool auto_arm\n"
;
  }

  static const char* value(const ::lorett_c4s::NavigateRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lorett_c4s::NavigateRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.yaw);
      stream.next(m.yaw_rate);
      stream.next(m.speed);
      stream.next(m.frame_id);
      stream.next(m.auto_arm);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavigateRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lorett_c4s::NavigateRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lorett_c4s::NavigateRequest_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "yaw_rate: ";
    Printer<float>::stream(s, indent + "  ", v.yaw_rate);
    s << indent << "speed: ";
    Printer<float>::stream(s, indent + "  ", v.speed);
    s << indent << "frame_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.frame_id);
    s << indent << "auto_arm: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.auto_arm);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LORETT_C4S_MESSAGE_NAVIGATEREQUEST_H
