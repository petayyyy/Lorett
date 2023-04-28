// Generated by gencpp from file lorett_c4s/SetAttitudeRequest.msg
// DO NOT EDIT!


#ifndef LORETT_C4S_MESSAGE_SETATTITUDEREQUEST_H
#define LORETT_C4S_MESSAGE_SETATTITUDEREQUEST_H


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
struct SetAttitudeRequest_
{
  typedef SetAttitudeRequest_<ContainerAllocator> Type;

  SetAttitudeRequest_()
    : pitch(0.0)
    , roll(0.0)
    , yaw(0.0)
    , thrust(0.0)
    , frame_id()
    , auto_arm(false)  {
    }
  SetAttitudeRequest_(const ContainerAllocator& _alloc)
    : pitch(0.0)
    , roll(0.0)
    , yaw(0.0)
    , thrust(0.0)
    , frame_id(_alloc)
    , auto_arm(false)  {
  (void)_alloc;
    }



   typedef float _pitch_type;
  _pitch_type pitch;

   typedef float _roll_type;
  _roll_type roll;

   typedef float _yaw_type;
  _yaw_type yaw;

   typedef float _thrust_type;
  _thrust_type thrust;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _frame_id_type;
  _frame_id_type frame_id;

   typedef uint8_t _auto_arm_type;
  _auto_arm_type auto_arm;





  typedef boost::shared_ptr< ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetAttitudeRequest_

typedef ::lorett_c4s::SetAttitudeRequest_<std::allocator<void> > SetAttitudeRequest;

typedef boost::shared_ptr< ::lorett_c4s::SetAttitudeRequest > SetAttitudeRequestPtr;
typedef boost::shared_ptr< ::lorett_c4s::SetAttitudeRequest const> SetAttitudeRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator1> & lhs, const ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator2> & rhs)
{
  return lhs.pitch == rhs.pitch &&
    lhs.roll == rhs.roll &&
    lhs.yaw == rhs.yaw &&
    lhs.thrust == rhs.thrust &&
    lhs.frame_id == rhs.frame_id &&
    lhs.auto_arm == rhs.auto_arm;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator1> & lhs, const ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lorett_c4s

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "57b7e3577c60ddfb174e6a5be8792292";
  }

  static const char* value(const ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x57b7e3577c60ddfbULL;
  static const uint64_t static_value2 = 0x174e6a5be8792292ULL;
};

template<class ContainerAllocator>
struct DataType< ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lorett_c4s/SetAttitudeRequest";
  }

  static const char* value(const ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 pitch\n"
"float32 roll\n"
"float32 yaw\n"
"float32 thrust\n"
"string frame_id\n"
"bool auto_arm\n"
;
  }

  static const char* value(const ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pitch);
      stream.next(m.roll);
      stream.next(m.yaw);
      stream.next(m.thrust);
      stream.next(m.frame_id);
      stream.next(m.auto_arm);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetAttitudeRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lorett_c4s::SetAttitudeRequest_<ContainerAllocator>& v)
  {
    s << indent << "pitch: ";
    Printer<float>::stream(s, indent + "  ", v.pitch);
    s << indent << "roll: ";
    Printer<float>::stream(s, indent + "  ", v.roll);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "thrust: ";
    Printer<float>::stream(s, indent + "  ", v.thrust);
    s << indent << "frame_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.frame_id);
    s << indent << "auto_arm: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.auto_arm);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LORETT_C4S_MESSAGE_SETATTITUDEREQUEST_H