// Generated by gencpp from file lorett_c4s/yawPoseRequest.msg
// DO NOT EDIT!


#ifndef LORETT_C4S_MESSAGE_YAWPOSEREQUEST_H
#define LORETT_C4S_MESSAGE_YAWPOSEREQUEST_H


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
struct yawPoseRequest_
{
  typedef yawPoseRequest_<ContainerAllocator> Type;

  yawPoseRequest_()
    : yaw(0.0)  {
    }
  yawPoseRequest_(const ContainerAllocator& _alloc)
    : yaw(0.0)  {
  (void)_alloc;
    }



   typedef float _yaw_type;
  _yaw_type yaw;





  typedef boost::shared_ptr< ::lorett_c4s::yawPoseRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lorett_c4s::yawPoseRequest_<ContainerAllocator> const> ConstPtr;

}; // struct yawPoseRequest_

typedef ::lorett_c4s::yawPoseRequest_<std::allocator<void> > yawPoseRequest;

typedef boost::shared_ptr< ::lorett_c4s::yawPoseRequest > yawPoseRequestPtr;
typedef boost::shared_ptr< ::lorett_c4s::yawPoseRequest const> yawPoseRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lorett_c4s::yawPoseRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lorett_c4s::yawPoseRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lorett_c4s::yawPoseRequest_<ContainerAllocator1> & lhs, const ::lorett_c4s::yawPoseRequest_<ContainerAllocator2> & rhs)
{
  return lhs.yaw == rhs.yaw;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lorett_c4s::yawPoseRequest_<ContainerAllocator1> & lhs, const ::lorett_c4s::yawPoseRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lorett_c4s

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::lorett_c4s::yawPoseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lorett_c4s::yawPoseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lorett_c4s::yawPoseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lorett_c4s::yawPoseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lorett_c4s::yawPoseRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lorett_c4s::yawPoseRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lorett_c4s::yawPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2160bf7632f25ad6dc7c5aab561198d4";
  }

  static const char* value(const ::lorett_c4s::yawPoseRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2160bf7632f25ad6ULL;
  static const uint64_t static_value2 = 0xdc7c5aab561198d4ULL;
};

template<class ContainerAllocator>
struct DataType< ::lorett_c4s::yawPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lorett_c4s/yawPoseRequest";
  }

  static const char* value(const ::lorett_c4s::yawPoseRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lorett_c4s::yawPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 yaw\n"
;
  }

  static const char* value(const ::lorett_c4s::yawPoseRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lorett_c4s::yawPoseRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.yaw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct yawPoseRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lorett_c4s::yawPoseRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lorett_c4s::yawPoseRequest_<ContainerAllocator>& v)
  {
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LORETT_C4S_MESSAGE_YAWPOSEREQUEST_H
