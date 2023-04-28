// Generated by gencpp from file lorett_c4s/publishPoseRequest.msg
// DO NOT EDIT!


#ifndef LORETT_C4S_MESSAGE_PUBLISHPOSEREQUEST_H
#define LORETT_C4S_MESSAGE_PUBLISHPOSEREQUEST_H


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
struct publishPoseRequest_
{
  typedef publishPoseRequest_<ContainerAllocator> Type;

  publishPoseRequest_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  publishPoseRequest_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;





  typedef boost::shared_ptr< ::lorett_c4s::publishPoseRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lorett_c4s::publishPoseRequest_<ContainerAllocator> const> ConstPtr;

}; // struct publishPoseRequest_

typedef ::lorett_c4s::publishPoseRequest_<std::allocator<void> > publishPoseRequest;

typedef boost::shared_ptr< ::lorett_c4s::publishPoseRequest > publishPoseRequestPtr;
typedef boost::shared_ptr< ::lorett_c4s::publishPoseRequest const> publishPoseRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lorett_c4s::publishPoseRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lorett_c4s::publishPoseRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lorett_c4s::publishPoseRequest_<ContainerAllocator1> & lhs, const ::lorett_c4s::publishPoseRequest_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lorett_c4s::publishPoseRequest_<ContainerAllocator1> & lhs, const ::lorett_c4s::publishPoseRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lorett_c4s

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::lorett_c4s::publishPoseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lorett_c4s::publishPoseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lorett_c4s::publishPoseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lorett_c4s::publishPoseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lorett_c4s::publishPoseRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lorett_c4s::publishPoseRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lorett_c4s::publishPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cc153912f1453b708d221682bc23d9ac";
  }

  static const char* value(const ::lorett_c4s::publishPoseRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcc153912f1453b70ULL;
  static const uint64_t static_value2 = 0x8d221682bc23d9acULL;
};

template<class ContainerAllocator>
struct DataType< ::lorett_c4s::publishPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lorett_c4s/publishPoseRequest";
  }

  static const char* value(const ::lorett_c4s::publishPoseRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lorett_c4s::publishPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::lorett_c4s::publishPoseRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lorett_c4s::publishPoseRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct publishPoseRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lorett_c4s::publishPoseRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lorett_c4s::publishPoseRequest_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LORETT_C4S_MESSAGE_PUBLISHPOSEREQUEST_H
