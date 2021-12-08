// Generated by gencpp from file wtr_serial/actor_1.msg
// DO NOT EDIT!


#ifndef WTR_SERIAL_MESSAGE_ACTOR_1_H
#define WTR_SERIAL_MESSAGE_ACTOR_1_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace wtr_serial
{
template <class ContainerAllocator>
struct actor_1_
{
  typedef actor_1_<ContainerAllocator> Type;

  actor_1_()
    : id(0)
    , agl_0(0.0)
    , agl_1(0.0)
    , agl_2(0.0)  {
    }
  actor_1_(const ContainerAllocator& _alloc)
    : id(0)
    , agl_0(0.0)
    , agl_1(0.0)
    , agl_2(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _id_type;
  _id_type id;

   typedef float _agl_0_type;
  _agl_0_type agl_0;

   typedef float _agl_1_type;
  _agl_1_type agl_1;

   typedef float _agl_2_type;
  _agl_2_type agl_2;





  typedef boost::shared_ptr< ::wtr_serial::actor_1_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::wtr_serial::actor_1_<ContainerAllocator> const> ConstPtr;

}; // struct actor_1_

typedef ::wtr_serial::actor_1_<std::allocator<void> > actor_1;

typedef boost::shared_ptr< ::wtr_serial::actor_1 > actor_1Ptr;
typedef boost::shared_ptr< ::wtr_serial::actor_1 const> actor_1ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::wtr_serial::actor_1_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::wtr_serial::actor_1_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::wtr_serial::actor_1_<ContainerAllocator1> & lhs, const ::wtr_serial::actor_1_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.agl_0 == rhs.agl_0 &&
    lhs.agl_1 == rhs.agl_1 &&
    lhs.agl_2 == rhs.agl_2;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::wtr_serial::actor_1_<ContainerAllocator1> & lhs, const ::wtr_serial::actor_1_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace wtr_serial

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::wtr_serial::actor_1_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::wtr_serial::actor_1_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wtr_serial::actor_1_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wtr_serial::actor_1_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wtr_serial::actor_1_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wtr_serial::actor_1_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::wtr_serial::actor_1_<ContainerAllocator> >
{
  static const char* value()
  {
    return "41170c4af0e4f57d0ef889762422e6e9";
  }

  static const char* value(const ::wtr_serial::actor_1_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x41170c4af0e4f57dULL;
  static const uint64_t static_value2 = 0x0ef889762422e6e9ULL;
};

template<class ContainerAllocator>
struct DataType< ::wtr_serial::actor_1_<ContainerAllocator> >
{
  static const char* value()
  {
    return "wtr_serial/actor_1";
  }

  static const char* value(const ::wtr_serial::actor_1_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::wtr_serial::actor_1_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 id\n"
"float32 agl_0\n"
"float32 agl_1\n"
"float32 agl_2\n"
;
  }

  static const char* value(const ::wtr_serial::actor_1_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::wtr_serial::actor_1_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.agl_0);
      stream.next(m.agl_1);
      stream.next(m.agl_2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct actor_1_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::wtr_serial::actor_1_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::wtr_serial::actor_1_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id);
    s << indent << "agl_0: ";
    Printer<float>::stream(s, indent + "  ", v.agl_0);
    s << indent << "agl_1: ";
    Printer<float>::stream(s, indent + "  ", v.agl_1);
    s << indent << "agl_2: ";
    Printer<float>::stream(s, indent + "  ", v.agl_2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WTR_SERIAL_MESSAGE_ACTOR_1_H