// Generated by gencpp from file kobuki_msgs/ControllerInfo.msg
// DO NOT EDIT!


#ifndef KOBUKI_MSGS_MESSAGE_CONTROLLERINFO_H
#define KOBUKI_MSGS_MESSAGE_CONTROLLERINFO_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kobuki_msgs
{
template <class ContainerAllocator>
struct ControllerInfo_
{
  typedef ControllerInfo_<ContainerAllocator> Type;

  ControllerInfo_()
    : type(0)
    , p_gain(0.0)
    , i_gain(0.0)
    , d_gain(0.0)  {
    }
  ControllerInfo_(const ContainerAllocator& _alloc)
    : type(0)
    , p_gain(0.0)
    , i_gain(0.0)
    , d_gain(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _type_type;
  _type_type type;

   typedef double _p_gain_type;
  _p_gain_type p_gain;

   typedef double _i_gain_type;
  _i_gain_type i_gain;

   typedef double _d_gain_type;
  _d_gain_type d_gain;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(DEFAULT)
  #undef DEFAULT
#endif
#if defined(_WIN32) && defined(USER_CONFIGURED)
  #undef USER_CONFIGURED
#endif

  enum {
    DEFAULT = 0u,
    USER_CONFIGURED = 1u,
  };


  typedef boost::shared_ptr< ::kobuki_msgs::ControllerInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kobuki_msgs::ControllerInfo_<ContainerAllocator> const> ConstPtr;

}; // struct ControllerInfo_

typedef ::kobuki_msgs::ControllerInfo_<std::allocator<void> > ControllerInfo;

typedef boost::shared_ptr< ::kobuki_msgs::ControllerInfo > ControllerInfoPtr;
typedef boost::shared_ptr< ::kobuki_msgs::ControllerInfo const> ControllerInfoConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kobuki_msgs::ControllerInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kobuki_msgs::ControllerInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kobuki_msgs::ControllerInfo_<ContainerAllocator1> & lhs, const ::kobuki_msgs::ControllerInfo_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type &&
    lhs.p_gain == rhs.p_gain &&
    lhs.i_gain == rhs.i_gain &&
    lhs.d_gain == rhs.d_gain;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kobuki_msgs::ControllerInfo_<ContainerAllocator1> & lhs, const ::kobuki_msgs::ControllerInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kobuki_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::kobuki_msgs::ControllerInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kobuki_msgs::ControllerInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kobuki_msgs::ControllerInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kobuki_msgs::ControllerInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kobuki_msgs::ControllerInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kobuki_msgs::ControllerInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kobuki_msgs::ControllerInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "46af38e72f0640f41ed22174d43a2550";
  }

  static const char* value(const ::kobuki_msgs::ControllerInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x46af38e72f0640f4ULL;
  static const uint64_t static_value2 = 0x1ed22174d43a2550ULL;
};

template<class ContainerAllocator>
struct DataType< ::kobuki_msgs::ControllerInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kobuki_msgs/ControllerInfo";
  }

  static const char* value(const ::kobuki_msgs::ControllerInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kobuki_msgs::ControllerInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Controller info message, contains PID parameters\n"
"\n"
"uint8 DEFAULT   =  0\n"
"uint8 USER_CONFIGURED =  1\n"
"\n"
"uint8 type\n"
"float64 p_gain #should be positive\n"
"float64 i_gain #should be positive\n"
"float64 d_gain #should be positive\n"
;
  }

  static const char* value(const ::kobuki_msgs::ControllerInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kobuki_msgs::ControllerInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.p_gain);
      stream.next(m.i_gain);
      stream.next(m.d_gain);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ControllerInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kobuki_msgs::ControllerInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kobuki_msgs::ControllerInfo_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "p_gain: ";
    Printer<double>::stream(s, indent + "  ", v.p_gain);
    s << indent << "i_gain: ";
    Printer<double>::stream(s, indent + "  ", v.i_gain);
    s << indent << "d_gain: ";
    Printer<double>::stream(s, indent + "  ", v.d_gain);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KOBUKI_MSGS_MESSAGE_CONTROLLERINFO_H
