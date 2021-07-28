// Generated by gencpp from file my_example_pkg/my_servicesResponse.msg
// DO NOT EDIT!


#ifndef MY_EXAMPLE_PKG_MESSAGE_MY_SERVICESRESPONSE_H
#define MY_EXAMPLE_PKG_MESSAGE_MY_SERVICESRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace my_example_pkg
{
template <class ContainerAllocator>
struct my_servicesResponse_
{
  typedef my_servicesResponse_<ContainerAllocator> Type;

  my_servicesResponse_()
    : answer()  {
    }
  my_servicesResponse_(const ContainerAllocator& _alloc)
    : answer(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _answer_type;
  _answer_type answer;





  typedef boost::shared_ptr< ::my_example_pkg::my_servicesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::my_example_pkg::my_servicesResponse_<ContainerAllocator> const> ConstPtr;

}; // struct my_servicesResponse_

typedef ::my_example_pkg::my_servicesResponse_<std::allocator<void> > my_servicesResponse;

typedef boost::shared_ptr< ::my_example_pkg::my_servicesResponse > my_servicesResponsePtr;
typedef boost::shared_ptr< ::my_example_pkg::my_servicesResponse const> my_servicesResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::my_example_pkg::my_servicesResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::my_example_pkg::my_servicesResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::my_example_pkg::my_servicesResponse_<ContainerAllocator1> & lhs, const ::my_example_pkg::my_servicesResponse_<ContainerAllocator2> & rhs)
{
  return lhs.answer == rhs.answer;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::my_example_pkg::my_servicesResponse_<ContainerAllocator1> & lhs, const ::my_example_pkg::my_servicesResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace my_example_pkg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::my_example_pkg::my_servicesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::my_example_pkg::my_servicesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::my_example_pkg::my_servicesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::my_example_pkg::my_servicesResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_example_pkg::my_servicesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_example_pkg::my_servicesResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::my_example_pkg::my_servicesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d7e708f879c94bb931716d8f4f130f30";
  }

  static const char* value(const ::my_example_pkg::my_servicesResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd7e708f879c94bb9ULL;
  static const uint64_t static_value2 = 0x31716d8f4f130f30ULL;
};

template<class ContainerAllocator>
struct DataType< ::my_example_pkg::my_servicesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "my_example_pkg/my_servicesResponse";
  }

  static const char* value(const ::my_example_pkg::my_servicesResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::my_example_pkg::my_servicesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string answer\n"
;
  }

  static const char* value(const ::my_example_pkg::my_servicesResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::my_example_pkg::my_servicesResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.answer);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct my_servicesResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::my_example_pkg::my_servicesResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::my_example_pkg::my_servicesResponse_<ContainerAllocator>& v)
  {
    s << indent << "answer: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.answer);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MY_EXAMPLE_PKG_MESSAGE_MY_SERVICESRESPONSE_H
