// Generated by gencpp from file niryo_one_msgs/SetLedsRequest.msg
// DO NOT EDIT!


#ifndef NIRYO_ONE_MSGS_MESSAGE_SETLEDSREQUEST_H
#define NIRYO_ONE_MSGS_MESSAGE_SETLEDSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace niryo_one_msgs
{
template <class ContainerAllocator>
struct SetLedsRequest_
{
  typedef SetLedsRequest_<ContainerAllocator> Type;

  SetLedsRequest_()
    : values()  {
    }
  SetLedsRequest_(const ContainerAllocator& _alloc)
    : values(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _values_type;
  _values_type values;





  typedef boost::shared_ptr< ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetLedsRequest_

typedef ::niryo_one_msgs::SetLedsRequest_<std::allocator<void> > SetLedsRequest;

typedef boost::shared_ptr< ::niryo_one_msgs::SetLedsRequest > SetLedsRequestPtr;
typedef boost::shared_ptr< ::niryo_one_msgs::SetLedsRequest const> SetLedsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace niryo_one_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'shape_msgs': ['/opt/ros/kinetic/share/shape_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'moveit_msgs': ['/opt/ros/kinetic/share/moveit_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'object_recognition_msgs': ['/opt/ros/kinetic/share/object_recognition_msgs/cmake/../msg'], 'octomap_msgs': ['/opt/ros/kinetic/share/octomap_msgs/cmake/../msg'], 'niryo_one_msgs': ['/home/pedro/catkin_ws/src/niryo_one_ros/niryo_one_msgs/msg', '/home/pedro/catkin_ws/devel/share/niryo_one_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5dd1053b3769329bd3895728a55810d3";
  }

  static const char* value(const ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5dd1053b3769329bULL;
  static const uint64_t static_value2 = 0xd3895728a55810d3ULL;
};

template<class ContainerAllocator>
struct DataType< ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "niryo_one_msgs/SetLedsRequest";
  }

  static const char* value(const ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32[] values\n\
";
  }

  static const char* value(const ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.values);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetLedsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::niryo_one_msgs::SetLedsRequest_<ContainerAllocator>& v)
  {
    s << indent << "values[]" << std::endl;
    for (size_t i = 0; i < v.values.size(); ++i)
    {
      s << indent << "  values[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.values[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // NIRYO_ONE_MSGS_MESSAGE_SETLEDSREQUEST_H
