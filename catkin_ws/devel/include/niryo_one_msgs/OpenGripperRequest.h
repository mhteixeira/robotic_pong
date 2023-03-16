// Generated by gencpp from file niryo_one_msgs/OpenGripperRequest.msg
// DO NOT EDIT!


#ifndef NIRYO_ONE_MSGS_MESSAGE_OPENGRIPPERREQUEST_H
#define NIRYO_ONE_MSGS_MESSAGE_OPENGRIPPERREQUEST_H


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
struct OpenGripperRequest_
{
  typedef OpenGripperRequest_<ContainerAllocator> Type;

  OpenGripperRequest_()
    : id(0)
    , open_position(0)
    , open_speed(0)
    , open_hold_torque(0)  {
    }
  OpenGripperRequest_(const ContainerAllocator& _alloc)
    : id(0)
    , open_position(0)
    , open_speed(0)
    , open_hold_torque(0)  {
  (void)_alloc;
    }



   typedef uint8_t _id_type;
  _id_type id;

   typedef int16_t _open_position_type;
  _open_position_type open_position;

   typedef int16_t _open_speed_type;
  _open_speed_type open_speed;

   typedef int16_t _open_hold_torque_type;
  _open_hold_torque_type open_hold_torque;





  typedef boost::shared_ptr< ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> const> ConstPtr;

}; // struct OpenGripperRequest_

typedef ::niryo_one_msgs::OpenGripperRequest_<std::allocator<void> > OpenGripperRequest;

typedef boost::shared_ptr< ::niryo_one_msgs::OpenGripperRequest > OpenGripperRequestPtr;
typedef boost::shared_ptr< ::niryo_one_msgs::OpenGripperRequest const> OpenGripperRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace niryo_one_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'shape_msgs': ['/opt/ros/kinetic/share/shape_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'moveit_msgs': ['/opt/ros/kinetic/share/moveit_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'object_recognition_msgs': ['/opt/ros/kinetic/share/object_recognition_msgs/cmake/../msg'], 'octomap_msgs': ['/opt/ros/kinetic/share/octomap_msgs/cmake/../msg'], 'niryo_one_msgs': ['/home/pedro/catkin_ws/src/niryo_one_ros/niryo_one_msgs/msg', '/home/pedro/catkin_ws/devel/share/niryo_one_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bcf0f25a53019052992fbcb00df9771a";
  }

  static const char* value(const ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbcf0f25a53019052ULL;
  static const uint64_t static_value2 = 0x992fbcb00df9771aULL;
};

template<class ContainerAllocator>
struct DataType< ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "niryo_one_msgs/OpenGripperRequest";
  }

  static const char* value(const ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
uint8 id\n\
\n\
int16 open_position\n\
int16 open_speed\n\
int16 open_hold_torque\n\
";
  }

  static const char* value(const ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.open_position);
      stream.next(m.open_speed);
      stream.next(m.open_hold_torque);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct OpenGripperRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::niryo_one_msgs::OpenGripperRequest_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id);
    s << indent << "open_position: ";
    Printer<int16_t>::stream(s, indent + "  ", v.open_position);
    s << indent << "open_speed: ";
    Printer<int16_t>::stream(s, indent + "  ", v.open_speed);
    s << indent << "open_hold_torque: ";
    Printer<int16_t>::stream(s, indent + "  ", v.open_hold_torque);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NIRYO_ONE_MSGS_MESSAGE_OPENGRIPPERREQUEST_H
