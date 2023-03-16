// Generated by gencpp from file niryo_one_msgs/UpdateConveyorIdRequest.msg
// DO NOT EDIT!


#ifndef NIRYO_ONE_MSGS_MESSAGE_UPDATECONVEYORIDREQUEST_H
#define NIRYO_ONE_MSGS_MESSAGE_UPDATECONVEYORIDREQUEST_H


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
struct UpdateConveyorIdRequest_
{
  typedef UpdateConveyorIdRequest_<ContainerAllocator> Type;

  UpdateConveyorIdRequest_()
    : old_id(0)
    , new_id(0)  {
    }
  UpdateConveyorIdRequest_(const ContainerAllocator& _alloc)
    : old_id(0)
    , new_id(0)  {
  (void)_alloc;
    }



   typedef uint8_t _old_id_type;
  _old_id_type old_id;

   typedef uint8_t _new_id_type;
  _new_id_type new_id;





  typedef boost::shared_ptr< ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> const> ConstPtr;

}; // struct UpdateConveyorIdRequest_

typedef ::niryo_one_msgs::UpdateConveyorIdRequest_<std::allocator<void> > UpdateConveyorIdRequest;

typedef boost::shared_ptr< ::niryo_one_msgs::UpdateConveyorIdRequest > UpdateConveyorIdRequestPtr;
typedef boost::shared_ptr< ::niryo_one_msgs::UpdateConveyorIdRequest const> UpdateConveyorIdRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3e7bf0b60e9cee26653ed700a1f3581c";
  }

  static const char* value(const ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3e7bf0b60e9cee26ULL;
  static const uint64_t static_value2 = 0x653ed700a1f3581cULL;
};

template<class ContainerAllocator>
struct DataType< ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "niryo_one_msgs/UpdateConveyorIdRequest";
  }

  static const char* value(const ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 old_id\n\
uint8 new_id\n\
";
  }

  static const char* value(const ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.old_id);
      stream.next(m.new_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UpdateConveyorIdRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::niryo_one_msgs::UpdateConveyorIdRequest_<ContainerAllocator>& v)
  {
    s << indent << "old_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.old_id);
    s << indent << "new_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.new_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NIRYO_ONE_MSGS_MESSAGE_UPDATECONVEYORIDREQUEST_H
