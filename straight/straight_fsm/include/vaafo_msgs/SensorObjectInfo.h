// Generated by gencpp from file vaafo_msgs/SensorObjectInfo.msg
// DO NOT EDIT!


#ifndef VAAFO_MSGS_MESSAGE_SENSOROBJECTINFO_H
#define VAAFO_MSGS_MESSAGE_SENSOROBJECTINFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

namespace vaafo_msgs
{
template <class ContainerAllocator>
struct SensorObjectInfo_
{
  typedef SensorObjectInfo_<ContainerAllocator> Type;

  SensorObjectInfo_()
    : header()
    , traffic_id(0)
    , pose()
    , velocity()
    , acceleration()
    , dimension()
    , rel_velocity()
    , rel_Ref_dis(0.0)
    , rel_Near_dis(0.0)
    , rel_Ref_azimut(0.0)
    , rel_Near_azimut(0.0)
    , rel_Ref_orientation(0.0)
    , rel_Ref_vel(0.0)  {
    }
  SensorObjectInfo_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , traffic_id(0)
    , pose(_alloc)
    , velocity(_alloc)
    , acceleration(_alloc)
    , dimension(_alloc)
    , rel_velocity(_alloc)
    , rel_Ref_dis(0.0)
    , rel_Near_dis(0.0)
    , rel_Ref_azimut(0.0)
    , rel_Near_azimut(0.0)
    , rel_Ref_orientation(0.0)
    , rel_Ref_vel(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int8_t _traffic_id_type;
  _traffic_id_type traffic_id;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _acceleration_type;
  _acceleration_type acceleration;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _dimension_type;
  _dimension_type dimension;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _rel_velocity_type;
  _rel_velocity_type rel_velocity;

   typedef double _rel_Ref_dis_type;
  _rel_Ref_dis_type rel_Ref_dis;

   typedef double _rel_Near_dis_type;
  _rel_Near_dis_type rel_Near_dis;

   typedef double _rel_Ref_azimut_type;
  _rel_Ref_azimut_type rel_Ref_azimut;

   typedef double _rel_Near_azimut_type;
  _rel_Near_azimut_type rel_Near_azimut;

   typedef double _rel_Ref_orientation_type;
  _rel_Ref_orientation_type rel_Ref_orientation;

   typedef double _rel_Ref_vel_type;
  _rel_Ref_vel_type rel_Ref_vel;





  typedef boost::shared_ptr< ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> const> ConstPtr;

}; // struct SensorObjectInfo_

typedef ::vaafo_msgs::SensorObjectInfo_<std::allocator<void> > SensorObjectInfo;

typedef boost::shared_ptr< ::vaafo_msgs::SensorObjectInfo > SensorObjectInfoPtr;
typedef boost::shared_ptr< ::vaafo_msgs::SensorObjectInfo const> SensorObjectInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator1> & lhs, const ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.traffic_id == rhs.traffic_id &&
    lhs.pose == rhs.pose &&
    lhs.velocity == rhs.velocity &&
    lhs.acceleration == rhs.acceleration &&
    lhs.dimension == rhs.dimension &&
    lhs.rel_velocity == rhs.rel_velocity &&
    lhs.rel_Ref_dis == rhs.rel_Ref_dis &&
    lhs.rel_Near_dis == rhs.rel_Near_dis &&
    lhs.rel_Ref_azimut == rhs.rel_Ref_azimut &&
    lhs.rel_Near_azimut == rhs.rel_Near_azimut &&
    lhs.rel_Ref_orientation == rhs.rel_Ref_orientation &&
    lhs.rel_Ref_vel == rhs.rel_Ref_vel;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator1> & lhs, const ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vaafo_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0c13c790ba04f0aa1f5e8de3cf8430e3";
  }

  static const char* value(const ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0c13c790ba04f0aaULL;
  static const uint64_t static_value2 = 0x1f5e8de3cf8430e3ULL;
};

template<class ContainerAllocator>
struct DataType< ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vaafo_msgs/SensorObjectInfo";
  }

  static const char* value(const ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"int8 traffic_id\n"
"geometry_msgs/Pose pose\n"
"geometry_msgs/Twist velocity\n"
"geometry_msgs/Twist acceleration\n"
"geometry_msgs/Point dimension\n"
"geometry_msgs/Twist rel_velocity\n"
"float64 rel_Ref_dis\n"
"float64 rel_Near_dis\n"
"float64 rel_Ref_azimut\n"
"float64 rel_Near_azimut\n"
"float64 rel_Ref_orientation\n"
"float64 rel_Ref_vel\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.traffic_id);
      stream.next(m.pose);
      stream.next(m.velocity);
      stream.next(m.acceleration);
      stream.next(m.dimension);
      stream.next(m.rel_velocity);
      stream.next(m.rel_Ref_dis);
      stream.next(m.rel_Near_dis);
      stream.next(m.rel_Ref_azimut);
      stream.next(m.rel_Near_azimut);
      stream.next(m.rel_Ref_orientation);
      stream.next(m.rel_Ref_vel);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SensorObjectInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vaafo_msgs::SensorObjectInfo_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "traffic_id: ";
    Printer<int8_t>::stream(s, indent + "  ", v.traffic_id);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
    s << indent << "acceleration: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.acceleration);
    s << indent << "dimension: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.dimension);
    s << indent << "rel_velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.rel_velocity);
    s << indent << "rel_Ref_dis: ";
    Printer<double>::stream(s, indent + "  ", v.rel_Ref_dis);
    s << indent << "rel_Near_dis: ";
    Printer<double>::stream(s, indent + "  ", v.rel_Near_dis);
    s << indent << "rel_Ref_azimut: ";
    Printer<double>::stream(s, indent + "  ", v.rel_Ref_azimut);
    s << indent << "rel_Near_azimut: ";
    Printer<double>::stream(s, indent + "  ", v.rel_Near_azimut);
    s << indent << "rel_Ref_orientation: ";
    Printer<double>::stream(s, indent + "  ", v.rel_Ref_orientation);
    s << indent << "rel_Ref_vel: ";
    Printer<double>::stream(s, indent + "  ", v.rel_Ref_vel);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VAAFO_MSGS_MESSAGE_SENSOROBJECTINFO_H