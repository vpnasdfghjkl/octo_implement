// Generated by gencpp from file dynamic_biped/recordArmHandPose.msg
// DO NOT EDIT!


#ifndef DYNAMIC_BIPED_MESSAGE_RECORDARMHANDPOSE_H
#define DYNAMIC_BIPED_MESSAGE_RECORDARMHANDPOSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <dynamic_biped/armHandPose.h>
#include <dynamic_biped/armHandPose.h>

namespace dynamic_biped
{
template <class ContainerAllocator>
struct recordArmHandPose_
{
  typedef recordArmHandPose_<ContainerAllocator> Type;

  recordArmHandPose_()
    : header()
    , left_pose()
    , right_pose()  {
    }
  recordArmHandPose_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , left_pose(_alloc)
    , right_pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::dynamic_biped::armHandPose_<ContainerAllocator>  _left_pose_type;
  _left_pose_type left_pose;

   typedef  ::dynamic_biped::armHandPose_<ContainerAllocator>  _right_pose_type;
  _right_pose_type right_pose;





  typedef boost::shared_ptr< ::dynamic_biped::recordArmHandPose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dynamic_biped::recordArmHandPose_<ContainerAllocator> const> ConstPtr;

}; // struct recordArmHandPose_

typedef ::dynamic_biped::recordArmHandPose_<std::allocator<void> > recordArmHandPose;

typedef boost::shared_ptr< ::dynamic_biped::recordArmHandPose > recordArmHandPosePtr;
typedef boost::shared_ptr< ::dynamic_biped::recordArmHandPose const> recordArmHandPoseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dynamic_biped::recordArmHandPose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dynamic_biped::recordArmHandPose_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dynamic_biped::recordArmHandPose_<ContainerAllocator1> & lhs, const ::dynamic_biped::recordArmHandPose_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.left_pose == rhs.left_pose &&
    lhs.right_pose == rhs.right_pose;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dynamic_biped::recordArmHandPose_<ContainerAllocator1> & lhs, const ::dynamic_biped::recordArmHandPose_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dynamic_biped

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dynamic_biped::recordArmHandPose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dynamic_biped::recordArmHandPose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamic_biped::recordArmHandPose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamic_biped::recordArmHandPose_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamic_biped::recordArmHandPose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamic_biped::recordArmHandPose_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dynamic_biped::recordArmHandPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ebe76034e32233813bca8e9a0bd81c2c";
  }

  static const char* value(const ::dynamic_biped::recordArmHandPose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xebe76034e3223381ULL;
  static const uint64_t static_value2 = 0x3bca8e9a0bd81c2cULL;
};

template<class ContainerAllocator>
struct DataType< ::dynamic_biped::recordArmHandPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dynamic_biped/recordArmHandPose";
  }

  static const char* value(const ::dynamic_biped::recordArmHandPose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dynamic_biped::recordArmHandPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"armHandPose  left_pose\n"
"armHandPose  right_pose\n"
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
"MSG: dynamic_biped/armHandPose\n"
"float64[3] pos_xyz\n"
"float64[4] quat_xyzw\n"
"\n"
"float64[7] joint_angles\n"
;
  }

  static const char* value(const ::dynamic_biped::recordArmHandPose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dynamic_biped::recordArmHandPose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.left_pose);
      stream.next(m.right_pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct recordArmHandPose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dynamic_biped::recordArmHandPose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dynamic_biped::recordArmHandPose_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "left_pose: ";
    s << std::endl;
    Printer< ::dynamic_biped::armHandPose_<ContainerAllocator> >::stream(s, indent + "  ", v.left_pose);
    s << indent << "right_pose: ";
    s << std::endl;
    Printer< ::dynamic_biped::armHandPose_<ContainerAllocator> >::stream(s, indent + "  ", v.right_pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DYNAMIC_BIPED_MESSAGE_RECORDARMHANDPOSE_H
