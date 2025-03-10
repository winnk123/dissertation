// Generated by gencpp from file multirobot_map_merge/mapPair2tfRequest.msg
// DO NOT EDIT!


#ifndef MULTIROBOT_MAP_MERGE_MESSAGE_MAPPAIR2TFREQUEST_H
#define MULTIROBOT_MAP_MERGE_MESSAGE_MAPPAIR2TFREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/OccupancyGrid.h>

namespace multirobot_map_merge
{
template <class ContainerAllocator>
struct mapPair2tfRequest_
{
  typedef mapPair2tfRequest_<ContainerAllocator> Type;

  mapPair2tfRequest_()
    : grid1()
    , grid2()  {
    }
  mapPair2tfRequest_(const ContainerAllocator& _alloc)
    : grid1(_alloc)
    , grid2(_alloc)  {
  (void)_alloc;
    }



   typedef  ::nav_msgs::OccupancyGrid_<ContainerAllocator>  _grid1_type;
  _grid1_type grid1;

   typedef  ::nav_msgs::OccupancyGrid_<ContainerAllocator>  _grid2_type;
  _grid2_type grid2;





  typedef boost::shared_ptr< ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> const> ConstPtr;

}; // struct mapPair2tfRequest_

typedef ::multirobot_map_merge::mapPair2tfRequest_<std::allocator<void> > mapPair2tfRequest;

typedef boost::shared_ptr< ::multirobot_map_merge::mapPair2tfRequest > mapPair2tfRequestPtr;
typedef boost::shared_ptr< ::multirobot_map_merge::mapPair2tfRequest const> mapPair2tfRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator1> & lhs, const ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator2> & rhs)
{
  return lhs.grid1 == rhs.grid1 &&
    lhs.grid2 == rhs.grid2;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator1> & lhs, const ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace multirobot_map_merge

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8cca8e0556615a0ebc57fe8b159f5410";
  }

  static const char* value(const ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8cca8e0556615a0eULL;
  static const uint64_t static_value2 = 0xbc57fe8b159f5410ULL;
};

template<class ContainerAllocator>
struct DataType< ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "multirobot_map_merge/mapPair2tfRequest";
  }

  static const char* value(const ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nav_msgs/OccupancyGrid grid1\n"
"nav_msgs/OccupancyGrid grid2\n"
"\n"
"================================================================================\n"
"MSG: nav_msgs/OccupancyGrid\n"
"# This represents a 2-D grid map, in which each cell represents the probability of\n"
"# occupancy.\n"
"\n"
"Header header \n"
"\n"
"#MetaData for the map\n"
"MapMetaData info\n"
"\n"
"# The map data, in row-major order, starting with (0,0).  Occupancy\n"
"# probabilities are in the range [0,100].  Unknown is -1.\n"
"int8[] data\n"
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
"MSG: nav_msgs/MapMetaData\n"
"# This hold basic information about the characterists of the OccupancyGrid\n"
"\n"
"# The time at which the map was loaded\n"
"time map_load_time\n"
"# The map resolution [m/cell]\n"
"float32 resolution\n"
"# Map width [cells]\n"
"uint32 width\n"
"# Map height [cells]\n"
"uint32 height\n"
"# The origin of the map [m, m, rad].  This is the real-world pose of the\n"
"# cell (0,0) in the map.\n"
"geometry_msgs/Pose origin\n"
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
;
  }

  static const char* value(const ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.grid1);
      stream.next(m.grid2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct mapPair2tfRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::multirobot_map_merge::mapPair2tfRequest_<ContainerAllocator>& v)
  {
    s << indent << "grid1: ";
    s << std::endl;
    Printer< ::nav_msgs::OccupancyGrid_<ContainerAllocator> >::stream(s, indent + "  ", v.grid1);
    s << indent << "grid2: ";
    s << std::endl;
    Printer< ::nav_msgs::OccupancyGrid_<ContainerAllocator> >::stream(s, indent + "  ", v.grid2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MULTIROBOT_MAP_MERGE_MESSAGE_MAPPAIR2TFREQUEST_H
