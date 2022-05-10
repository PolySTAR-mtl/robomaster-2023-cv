// Generated by gencpp from file detection/Detections.msg
// DO NOT EDIT!


#ifndef DETECTION_MESSAGE_DETECTIONS_H
#define DETECTION_MESSAGE_DETECTIONS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <detection/Detection.h>

namespace detection
{
template <class ContainerAllocator>
struct Detections_
{
  typedef Detections_<ContainerAllocator> Type;

  Detections_()
    : detections()  {
    }
  Detections_(const ContainerAllocator& _alloc)
    : detections(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::detection::Detection_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::detection::Detection_<ContainerAllocator> >::other >  _detections_type;
  _detections_type detections;





  typedef boost::shared_ptr< ::detection::Detections_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::detection::Detections_<ContainerAllocator> const> ConstPtr;

}; // struct Detections_

typedef ::detection::Detections_<std::allocator<void> > Detections;

typedef boost::shared_ptr< ::detection::Detections > DetectionsPtr;
typedef boost::shared_ptr< ::detection::Detections const> DetectionsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::detection::Detections_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::detection::Detections_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::detection::Detections_<ContainerAllocator1> & lhs, const ::detection::Detections_<ContainerAllocator2> & rhs)
{
  return lhs.detections == rhs.detections;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::detection::Detections_<ContainerAllocator1> & lhs, const ::detection::Detections_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace detection

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::detection::Detections_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::detection::Detections_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::detection::Detections_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::detection::Detections_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::detection::Detections_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::detection::Detections_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::detection::Detections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cafb60d89a040a540def8d31f5cdc037";
  }

  static const char* value(const ::detection::Detections_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcafb60d89a040a54ULL;
  static const uint64_t static_value2 = 0x0def8d31f5cdc037ULL;
};

template<class ContainerAllocator>
struct DataType< ::detection::Detections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "detection/Detections";
  }

  static const char* value(const ::detection::Detections_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::detection::Detections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Detections.msg\n"
"## List of detected bounding boxes\n"
"\n"
"# Header header\n"
"Detection[] detections\n"
"================================================================================\n"
"MSG: detection/Detection\n"
"# Detection.msg\n"
"## Bounding box with class and confidence\n"
"\n"
"# Bounding box\n"
"float32 x\n"
"float32 y\n"
"float32 w\n"
"float32 h\n"
"\n"
"# class\n"
"uint8 clss\n"
"\n"
"float32 score\n"
;
  }

  static const char* value(const ::detection::Detections_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::detection::Detections_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.detections);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Detections_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::detection::Detections_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::detection::Detections_<ContainerAllocator>& v)
  {
    s << indent << "detections[]" << std::endl;
    for (size_t i = 0; i < v.detections.size(); ++i)
    {
      s << indent << "  detections[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::detection::Detection_<ContainerAllocator> >::stream(s, indent + "    ", v.detections[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DETECTION_MESSAGE_DETECTIONS_H
