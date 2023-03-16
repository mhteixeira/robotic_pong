// Generated by gencpp from file niryo_one_msgs/GetWorkspaceRatio.msg
// DO NOT EDIT!


#ifndef NIRYO_ONE_MSGS_MESSAGE_GETWORKSPACERATIO_H
#define NIRYO_ONE_MSGS_MESSAGE_GETWORKSPACERATIO_H

#include <ros/service_traits.h>


#include <niryo_one_msgs/GetWorkspaceRatioRequest.h>
#include <niryo_one_msgs/GetWorkspaceRatioResponse.h>


namespace niryo_one_msgs
{

struct GetWorkspaceRatio
{

typedef GetWorkspaceRatioRequest Request;
typedef GetWorkspaceRatioResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetWorkspaceRatio
} // namespace niryo_one_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::niryo_one_msgs::GetWorkspaceRatio > {
  static const char* value()
  {
    return "ce70a1191ba3e011669c12b7ee6501e1";
  }

  static const char* value(const ::niryo_one_msgs::GetWorkspaceRatio&) { return value(); }
};

template<>
struct DataType< ::niryo_one_msgs::GetWorkspaceRatio > {
  static const char* value()
  {
    return "niryo_one_msgs/GetWorkspaceRatio";
  }

  static const char* value(const ::niryo_one_msgs::GetWorkspaceRatio&) { return value(); }
};


// service_traits::MD5Sum< ::niryo_one_msgs::GetWorkspaceRatioRequest> should match 
// service_traits::MD5Sum< ::niryo_one_msgs::GetWorkspaceRatio > 
template<>
struct MD5Sum< ::niryo_one_msgs::GetWorkspaceRatioRequest>
{
  static const char* value()
  {
    return MD5Sum< ::niryo_one_msgs::GetWorkspaceRatio >::value();
  }
  static const char* value(const ::niryo_one_msgs::GetWorkspaceRatioRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::niryo_one_msgs::GetWorkspaceRatioRequest> should match 
// service_traits::DataType< ::niryo_one_msgs::GetWorkspaceRatio > 
template<>
struct DataType< ::niryo_one_msgs::GetWorkspaceRatioRequest>
{
  static const char* value()
  {
    return DataType< ::niryo_one_msgs::GetWorkspaceRatio >::value();
  }
  static const char* value(const ::niryo_one_msgs::GetWorkspaceRatioRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::niryo_one_msgs::GetWorkspaceRatioResponse> should match 
// service_traits::MD5Sum< ::niryo_one_msgs::GetWorkspaceRatio > 
template<>
struct MD5Sum< ::niryo_one_msgs::GetWorkspaceRatioResponse>
{
  static const char* value()
  {
    return MD5Sum< ::niryo_one_msgs::GetWorkspaceRatio >::value();
  }
  static const char* value(const ::niryo_one_msgs::GetWorkspaceRatioResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::niryo_one_msgs::GetWorkspaceRatioResponse> should match 
// service_traits::DataType< ::niryo_one_msgs::GetWorkspaceRatio > 
template<>
struct DataType< ::niryo_one_msgs::GetWorkspaceRatioResponse>
{
  static const char* value()
  {
    return DataType< ::niryo_one_msgs::GetWorkspaceRatio >::value();
  }
  static const char* value(const ::niryo_one_msgs::GetWorkspaceRatioResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // NIRYO_ONE_MSGS_MESSAGE_GETWORKSPACERATIO_H
