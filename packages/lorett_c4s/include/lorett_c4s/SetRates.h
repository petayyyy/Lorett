// Generated by gencpp from file lorett_c4s/SetRates.msg
// DO NOT EDIT!


#ifndef LORETT_C4S_MESSAGE_SETRATES_H
#define LORETT_C4S_MESSAGE_SETRATES_H

#include <ros/service_traits.h>


#include <lorett_c4s/SetRatesRequest.h>
#include <lorett_c4s/SetRatesResponse.h>


namespace lorett_c4s
{

struct SetRates
{

typedef SetRatesRequest Request;
typedef SetRatesResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetRates
} // namespace lorett_c4s


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::lorett_c4s::SetRates > {
  static const char* value()
  {
    return "a9cc2408dc007c6dd1f503c73d267539";
  }

  static const char* value(const ::lorett_c4s::SetRates&) { return value(); }
};

template<>
struct DataType< ::lorett_c4s::SetRates > {
  static const char* value()
  {
    return "lorett_c4s/SetRates";
  }

  static const char* value(const ::lorett_c4s::SetRates&) { return value(); }
};


// service_traits::MD5Sum< ::lorett_c4s::SetRatesRequest> should match
// service_traits::MD5Sum< ::lorett_c4s::SetRates >
template<>
struct MD5Sum< ::lorett_c4s::SetRatesRequest>
{
  static const char* value()
  {
    return MD5Sum< ::lorett_c4s::SetRates >::value();
  }
  static const char* value(const ::lorett_c4s::SetRatesRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::lorett_c4s::SetRatesRequest> should match
// service_traits::DataType< ::lorett_c4s::SetRates >
template<>
struct DataType< ::lorett_c4s::SetRatesRequest>
{
  static const char* value()
  {
    return DataType< ::lorett_c4s::SetRates >::value();
  }
  static const char* value(const ::lorett_c4s::SetRatesRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::lorett_c4s::SetRatesResponse> should match
// service_traits::MD5Sum< ::lorett_c4s::SetRates >
template<>
struct MD5Sum< ::lorett_c4s::SetRatesResponse>
{
  static const char* value()
  {
    return MD5Sum< ::lorett_c4s::SetRates >::value();
  }
  static const char* value(const ::lorett_c4s::SetRatesResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::lorett_c4s::SetRatesResponse> should match
// service_traits::DataType< ::lorett_c4s::SetRates >
template<>
struct DataType< ::lorett_c4s::SetRatesResponse>
{
  static const char* value()
  {
    return DataType< ::lorett_c4s::SetRates >::value();
  }
  static const char* value(const ::lorett_c4s::SetRatesResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // LORETT_C4S_MESSAGE_SETRATES_H
