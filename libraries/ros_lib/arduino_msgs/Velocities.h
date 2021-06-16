#ifndef _ROS_arduino_msgs_Velocities_h
#define _ROS_arduino_msgs_Velocities_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arduino_msgs
{

  class Velocities : public ros::Msg
  {
    public:
      typedef int8_t _Vr_type;
      _Vr_type Vr;
      typedef int8_t _Vl_type;
      _Vl_type Vl;

    Velocities():
      Vr(0),
      Vl(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_Vr;
      u_Vr.real = this->Vr;
      *(outbuffer + offset + 0) = (u_Vr.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Vr);
      union {
        int8_t real;
        uint8_t base;
      } u_Vl;
      u_Vl.real = this->Vl;
      *(outbuffer + offset + 0) = (u_Vl.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Vl);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_Vr;
      u_Vr.base = 0;
      u_Vr.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Vr = u_Vr.real;
      offset += sizeof(this->Vr);
      union {
        int8_t real;
        uint8_t base;
      } u_Vl;
      u_Vl.base = 0;
      u_Vl.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Vl = u_Vl.real;
      offset += sizeof(this->Vl);
     return offset;
    }

    virtual const char * getType() override { return "arduino_msgs/Velocities"; };
    virtual const char * getMD5() override { return "17fcf83ee00711f7863bf81b7d810e78"; };

  };

}
#endif
