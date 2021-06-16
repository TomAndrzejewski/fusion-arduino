#ifndef _ROS_arduino_msgs_Encoders_h
#define _ROS_arduino_msgs_Encoders_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arduino_msgs
{

  class Encoders : public ros::Msg
  {
    public:
      typedef int16_t _leftEnc_type;
      _leftEnc_type leftEnc;
      typedef int16_t _rightEnc_type;
      _rightEnc_type rightEnc;

    Encoders():
      leftEnc(0),
      rightEnc(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_leftEnc;
      u_leftEnc.real = this->leftEnc;
      *(outbuffer + offset + 0) = (u_leftEnc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leftEnc.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->leftEnc);
      union {
        int16_t real;
        uint16_t base;
      } u_rightEnc;
      u_rightEnc.real = this->rightEnc;
      *(outbuffer + offset + 0) = (u_rightEnc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rightEnc.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rightEnc);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_leftEnc;
      u_leftEnc.base = 0;
      u_leftEnc.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leftEnc.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->leftEnc = u_leftEnc.real;
      offset += sizeof(this->leftEnc);
      union {
        int16_t real;
        uint16_t base;
      } u_rightEnc;
      u_rightEnc.base = 0;
      u_rightEnc.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rightEnc.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rightEnc = u_rightEnc.real;
      offset += sizeof(this->rightEnc);
     return offset;
    }

    virtual const char * getType() override { return "arduino_msgs/Encoders"; };
    virtual const char * getMD5() override { return "1348a2f064c923ebaced7650e8150dc8"; };

  };

}
#endif
