#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "Controller.hpp"
#include "main.h"

enum {ID1=0x201, ID2, ID3, ID4, ID5, ID6, ID7, ID8};
enum motor_type {M2006, M3508, M6020};
enum motor_mode {SPD, POS, ACE};
enum {pre = 0, now};


class Motor{
public:
  Motor(const motor_type type, const motor_mode mode, const uint32_t id, PID _speed, PID _position);
	Motor(const motor_type type, const motor_mode mode, const uint32_t id, PID _speed);
  int32_t Set_range(const int32_t original, const int32_t range);
  void Get_limit(motor_type type);
  void Update_output(uint8_t* odata);
  void Change_speed(const int32_t &tar_speed);
  void Change_angle(const int32_t &tar_angle);
  int16_t Get_curspeed(void);
  int16_t Get_curangle(void);
private:
  uint8_t type;
  uint8_t mode;
  uint8_t ID;
  uint8_t ID_frame;

  int16_t curspeed, actual_torque_current, angle[2];
  uint8_t temperature;

  int32_t control_cur, setspeed, setangle;
  int32_t cur_circle, spinning, setcircle;
  uint32_t adjspeed;
  uint16_t maxspeed, maxcurrent;

  PID pid_speed;
  PID pid_position;
};
#endif
