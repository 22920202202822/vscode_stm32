#ifndef ROBOT_HPP
#define ROBOT_HPP
#include "main.h"
#include "Data_container.h"
#include "Motor.hpp"
// angle transform
#define degreeToMechanical(a) ((a)*8192.f/360.f)
#define mechanicalToDegree(a) ((a)*360.f/8192.f)
class Robot{
public:
  void Init(void);
  void Chassis_toward_gimbal(int32_t &speedx, int32_t &speedy, int32_t &speedz);
  void Chassis_follow_gimbal(int32_t &speedx, int32_t &speedy, int32_t &speedz);
  
  float GetDelta(const float &delta);
private:
  remote *p_remote;
  imu *p_imu;

  typedef struct{
    void Init(void);
    void Update(void);
    int16_t Ramp(const int32_t &cur, const int32_t &tar);

    Motor *LfRfRbLb[4];
    int32_t speedx, speedy, speedz;
    uint8_t Ramp_step;
  }Chassis;

  typedef struct{
    enum {MACHINE, IMU}; // 云台控制方式
    void Init(void);
    void Update(void);
    Motor *pitch, *yaw;
    uint32_t mid_yaw, mid_pitch;
    uint32_t max_pitch, min_pitch;
    uint32_t Ramp_step;
  }Gimbal;

  typedef struct{
    void Init(void);
    void Update(void);
    Motor *rubber[2], *pinwheel; // 摩擦轮和拨弹轮
  }Shooter;

  Chassis chassis;
  Gimbal gimbal;
  Shooter shooter;

};

#endif
