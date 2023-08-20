#include "Robot.hpp"
#include "math.h"
const float PI = 3.1415926;
extern remote m_remote;
extern Motor can1_motor[8];
extern Motor can2_motor[8];
extern imu m_imu;
void Robot::Init(void)
{
  // Initialize the indexs
  p_remote = &m_remote;
  p_imu = &m_imu;

  this->chassis.Init();
}
void Robot::Chassis::Init(void)
{
  // Initialize the indexs of motors
  for(int i = 0; i < 4; i++)
  {
    this->LfRfRbLb[i] = &can1_motor[i];
  }
  // Initialize the chassis ramp step
  this->Ramp_step = 15.f;
}
void Robot::Chassis::Update(void)
{
  LfRfRbLb[0]->Change_speed(Ramp(LfRfRbLb[0]->Get_curspeed(), +speedx + speedy + speedz));
	LfRfRbLb[1]->Change_speed(Ramp(LfRfRbLb[1]->Get_curspeed(), -speedx + speedy + speedz));
	LfRfRbLb[2]->Change_speed(Ramp(LfRfRbLb[2]->Get_curspeed(), -speedx - speedy + speedz));
	LfRfRbLb[3]->Change_speed(Ramp(LfRfRbLb[3]->Get_curspeed(), +speedx - speedy + speedz));
}
int16_t Robot::Chassis::Ramp(const int32_t &cur, const int32_t &tar)
{
  if(cur - Ramp_step >= tar)
    return cur - Ramp_step;
  else if(cur + Ramp_step <= tar)
    return cur + Ramp_step;
  else
    return cur;
}
void Robot::Chassis_toward_gimbal(int32_t &speedx, int32_t &speedy, int32_t &speedz)
{

}
void Robot::Chassis_follow_gimbal(int32_t &speedx, int32_t &speedy, int32_t &speedz)
{
  
  float theta = GetDelta(mechanicalToDegree(gimbal.yaw->Get_curangle()) - mechanicalToDegree(gimbal.mid_yaw)) * PI;
  float sint = std::sin(theta);
  float cost = std::cos(theta);
  this->chassis.speedx = speedx * cost - speedy * sint;
  this->chassis.speedy = speedx * sint + speedy * cost;
  this->chassis.speedz = speedz;
}
void Robot::Gimbal::Update(void)
{

}
void Robot::Shooter::Update(void)
{

}
float Robot::GetDelta(const float &delta)
{
  if(delta <= -180.f)
    return delta + 360.f;
  else if(delta > 180.f)
    return delta - 360.f;
  else 
    return 0.f;
}

