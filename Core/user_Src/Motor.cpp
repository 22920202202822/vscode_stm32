#include "Motor.hpp"
#include "string.h"
#include "algorithm"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "can.h"

extern uint8_t can1_data[8][8];
extern uint8_t can2_data[8][8];
uint8_t output_can1_data[16], output_can2_data[16];
uint32_t cur_cal_value = 0;
Motor can1_motor[8] = {
	Motor(M3508, SPD, ID1, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508, SPD, ID2, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508, SPD, ID3, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508, SPD, ID4, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508, SPD, ID5, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508, SPD, ID6, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508, SPD, ID7, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508, SPD, ID8, PID(10.f, 0.0655f, 3.49e-4f))
};
Motor can2_motor[8] = {
	Motor(M3508, SPD, ID1, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508, SPD, ID2, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508, SPD, ID3, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508, SPD, ID4, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508, SPD, ID5, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508, SPD, ID6, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508, SPD, ID7, PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508, SPD, ID8, PID(10.f, 0.0655f, 3.49e-4f))
};
Motor::Motor(const motor_type type, const motor_mode mode, const uint32_t id, PID _speed, PID _position)
	: ID(id)
	, type(type)
	, mode(mode)
{
	ID_frame = ID - 0x201;
	Get_limit(type);
	memcpy(&pid_speed, &_speed, sizeof(PID));
	memcpy(&pid_position, &_position, sizeof(PID));
}
Motor::Motor(const motor_type type, const motor_mode mode, const uint32_t id, PID _speed)
	: ID(id)
	, type(type)
	, mode(mode)
{
	ID_frame = ID - 0x201;
	Get_limit(type);
	memcpy(&pid_speed, &_speed, sizeof(PID));
}
void Motor::Get_limit(motor_type type) {
	adjspeed = 3000;
	switch (type)
	{
	case M3508:
		maxcurrent = 13000;
		maxspeed = 9000;
		break;
	case M2006:
		maxcurrent = 13000;
		maxspeed = 9000;
		adjspeed = 1000;
		break;
	case M6020:
		maxcurrent = 30000; 
		maxspeed = 200;    
		adjspeed = 80;
		break;
	default:;
	}
}
int32_t Motor::Set_range(const int32_t original, const int32_t range) {
	return std::max(std::min(range, original), -range);
}
void Motor::Update_output(uint8_t* odata){
	// Update current motor status
	angle[now] = (int16_t(can1_data[ID_frame][0]) << 8) + can1_data[ID_frame][1];
	curspeed = (int16_t(can1_data[ID_frame][2]) << 8) + can1_data[ID_frame][3];
	actual_torque_current = (int16_t(can1_data[ID_frame][4]) << 8) + can1_data[ID_frame][5];
	temperature = can1_data[ID_frame][6];
	// Calculate motor output control variable
	if (mode == SPD)
	{
		control_cur += pid_speed.Delta(setspeed - curspeed);
		control_cur = Set_range(control_cur, maxcurrent);
	}
	else if(mode == POS)
	{
		setspeed = pid_position.Position(setangle - angle[now]);
		setspeed = Set_range(setspeed, maxspeed);
		control_cur = pid_speed.Position(setspeed - curspeed);
		control_cur = Set_range(control_cur, maxcurrent);
	}
	else if(mode == ACE)
	{
		
	}
	angle[pre] = angle[now];           //更新角度
	odata[ID_frame * 2] = (control_cur & 0xff00) >> 8;
	odata[ID_frame * 2 + 1] = control_cur & 0x00ff;
}
void Motor::Change_speed(const int32_t &tar_speed)
{
	this->setspeed = tar_speed;
}
void Motor::Change_angle(const int32_t &tar_angle)
{
	this->setangle = tar_angle;
}
int16_t Motor::Get_curspeed(void)
{
	return curspeed;
}
int16_t Motor::Get_curangle(void)
{
	return angle[now];
}
extern "C" {
	void MotorUpdate(void *argument)
	{
		for(;;)
		{
			for(uint8_t i = 0; i < 8; i++)
			{
				can1_motor[i].Update_output(output_can1_data);
				can2_motor[i].Update_output(output_can2_data);
			}
			// switch(cur_cal_value % 4)
			// {
			// 	case 0:
			// 		CAN_Send_Msg(m_CAN1, 0x200, output_can1_data, 8);
			// 		break;
			// 	case 1:
			// 		CAN_Send_Msg(m_CAN2, 0x200, output_can2_data, 8);
			// 		break;
			// 	case 2:
			// 		CAN_Send_Msg(m_CAN1, 0x1ff, output_can1_data + 8, 8);
			// 		break;
			// 	case 3:
			// 		CAN_Send_Msg(m_CAN2, 0x1ff, output_can1_data + 8, 8);
			// 		break;
			// 	default:
			// 		break;
			// }
			cur_cal_value++;
			osDelay(1);
		}
	}
}