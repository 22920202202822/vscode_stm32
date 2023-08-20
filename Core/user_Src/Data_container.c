#include "Data_container.h"
#include "string.h"   // for memcpy function
//******Variable Definition******//
remote m_remote;
// 用于发送电机控制电流数据的数组缓冲区
uint8_t can1_data[8][8];
uint8_t can2_data[8][8];
imu m_imu;
judgement m_judgement;
float_byte temp_float2byte; // 作为byte转float缓冲区
u32_byte temp_u32byte;
//******Function Implementation******//
//*** remote ***//
void Remote_Decode(uint8_t* m_frame){
  if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;

		m_remote.m_rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
		m_remote.m_rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
		m_remote.m_rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
		m_remote.m_rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
		if (m_remote.m_rc.ch[0] <= 8 && m_remote.m_rc.ch[0] >= -8)m_remote.m_rc.ch[0] = 0;
		if (m_remote.m_rc.ch[1] <= 8 && m_remote.m_rc.ch[1] >= -8)m_remote.m_rc.ch[1] = 0;
		if (m_remote.m_rc.ch[2] <= 8 && m_remote.m_rc.ch[2] >= -8)m_remote.m_rc.ch[2] = 0;
		if (m_remote.m_rc.ch[3] <= 8 && m_remote.m_rc.ch[3] >= -8)m_remote.m_rc.ch[3] = 0;

		m_remote.m_rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
		m_remote.m_rc.s[1] = ((m_frame[5] >> 4) & 0x03);

		m_remote.m_pc.x = m_frame[6] | (m_frame[7] << 8);
		m_remote.m_pc.y = m_frame[8] | (m_frame[9] << 8);
		m_remote.m_pc.z = m_frame[10] | (m_frame[11] << 8);
		m_remote.m_pc.press_l = m_frame[12];
		m_remote.m_pc.press_r = m_frame[13];

		m_remote.m_pc.key_h = m_frame[15];
		m_remote.m_pc.key_l = m_frame[14];
}
//*** imu ***//
// 其中len为不包含SUM字段的数据帧长度
bool Imu_Check_Sum(uint8_t *data, uint32_t len)
{
	uint8_t check = data[len];
	for(int i = 0; i < len; i++)
	{
		check -= data[i];
	}
	return check == 0;
}
void Imu_Decode(uint8_t *m_frame)
{
	if(m_frame[0] != 0x55 || m_frame[1] != 0x55)
		return;
	switch(m_frame[2])
	{
		case 0x01:
			if(Imu_Check_Sum(m_frame, 10))
			{
				m_imu.angle_raw = (float)((int16_t)(m_frame[5] << 8) | m_frame[4]) / 32768 * 180;
				m_imu.angle_pitch = (float)((int16_t)(m_frame[7] << 8) | m_frame[6]) / 32768 * 180;
				m_imu.angle_yaw = (float)((int16_t)(m_frame[9] << 8) | m_frame[8]) / 32768 * 180;
			}
			break;
		case 0x02:
			if(Imu_Check_Sum(m_frame, 12))
			{
				m_imu.quaternion[0] = (float)((int16_t)(m_frame[5] << 8) | m_frame[4]) / 32768;
				m_imu.quaternion[1] = (float)((int16_t)(m_frame[7] << 8) | m_frame[6]) / 32768;
				m_imu.quaternion[2] = (float)((int16_t)(m_frame[9] << 8) | m_frame[8]) / 32768;
				m_imu.quaternion[3] = (float)((int16_t)(m_frame[11] << 8) | m_frame[10]) / 32768;
			}
			break;
		case 0x03:
			if(Imu_Check_Sum(m_frame, 16))
			{
				m_imu.acc[0] = (float)((int16_t)(m_frame[5] << 8) | m_frame[4]) / 32768 * 4 * 9.8;
				m_imu.acc[1] = (float)((int16_t)(m_frame[7] << 8) | m_frame[6]) / 32768 * 4 * 9.8;
				m_imu.acc[2] = (float)((int16_t)(m_frame[9] << 8) | m_frame[8]) / 32768 * 4 * 9.8;
				m_imu.gyro[0] = (float)((int16_t)(m_frame[11] << 8) | m_frame[10]) / 32768 * 2000;
				m_imu.gyro[1] = (float)((int16_t)(m_frame[13] << 8) | m_frame[12]) / 32768 * 2000;
				m_imu.gyro[2] = (float)((int16_t)(m_frame[15] << 8) | m_frame[14]) / 32768 * 2000;
			}
			break;
		default:
			break;
	}
}
// 使用memcpy将会采用小端序（数据低位存储于低地址），而传输协议是大端序，无法直接使用memcpy函数
void Judgement_Decode(uint16_t cmd_id, uint8_t *m_frame)
{
	switch (cmd_id)
	{
	case 0x0001:
		m_judgement.ext_game_status_t.game_type = m_frame[0] & 0x0f;
		m_judgement.ext_game_status_t.game_progress = m_frame[0] >> 4;
		m_judgement.ext_game_status_t.stage_remain_time = ((uint16_t)m_frame[1] << 8) | m_frame[2];
		break;
	case 0x0002:
		m_judgement.ext_game_result_t.winner = m_frame[0];
		break;
	case 0x0003:
		m_judgement.ext_game_robot_HP_t.red_1_robot_HP = ((uint16_t)m_frame[0] << 8) | m_frame[1];
		m_judgement.ext_game_robot_HP_t.red_2_robot_HP = ((uint16_t)m_frame[2] << 8) | m_frame[3];
		m_judgement.ext_game_robot_HP_t.red_3_robot_HP = ((uint16_t)m_frame[4] << 8) | m_frame[5];
		m_judgement.ext_game_robot_HP_t.red_4_robot_HP = ((uint16_t)m_frame[6] << 8) | m_frame[7];
		m_judgement.ext_game_robot_HP_t.red_5_robot_HP = ((uint16_t)m_frame[8] << 8) | m_frame[9];
		m_judgement.ext_game_robot_HP_t.red_7_robot_HP = ((uint16_t)m_frame[10] << 8) | m_frame[11];
		m_judgement.ext_game_robot_HP_t.red_outpost_HP = ((uint16_t)m_frame[12] << 8) | m_frame[13];
		m_judgement.ext_game_robot_HP_t.red_base_HP = ((uint16_t)m_frame[14] << 8) | m_frame[15];
		m_judgement.ext_game_robot_HP_t.blue_1_robot_HP = ((uint16_t)m_frame[16] << 8) | m_frame[17];
		m_judgement.ext_game_robot_HP_t.blue_2_robot_HP = ((uint16_t)m_frame[18] << 8) | m_frame[19];
		m_judgement.ext_game_robot_HP_t.blue_3_robot_HP = ((uint16_t)m_frame[20] << 8) | m_frame[21];
		m_judgement.ext_game_robot_HP_t.blue_4_robot_HP = ((uint16_t)m_frame[22] << 8) | m_frame[23];
		m_judgement.ext_game_robot_HP_t.blue_5_robot_HP = ((uint16_t)m_frame[24] << 8) | m_frame[25];
		m_judgement.ext_game_robot_HP_t.blue_7_robot_HP = ((uint16_t)m_frame[26] << 8) | m_frame[27];
		m_judgement.ext_game_robot_HP_t.blue_outpost_HP = ((uint16_t)m_frame[28] << 8) | m_frame[29];
		m_judgement.ext_game_robot_HP_t.blue_base_HP = ((uint16_t)m_frame[30] << 8) | m_frame[31];
		break;
	case 0x0101:
		m_judgement.ext_event_data_t.event_data = Byte2U32(m_frame);
		//memcpy(&m_judgement.ext_event_data_t.event_data, m_frame, sizeof(uint32_t));
		break;
	case 0x0102:
		m_judgement.ext_supply_projectile_action_t.supply_projectile_id = m_frame[0];
		m_judgement.ext_supply_projectile_action_t.supply_robot_id = m_frame[1];
		m_judgement.ext_supply_projectile_action_t.supply_projectile_step = m_frame[2];
		m_judgement.ext_supply_projectile_action_t.supply_projectile_num = m_frame[3];
		break;
	case 0x0104:
		m_judgement.ext_referee_warning_t.level = m_frame[0];
		m_judgement.ext_referee_warning_t.offending_robot_id = m_frame[1]; 
		break;
	case 0x0105:
		m_judgement.ext_dart_remaining_time_t.dart_remaining_time = m_frame[0];
		break;
	case 0x0201:
		m_judgement.ext_game_robot_status_t.robot_id = m_frame[0];
		m_judgement.ext_game_robot_status_t.robot_level = m_frame[1];
		m_judgement.ext_game_robot_status_t.remain_HP = ((uint16_t)m_frame[2] << 8) | m_frame[3];
		m_judgement.ext_game_robot_status_t.max_HP = ((uint16_t)m_frame[4] << 8) | m_frame[5];
		m_judgement.ext_game_robot_status_t.shooter_id1_17mm_cooling_rate = ((uint16_t)m_frame[6] << 8) | m_frame[7];
		m_judgement.ext_game_robot_status_t.shooter_id1_17mm_cooling_limit = ((uint16_t)m_frame[8] << 8) | m_frame[9];
		m_judgement.ext_game_robot_status_t.shooter_id1_17mm_speed_limit = ((uint16_t)m_frame[10] << 8) | m_frame[11];
		m_judgement.ext_game_robot_status_t.shooter_id2_17mm_cooling_rate = ((uint16_t)m_frame[12] << 8) | m_frame[13];
		m_judgement.ext_game_robot_status_t.shooter_id2_17mm_cooling_limit = ((uint16_t)m_frame[14] << 8) | m_frame[15];
		m_judgement.ext_game_robot_status_t.shooter_id2_17mm_speed_limit = ((uint16_t)m_frame[16] << 8) | m_frame[17];
		m_judgement.ext_game_robot_status_t.shooter_id1_42mm_cooling_rate = ((uint16_t)m_frame[18] << 8) | m_frame[19];
		m_judgement.ext_game_robot_status_t.shooter_id1_42mm_cooling_limit = ((uint16_t)m_frame[20] << 8) | m_frame[21];
		m_judgement.ext_game_robot_status_t.shooter_id1_42mm_speed_limit = ((uint16_t)m_frame[22] << 8) | m_frame[23];
		m_judgement.ext_game_robot_status_t.chassis_power_limit = ((uint16_t)m_frame[24] << 8) | m_frame[25];
		m_judgement.ext_game_robot_status_t.mains_power_gimbal_output = m_frame[26] & 0x01;
		m_judgement.ext_game_robot_status_t.mains_power_chassis_output = (m_frame[26] >> 1) & 0x01;
		m_judgement.ext_game_robot_status_t.mains_power_shooter_output = (m_frame[26] >> 2) & 0x01;
		break;
	case 0x0202:
		m_judgement.ext_power_heat_data_t.chassis_voltage = ((uint16_t)m_frame[0] << 8) | m_frame[1];
		m_judgement.ext_power_heat_data_t.chassis_current = ((uint16_t)m_frame[2] << 8) | m_frame[3];
		m_judgement.ext_power_heat_data_t.chassis_power = Byte2Float(m_frame+4);
		m_judgement.ext_power_heat_data_t.chassis_power_buffer = ((uint16_t)m_frame[5] << 8) | m_frame[6];
		m_judgement.ext_power_heat_data_t.shooter_id1_17mm_cooling_heat = ((uint16_t)m_frame[7] << 8) | m_frame[8];
		m_judgement.ext_power_heat_data_t.shooter_id2_17mm_cooling_heat = ((uint16_t)m_frame[9] << 8) | m_frame[10];
		m_judgement.ext_power_heat_data_t.shooter_id1_42mm_cooling_heat = ((uint16_t)m_frame[11] << 8) | m_frame[12];
		break;
	case 0x0203:
		m_judgement.ext_game_robot_pos_t.x = Byte2Float(m_frame);
		m_judgement.ext_game_robot_pos_t.y = Byte2Float(m_frame+4);
		m_judgement.ext_game_robot_pos_t.z = Byte2Float(m_frame+8);
		m_judgement.ext_game_robot_pos_t.yaw = Byte2Float(m_frame+12);
		break;
	case 0x0204:
		m_judgement.ext_buff_t.recovery_buff = m_frame[0];
		m_judgement.ext_buff_t.cooling_buff = m_frame[1];
		m_judgement.ext_buff_t.defence_buff = m_frame[2];
		m_judgement.ext_buff_t.attack_buff = ((uint16_t)m_frame[3] << 8) | m_frame[4];
		break;
	case 0x0205:
		m_judgement.air_support_data_t.airforce_status = m_frame[0];
		m_judgement.air_support_data_t.time_remain = m_frame[1];
		break;
	case 0x0206:
		m_judgement.ext_robot_hurt_t.armor_id = m_frame[0] & 0x0f;
		m_judgement.ext_robot_hurt_t.hurt_type = m_frame[0] >> 4;
		break;
	case 0x0207:
		m_judgement.ext_shoot_data_t.bullet_type = m_frame[0];
		m_judgement.ext_shoot_data_t.shooter_number = m_frame[1];
		m_judgement.ext_shoot_data_t.launching_frequency = m_frame[2];
		m_judgement.ext_shoot_data_t.initial_speed = Byte2Float(m_frame+3);
		break;
	case 0x0208:
		m_judgement.projectile_allowance_t.projectile_allowance_17mm = ((uint16_t)m_frame[0] << 8) | m_frame[1];
		m_judgement.projectile_allowance_t.projectile_allowance_42mm = ((uint16_t)m_frame[2] << 8) | m_frame[3];
		m_judgement.projectile_allowance_t.remaining_gold_coin = ((uint16_t)m_frame[4] << 8) | m_frame[5];
		break;
	case 0x0209:
		m_judgement.ext_rfid_status_t.rfid_status = Byte2U32(m_frame);
		break;
	case 0x020A:
		m_judgement.ext_dart_client_cmd_t.dart_launch_opening_status = m_frame[0];
		m_judgement.ext_dart_client_cmd_t.dart_attack_target = m_frame[1];
		m_judgement.ext_dart_client_cmd_t.target_change_time = ((uint16_t)m_frame[2] << 8) | m_frame[3];
		m_judgement.ext_dart_client_cmd_t.latest_launch_cmd_time = ((uint16_t)m_frame[4] << 8) | m_frame[5];
		break;
	case 0x020B:
		m_judgement.ground_robot_position_t.hero_x = Byte2Float(m_frame);
		m_judgement.ground_robot_position_t.hero_y = Byte2Float(m_frame+4);
		m_judgement.ground_robot_position_t.engineer_x = Byte2Float(m_frame+8);
		m_judgement.ground_robot_position_t.engineer_y = Byte2Float(m_frame+12);
		m_judgement.ground_robot_position_t.standard_3_x = Byte2Float(m_frame+16);
		m_judgement.ground_robot_position_t.standard_3_y = Byte2Float(m_frame+20);
		m_judgement.ground_robot_position_t.standard_4_x = Byte2Float(m_frame+24);
		m_judgement.ground_robot_position_t.standard_4_y = Byte2Float(m_frame+28);
		m_judgement.ground_robot_position_t.standard_5_x = Byte2Float(m_frame+32);
		m_judgement.ground_robot_position_t.standard_5_y = Byte2Float(m_frame+36);
		break;
	case 0x020C:
		m_judgement.radar_mark_data_t.mark_hero_progress = m_frame[0];
		m_judgement.radar_mark_data_t.mark_engineer_progress = m_frame[1];
		m_judgement.radar_mark_data_t.mark_standard_3_progress = m_frame[2];
		m_judgement.radar_mark_data_t.mark_standard_4_progress = m_frame[3];
		m_judgement.radar_mark_data_t.mark_standard_5_progress = m_frame[4];
		m_judgement.radar_mark_data_t.mark_sentry_progress = m_frame[5];
		break;
	default:
		break;
	}
}
// 获取各种位域标记状态
uint16_t Get_event_data(uint8_t tag)
{
	if(tag < 9)
	{
		return (m_judgement.ext_event_data_t.event_data >> tag) & 0x01;
	}
	else
	{
		return m_judgement.ext_event_data_t.event_data >> tag & 0x0f;
	}
}
float Byte2Float(uint8_t* byte)
{
	memcpy(temp_float2byte.byte, byte, sizeof(float));
	return temp_float2byte.value;
}
uint32_t Byte2U32(uint8_t* byte)
{
	memcpy(temp_u32byte.byte, byte, sizeof(uint32_t));
	return temp_u32byte.value;
}
