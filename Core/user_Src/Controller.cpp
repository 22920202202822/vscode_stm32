#include "Controller.hpp"
#include "algorithm"
float PID::Delta(float error){
  m_error[LLAST] = m_error[LAST] * 0.92f;
	m_error[LAST] = m_error[NOW] * 0.92f;
	m_error[NOW] = error * 1.08f;

	return m_Kp * (m_error[NOW] - m_error[LAST]) + m_Ti * m_error[NOW] + m_Td * (m_error[NOW] - 2 * m_error[LAST] + m_error[LLAST]);
}
float PID::Position(float error){
  m_error[NOW] = error;
	m_error[INTEGRATE] += m_error[NOW];
	m_error[INTEGRATE] = std::max(std::min(m_error[INTEGRATE], max_integrate), -max_integrate);
	//不完全微分
	this->m_lderivative = m_Td * (1.f - m_alpha) * (m_error[NOW] - m_error[LAST]) + m_alpha * m_lderivative;
	const float result = this->m_error[NOW] * this->m_Kp + this->m_error[INTEGRATE] * this->m_Ti + this->m_lderivative;
	m_error[LAST] = m_error[NOW];
	return result;
}

