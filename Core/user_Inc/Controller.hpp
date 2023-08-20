#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include "main.h"
enum{INTEGRATE = 0, LLAST = 0, LAST = 1, NOW = 2};
class PID
{
  public:
    PID(void)
		: m_Kp(0.f)
		, m_Ti(0.f)
		, m_Td(0.f), m_alpha(0.f) {max_integrate = 1000.f; }
	PID(float Kp, float Ti, float Td, float alpha = 0.0f)
		: m_Kp(Kp)
		, m_Ti(Ti)
		, m_Td(Td), m_alpha(alpha) {}  
  float Delta(float error);
  float Position(float error);
  private:
    float m_Kp, m_Ti, m_Td, m_alpha;
    float max_integrate;
    float m_error[3];
    float m_lderivative;
};

#endif
