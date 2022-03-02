#ifndef _SERVO_HPP_
#define _SERVO_HPP_

#include "FaBoPWM_PCA9685.h"

typedef struct Motor_slot
{
	uint8_t Control_PinA;
	uint8_t Control_PinB;
	uint8_t Control_PWM;
};

Motor_slot Motor_A = { 0,1,0 };

enum direction
{
	STOP = 0,
	RUN ,
	BAKE_OFF,
};

MeDCMotor::MeDCMotor(Motor_slot dir_motor)
{

}

class MeDCMotor
{
public:

	MeDCMotor(Motor_slot dir_motor);

	void SetMotor(uint8_t dir_motor, uint16_t pwm);

	void ResetMotor(uint8_t dir_motor);

	void run(uint8_t dir,int16_t pwm);

	void stop(void);
private:
	volatile uint8_t dc_dir_pin_A;
	volatile uint8_t dc_dir_pin_B;
	int16_t  last_speed;
	int16_t  fast_speed;
};





#endif
