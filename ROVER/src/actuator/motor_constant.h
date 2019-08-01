#pragma once

const static double MOTOR_DRIVE_UPDATE_INTERVAL_TIME = 0.01; //second
const static double MOTOR_DRIVE_PID_UPDATE_INTERVAL_TIME = 0.2; //second
//���[�^�[�s��
const static int PIN_PWM_A1 = 24;//���[�^PWM Right
const static int PIN_PWM_A2 = 23;//���[�^���]�s�� Right          
const static int PIN_PWM_B1 = 6;
const static int PIN_PWM_B2 = 5;
//const static int PIN_PWM_C1 = 25;
//const static int PIN_PWM_C2 = 23;//���[�^���]�s�� Right         

								 //���[�^�ݒ�
const static int MOTOR_MAX_POWER = 100;
const static double MOTOR_MAX_POWER_CHANGE = (double)5;//���[�^�o�͂̍ő�ω���
const static double MOTOR_PID_MAX_ANGLE_DIFF = 90;// d_angle in pid is limited to this range
