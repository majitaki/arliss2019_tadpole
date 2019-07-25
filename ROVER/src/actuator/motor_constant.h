#pragma once

const static double MOTOR_DRIVE_UPDATE_INTERVAL_TIME = 0.01; //second
const static double MOTOR_DRIVE_PID_UPDATE_INTERVAL_TIME = 0.2; //second
//モーターピン
const static int PIN_PWM_A1 = 24;//モータPWM Right
const static int PIN_PWM_A2 = 23;//モータ反転ピン Right          
const static int PIN_PWM_B1 = 6;
const static int PIN_PWM_B2 = 5;
//const static int PIN_PWM_C1 = 25;
//const static int PIN_PWM_C2 = 23;//モータ反転ピン Right         

								 //モータ設定
const static int MOTOR_MAX_POWER = 100;
const static double MOTOR_MAX_POWER_CHANGE = (double)5;//モータ出力の最大変化量
const static double MOTOR_PID_MAX_ANGLE_DIFF = 90;// d_angle in pid is limited to this range
