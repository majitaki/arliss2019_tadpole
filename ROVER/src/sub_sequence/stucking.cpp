#include <cmath>
#include<stdlib.h>
#include "../rover_util/delayed_execution.h"
#include "../rover_util/utils.h"
#include "../rover_util/serial_command.h"
#include "../actuator/motor.h"
#include "../actuator/motor_constant.h"
#include "../constants.h"
//#include "../manager/accel_manager.h"
#include "../actuator/servo.h"
#include "stucking.h"
#include "stucking_constant.h"
#include "../sensor/gps.h"
#include "../sensor/nineaxis.h"
#include "../actuator/servo_constant.h"

Stucking gStucking;
bool Stucking::onInit(const timespec & time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Stucking] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	//initialize
	mLastUpdateTime = time;
	mCheckTime = time;
	mSubState = Initial_Checking;
	mStuckRetryCount = 0;
	//mRandomCount = 0;
	mCheckStuckCount = 0;
	return true;
}

void Stucking::onUpdate(const timespec & time)
{
	double dt = Time::dt(time, mLastUpdateTime);
	if (dt < STUCKING_UPDATE_INTERVAL_TIME)return;
	mLastUpdateTime = time;

	if(gNineAxisSensor.isTurnSide() || gNineAxisSensor.isTurnBack()) 
	{ 
			mSubState = Final;
	}

	switch (mSubState)
	{
	case Initial_Checking:
		dt = Time::dt(time, mCheckTime);
		Debug::print(LOG_SUMMARY, "[Stucking] Initial Checking %1.0f / %1.0f\r\n", dt, STUCKING_CHECKING_TIME);

		if (gGPSSensor.isStuckGPS())
		{
			Debug::print(LOG_SUMMARY, "[Stucking] Initial Checking Failed\r\n");
			mSubState = Initial;
		}
		else
		{
			if (dt > STUCKING_CHECKING_TIME)
			{
				Debug::print(LOG_SUMMARY, "[Stucking] Not Stucking\r\n");
				setRunMode(false);
				return;
			}
		}
	case Initial:
		Debug::print(LOG_SUMMARY, "[Stucking] Initial\r\n");

		if (mStuckRetryCount++ >= STUCKING_RETRY_COUNT)
		{
			mSubState = Final;
			break;
		}
		Debug::print(LOG_SUMMARY, "[Stucking] Retry Count %d / %d\r\n", mStuckRetryCount, STUCKING_RETRY_COUNT);

		mSubState = IncWorm;
		mInchWormCount = 0;
		mCheckTime = time;
		break;
		/*case Random:
		{
			//random servo
			//int rnd_tmp = abs(rand()) % (RELEASE_PULSEWIDTH_PARA_SERVO - HOLD_PULSEWIDTH_PARA_SERVO);
			//int random_servo_pulse = HOLD_PULSEWIDTH_PARA_SERVO + rnd_tmp;

			int rnd_tmp = abs(rand());
			int random_servo_pulse = (rnd_tmp == 0 ? gServo.getRelasePulseWidth() : gServo.getHoldPulseWidth());
			gServo.ParaServo(random_servo_pulse);
			gServo.centerDirect();

			//random motor
			rnd_tmp = rand() % 2;
			int random_motor_sign = rnd_tmp == 0 ? -1 : 1;
			int random_motor_value = MOTOR_MAX_POWER * random_motor_sign;
			gMotorDrive.drive(random_motor_value);

			Debug::print(LOG_SUMMARY, "[Stucking] Random Count %d / %d Servo %d Motor %d\r\n", mRandomCount++, STUCKING_RANDOM_COUNT, random_servo_pulse, random_motor_value);

			mSubState = Random_Checking;
			mCheckTime = time;
			mCheckStuckCount = 0;
			break;
		}
		case Random_Checking:
			dt = Time::dt(time, mCheckTime);
			if (dt < STUCKING_RANDOM_TIME + 1)
			{
				Debug::print(LOG_SUMMARY, "[Stucking] Random %1.1f s / %1.1f s\r\n", dt, STUCKING_RANDOM_TIME);
				break;
			}

			if (gGPSSensor.isStuckGPS())
			{
				Debug::print(LOG_SUMMARY, "[Stucking] Random Checking Failed\r\n");
				mSubState = Random;
			}
			else
			{
				Debug::print(LOG_SUMMARY, "[Stucking] Random Checking %1.1f s / %1.1f s\r\n", dt - STUCKING_RANDOM_TIME, STUCKING_CHECKING_TIME);
				if (dt > STUCKING_RANDOM_TIME + STUCKING_CHECKING_TIME)
				{
					Debug::print(LOG_SUMMARY, "[Stucking] Not Stucking\r\n");
					setRunMode(false);
					return;
				}
			}

			if (mRandomCount > STUCKING_RANDOM_COUNT)
			{
				mSubState = IncWorm;
				mCheckTime = time;
				mInchWormCount = 0;
				break;
			}
			break;*/
	case IncWorm:
		gMotorDrive.drive(100);
		if (mInchWormCount > STUCKING_INCHWORM_COUNT)
		{
			mSubState = Back;
			mCheckTime = time;
			break;
		}
		Debug::print(LOG_SUMMARY, "[Stucking] IncWorm Count %d / %d\r\n", mInchWormCount++, STUCKING_INCHWORM_COUNT);

		mInchWormLoopCount = 0;
		mSubState = IncWorm_Shrink;
		mCheckTime = time;

		break;
	case IncWorm_Shrink:
		//gServo.holdPara();
		gServo.turn(0.0);
		gServo.wrap(1.0);
		//gServo.centerDirect();
		if (mInchWormLoopCount > STUCKING_INCHWORM_LOOP_COUNT)
		{
			mSubState = IncWorm_Checking;
			mCheckTime = time;
			break;
		}
		dt = Time::dt(time, mCheckTime);
		if (dt < STUCKING_INCHWORM_SHRINK_TIME) break;

		Debug::print(LOG_SUMMARY, "[Stucking] IncWorm Loop Count %d / %d\r\n", mInchWormLoopCount++, STUCKING_INCHWORM_LOOP_COUNT);
		Debug::print(LOG_SUMMARY, "[Stucking] IncWorm Shrink Finish\r\n");
		mSubState = IncWorm_Extend;
		mCheckTime = time;
		break;
	case IncWorm_Extend:
		gServo.turn(0.0);
		gServo.wrap(-1.0);
		//gServo.releasePara();
		//gServo.centerDirect();
		dt = Time::dt(time, mCheckTime);
		if (dt < STUCKING_INCHWORM_EXTEND_TIME) break;
		Debug::print(LOG_SUMMARY, "[Stucking] IncWorm Extend Finish\r\n");
		mSubState = IncWorm_Shrink;
		mCheckTime = time;
		break;
	case IncWorm_Checking:
		dt = Time::dt(time, mCheckTime);

		if (gGPSSensor.isStuckGPS())
		{
			Debug::print(LOG_SUMMARY, "[Stucking] IncWorm Checking Failed\r\n");
			mSubState = IncWorm;
			break;
		}
		else
		{
			Debug::print(LOG_SUMMARY, "[Stucking] IncWorm Checking %1.1f s / %1.1f s\r\n", dt, STUCKING_CHECKING_TIME);
			if (dt > STUCKING_CHECKING_TIME)
			{
				Debug::print(LOG_SUMMARY, "[Stucking] Not Stucking\r\n");
				setRunMode(false);
				return;
			}
		}
		break;
	case Back:
		dt = Time::dt(time, mCheckTime);
		Debug::print(LOG_SUMMARY, "[Stucking] Back %1.1f s/ %1.1f s\r\n", dt, STUCKING_BACK_TIME);
		gServo.turn(0.0);
		gServo.wrap(0.0);
		gMotorDrive.drive(-100);
		//gServo.releasePara();
		//gServo.centerDirect();

		if (dt > STUCKING_BACK_TIME)
		{
			//Debug::print(LOG_SUMMARY, "[Stucking] Back Checking\r\n");
			mSubState = Back_Checking;
			mCheckTime = time;
		}
		break;
	case Back_Checking:
	{
		dt = Time::dt(time, mCheckTime);
		Debug::print(LOG_SUMMARY, "[Stucking] Back Checking %1.1f s / %1.1f s\r\n", dt, STUCKING_CHECKING_TIME);

		if (!gGPSSensor.isStuckGPS())
		{
			Debug::print(LOG_SUMMARY, "[Stucking] Back Checking Failed\r\n");
			mSubState = Final;
			mCheckTime = time;
		}
		else
		{
			Debug::print(LOG_SUMMARY, "[Stucking] Back Checking Success\r\n");
			if (dt > STUCKING_CHECKING_TIME)
			{
				Debug::print(LOG_SUMMARY, "[Stucking] Not Stucking\r\n");
				mSubState = Avoid_Stucking;
				return;
			}
		}
		break;
	}
	case Avoid_Stucking:
	{
		Debug::print(LOG_SUMMARY, "[Stucking] Avoid_Stucking \r\n");
		mCheckTime = time;
		gMotorDrive.drive(100);
		if (rand() % 2 == 0){
            //gServo.leftDirect();
            gServo.wrap(0.0);
            gServo.turn(-1.0);
        }
		else{
            gServo.wrap(0.0);
            gServo.turn(1.0);
        }
		//else gServo.rightDirect();
		mSubState = Avoid_Stucking_Checking;
		break;
	}
	case Avoid_Stucking_Checking:
	{
		dt = Time::dt(time, mCheckTime);
		Debug::print(LOG_SUMMARY, "[Stucking] Avoid_Stucking_Checking %fs/%fs \r\n", dt, STUCKING_AVOIDING_TIME);
		if (dt > STUCKING_AVOIDING_TIME)
		{
			Debug::print(LOG_SUMMARY, "[Stucking] Avoid Stucking Completed\r\n");
			setRunMode(false);
			return;
		}
		break;
	}
	case Final:
		gMotorDrive.drive(0);
        gServo.wrap(0.0);
        gServo.turn(0.0);
		//gServo.holdPara();
		//gServo.centerDirect();
		//dt = Time::dt(time, mCheckTime);
		//if (dt < STUCKING_FINAL_INTERVAL_TIME) break;
		Debug::print(LOG_SUMMARY, "[Stucking] Give up\r\n");
		setRunMode(false);
		break;
	}
}

bool Stucking::onCommand(const std::vector<std::string>& args)
{
	return false;
}

void Stucking::onClean()
{
	Debug::print(LOG_SUMMARY, "[Stucking] Finished\r\n");
}

Stucking::Stucking()
{
	setName("stucking");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}

Stucking::~Stucking()
{
}
