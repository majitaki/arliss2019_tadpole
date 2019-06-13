#include <cmath>
#include "../rover_util/delayed_execution.h"
#include "../rover_util/utils.h"
#include "../rover_util/serial_command.h"
#include "../pwm/motor.h"
#include "../pwm/motor_constant.h"
#include "../constants.h"
#include "../manager/accel_manager.h"
#include "../pwm/servo.h"
#include "waking_turnside.h"
#include "waking_turnside_constant.h"

WakingFromTurnSide gWakingFromTurnSide;
bool WakingFromTurnSide::onInit(const timespec & time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[WakingTurnSide] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	//initialize
	mLastUpdateTime = time;
	mSubState = Rolling;
	mCurrentPower = 30;
	mWakeRetryCount = 0;
	mWakePID = PID(MOTOR_DRIVE_PID_UPDATE_INTERVAL_TIME, 10, -10, 0.1, 0.01, 0);
	return true;
}

void WakingFromTurnSide::onUpdate(const timespec & time)
{
	double dt = Time::dt(time, mLastUpdateTime);
	if (dt < WAKING_TURN_SIDE_UPDATE_INTERVAL_TIME)return;
	mLastUpdateTime = time;

	switch (mSubState)
	{
	case Rolling:
		Debug::print(LOG_SUMMARY, "[WakingTurnSide] Rolling\r\n");
		gMotorDrive.drive(MOTOR_MAX_POWER, mWakePID);
		gServo.releasePara();
		if (mCurrentPower > MOTOR_MAX_POWER || !gAccelManager.isTurnSide())
		{
			gMotorDrive.drive(0);
			mSubState = Checking;
			mCheckTime = time;
			break;
		}
		//mCurrentPower = WAKING_TURN_SIDE_SPEED_UP_PERIOD * mCurrentPower;
		mCurrentPower = gMotorDrive.getPowerR();
		Debug::print(LOG_SUMMARY, "[WakingTurnSide] %f/%d\r\n", mCurrentPower, MOTOR_MAX_POWER);
		break;
	case Checking:
		Debug::print(LOG_SUMMARY, "[WakingTurnSide] Checking\r\n");
		if (Time::dt(time, mCheckTime) > 3)
		{
			if (!gAccelManager.isTurnSide())
			{
				Debug::print(LOG_SUMMARY, "[WakingTurnSide] Successed\r\n");
				setRunMode(false);
				return;
			}

			if (mWakeRetryCount++ >= WAKING_TURN_SIDE_RETRY_COUNT)
			{
				setRunMode(false);
				return;
			}
			Debug::print(LOG_SUMMARY, "[WakingTurnSide] Retry Count %d\r\n", mWakeRetryCount);
			mSubState = Rolling;
			mCurrentPower = 30;
		}
		break;
	}
}

bool WakingFromTurnSide::onCommand(const std::vector<std::string>& args)
{
	return false;
}

void WakingFromTurnSide::onClean()
{
	gMotorDrive.drive(0);
	gServo.holdPara();
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[WakingTurnSide] Finished\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");

}

WakingFromTurnSide::WakingFromTurnSide():mWakePID()
{
	setName("waking_turnside");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}

WakingFromTurnSide::~WakingFromTurnSide()
{
}
