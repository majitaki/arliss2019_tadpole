﻿#include "../rover_util/delayed_execution.h"
#include "../rover_util/utils.h"
#include "../rover_util/serial_command.h"
#include "../actuator/motor.h"
#include "../actuator/motor_constant.h"
#include "../constants.h"
#include "../rover_util/logging.h"
//#include "../manager/accel_manager.h"
#include "../sequence/testing_sequence.h"
#include "../sequence/navigating_sequence.h"
#include "../sensor/gps.h"
#include "../sensor/nineaxis.h"
#include "../actuator/servo.h"
//#include "../actuator/servo_constant.h"
#include "./waking_turnside.h"
#include "waking_turnback.h"
#include "waking_turnback_constant.h"

WakingFromTurnBack gWakingFromTurnBack;
bool WakingFromTurnBack::onInit(const timespec & time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[WakingTurnBack] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	//initialize
	mCurStep = STEP_START;
	gMotorDrive.setRunMode(true);
	gServo.setRunMode(true);
    gNineAxisSensor.setRunMode(true);
	mWakeRetryCount = 0;
	mLastUpdateTime = time;
	mChangeFlag = false;
	return true;
}

void WakingFromTurnBack::onUpdate(const timespec & time)
{
	double power;
	const static double WAKING_THRESHOLD = 200;

	switch (mCurStep)
	{
	case STEP_START:
		if (Time::dt(time, mLastUpdateTime) > WAKING_TURN_BACK_TIMEOUT)
		{
			Debug::print(LOG_SUMMARY, "Waking Timeout : unable to spin\r\n");
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0);
		}

		if (gNineAxisSensor.isTurnBack())
		{
			Debug::print(LOG_SUMMARY, "TurnBack Detected : Rotation!\r\n");
			gServo.wrap(1.0);
			gServo.turn(0.0); 
            gMotorDrive.drive(-100);
			mCurStep = STEP_CHANGE;
			mLastUpdateTime = time;
		}
		break;
	case STEP_CHANGE:
		if (Time::dt(time, mLastUpdateTime) > WAKING_TURN_BACK_FORWARD_TIME){
            gMotorDrive.drive(100);
			mCurStep = STEP_VERIFY;
			mLastUpdateTime = time;
		}
		break;

	case STEP_VERIFY:
		if (Time::dt(time, mLastUpdateTime) <= WAKING_TURN_BACK_VERIFY_TIME)
		{
			return;
		}

		if (!gNineAxisSensor.isTurnBack())
		{
			Debug::print(LOG_SUMMARY, "Waking Succeed!\r\n");
			setRunMode(false);
			return;
		}
		else
		{
			mLastUpdateTime = time;
			mCurStep = STEP_START;

			if (++mWakeRetryCount > WAKING_TURN_BACK_RETRY_COUNT)
			{
				Debug::print(LOG_SUMMARY, "Waking Failed!\r\n");
				setRunMode(false);
				return;
			}

			Debug::print(LOG_SUMMARY, "Waking will be retried (%d / %d) by power %f\r\n", mWakeRetryCount, WAKING_TURN_BACK_RETRY_COUNT, power);
		}
		break;
	}
}

bool WakingFromTurnBack::onCommand(const std::vector<std::string>& args)
{
	return false;
}

void WakingFromTurnBack::onClean()
{
	gServo.wrap(0.0);
	gMotorDrive.drive(0);
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[WakingTurnBack] Finished\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
}

WakingFromTurnBack::WakingFromTurnBack()
{
	setName("waking_turnback");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}

WakingFromTurnBack::~WakingFromTurnBack()
{
}
