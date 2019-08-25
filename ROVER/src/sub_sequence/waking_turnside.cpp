#include <cmath>
#include "../rover_util/delayed_execution.h"
#include "../rover_util/utils.h"
#include "../rover_util/serial_command.h"
#include "../actuator/motor.h"
#include "../actuator/motor_constant.h"
#include "../constants.h"
//#include "../manager/accel_manager.h"
#include "../sensor/nineaxis.h"
#include "../actuator/servo.h"
#include "../actuator/servo_constant.h"
#include "waking_turnside.h"
#include "waking_turnside_constant.h"

WakingFromTurnSide gWakingFromTurnSide;
bool WakingFromTurnSide::onInit(const timespec & time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[WakingTurnSide] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	mLastUpdateTime = time;
	mSubState = CheckTurnSide;
	mCheckTime = time;
	mBridging = false;
	mCheckFlag = false;
	mCurrentPower = 30;
	mWakeRetryCount = 0;
	return true;
}

void WakingFromTurnSide::onUpdate(const timespec & time)
{
	double dt = Time::dt(time, mLastUpdateTime);
	if (dt < WAKING_TURN_SIDE_UPDATE_INTERVAL_TIME)return;
	mLastUpdateTime = time;

	if(!gNineAxisSensor.isTurnSide() && mSubState != Checking){
		Debug::print(LOG_SUMMARY, "[WakingTurnSide] Entry Checking\r\n");
		mSubState = Checking;
	}

	switch (mSubState)
	{
	case CheckTurnSide:
        gMotorDrive.drive(0);
        gServo.wrap(0);
        gServo.turn(0);

		mTurnSideDirection = gNineAxisSensor.getTurnSideDirection();
		//right
        if(mTurnSideDirection== Right){
			//gServo.turn(-1 * WAKING_TURN_SIDE_MAX_SIDE);
			Debug::print(LOG_SUMMARY, "[WakingTurnSide] Check Turn Side: Right\r\n");
		}
        else if(mTurnSideDirection == Left){
			//gServo.turn(WAKING_TURN_SIDE_MAX_SIDE);
			Debug::print(LOG_SUMMARY, "[WakingTurnSide] Check Turn Side: Left\r\n");
		}

		mSubState = Rolling;
		mCheckTime = time;
		break;
	case Rolling:
		Debug::print(LOG_SUMMARY, "[WakingTurnSide] Rolling\r\n");
        gServo.wrap(0);
        gServo.turn(0);
        gMotorDrive.drive(100);

		if (Time::dt(time, mCheckTime) > WAKING_TURN_SIDE_ONLY_ROLLING_TIME)
		{
			mSubState = BendRolling;
			mCheckTime = time;
		}
		break;
	
	case BendRolling:
		Debug::print(LOG_SUMMARY, "[WakingTurnSide] BendRolling\r\n");
        gMotorDrive.drive(100);
        gServo.wrap(0);

        if(mTurnSideDirection== Right){
			gServo.turn(-1 * WAKING_TURN_SIDE_MAX_SIDE);
		}
        else if(mTurnSideDirection == Left){
			gServo.turn(WAKING_TURN_SIDE_MAX_SIDE);
		}


		if (Time::dt(time, mCheckTime) > WAKING_TURN_SIDE_BEND_ROLLING_TIME)
		{
            gMotorDrive.drive(0);
			gServo.turn(0);
			gServo.wrap(0);
			mSubState = Checking;
			mCheckTime = time;
		}
		break;
	// case Bridging:
	// 	TurnSideDirection turn_side_state;
	// 	if(!gNineAxisSensor.isTurnSide())
	// 	{
	// 		gServo.wrap(0);
	// 		gMotorDrive.drive(0);
	// 		mSubState = Checking;
	// 		mCheckTime = time;
	// 	}
	// 	gServo.wrap(0.0);
	// 	turn_side_state = gNineAxisSensor.getTurnSideDirection();
	// 	if (turn_side_state == Right) {
	// 		gServo.turn(0.9);
	// 	}
	// 	else if (turn_side_state == Left) {
	// 		gServo.turn(-0.9);
	// 	}
	// 	gMotorDrive.drive(100);
	// 	break;

	case Checking:
		if(!mCheckFlag)
		{
			Debug::print(LOG_SUMMARY, "[WakingTurnSide] Checking\r\n");
			mCheckTime = time;
            gMotorDrive.drive(0);
			gServo.turn(0);
			gServo.wrap(1.0);
			mCheckFlag = !mCheckFlag;
		}

		if (Time::dt(time, mCheckTime) > WAKING_TURN_SIDE_CHECK_TIME)
		{
			Debug::print(LOG_SUMMARY, "[WakingTurnSide] End Checking\r\n");
            if(!gNineAxisSensor.isTurnSide())
			{
				Debug::print(LOG_SUMMARY, "[WakingTurnSide] Succeed\r\n");
				setRunMode(false);
				return;
			}

			if (mWakeRetryCount++ >= WAKING_TURN_SIDE_RETRY_COUNT)
			{
				setRunMode(false);
				return;
			}
			Debug::print(LOG_SUMMARY, "[WakingTurnSide] Retry Count %d\r\n", mWakeRetryCount);
			if (mBridging) 
			{
				Debug::print(LOG_SUMMARY, "[WakingTurnSide] Bridging\r\n");
				mSubState = Bridging;
			}
			else mSubState = CheckTurnSide;
			mCheckTime = time;
			mCheckFlag = false;
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
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[WakingTurnSide] Finished\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");

}

WakingFromTurnSide::WakingFromTurnSide()
{
	setName("waking_turnside");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}

WakingFromTurnSide::~WakingFromTurnSide()
{
}
