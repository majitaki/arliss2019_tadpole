#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <functional>
#include <stdarg.h>
#include <wiringPi.h>

#include "../rover_util/delayed_execution.h"
#include "../rover_util/utils.h"
#include "../rover_util/serial_command.h"
#include "../actuator/motor.h"
#include "../constants.h"
#include "../rover_util/logging.h"
#include "separating_sequence.h"
#include "separating_sequence_constant.h"
//#include "../manager/accel_manager.h"
#include "testing_sequence.h"
#include "navigating_sequence.h"
#include "../actuator/servo.h"
#include "../sensor/gps.h"
#include "../sensor/pressure.h"
#include "../sensor/nineaxis.h"

SeparatingState gSeparatingState;
bool SeparatingState::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Separating State] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	gServo.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gNineAxisSensor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gUnitedLoggingState.setRunMode(true);
	gMovementLoggingState.setRunMode(true);

	//gServo.waitingHoldPara();
	gServo.wrap(1.0);
	gServo.turn(-1.0);
	//gServo.centerDirect();

	mLastUpdateTime = time;
	mCurServoState = false;
	mServoCount = 0;
    mServoOpenCount = 0;
    mServoFightForFreeCount = 0;
    mServoGetDistanceCount = 0;
	mCurStep = STEP_STABI_OPEN;

	return true;
}
void SeparatingState::onUpdate(const struct timespec& time)
{
	switch (mCurStep)
	{
	case STEP_STABI_OPEN:
		//gServo.holdPara();
		gServo.wrap(1.0);
		gServo.turn(0.0); 

		mCurStep = STEP_WAIT_STABI_OPEN;
		mLastUpdateTime = time;
		break;
	case STEP_WAIT_STABI_OPEN:
		if (Time::dt(time, mLastUpdateTime) > 0.5)
		{
			mLastUpdateTime = time;
			mCurStep = STEP_SEPARATE;
		}
		break;
	case STEP_SEPARATE:
		if (Time::dt(time, mLastUpdateTime) < SEPARATING_SERVO_INTERVAL)return;
		mLastUpdateTime = time;

		mCurServoState = !mCurServoState;

		gServo.wrap(1.0);
		if (mCurServoState)
		{
			//gServo.releasePara();
			gServo.turn(1.0); 
		}
		else
		{
			//gServo.holdPara();
			gServo.turn(0.0); 
		}

		++mServoCount;
		Debug::print(LOG_SUMMARY, "[Separating State] Separating Count (%d/%d)\r\n", mServoCount, SEPARATING_SERVO_COUNT);

		if (mServoCount >= SEPARATING_SERVO_COUNT)
		{
			mLastUpdateTime = time;
            mCurServoState = true;
            mCurStep = STEP_SEPARATE_OPEN;
			Debug::print(LOG_SUMMARY, "[Separating State] Separating Finished\r\n");
			//gWakingState.setRunMode(true);
			//nextState();
		}
		break;
   case STEP_SEPARATE_OPEN:
        if (Time::dt(time, mLastUpdateTime) < SEPARATING_SERVO_INTERVAL)return;
        mLastUpdateTime = time;

		gServo.turn(0.0); 
        if(mCurServoState)
        {
			gServo.wrap(1.0);
            //gServo.holdPara();
        }
        else
        {
            gServo.wrap(-1.0);
        }
        mCurServoState = !mCurServoState;
		++mServoOpenCount;

        Debug::print(LOG_SUMMARY, "[Separating State] Opening Count (%d/%d) \r\n", mServoOpenCount, SEPARATING_OPEN_COUNT);
        if (mServoOpenCount >= SEPARATING_OPEN_COUNT)
        {
            mCurStep = STEP_FIGHT_FOR_FREE;
            mCurServoState = true;
            mLastUpdateTime = time;
			Debug::print(LOG_SUMMARY, "[Separating State] Opening Finished\r\n");
        }
        break;
   case STEP_FIGHT_FOR_FREE:
        if (Time::dt(time, mLastUpdateTime) < SEPARATING_SERVO_INTERVAL)return;
        mLastUpdateTime = time;

		gServo.wrap(0.0);
		if(mCurServoState){
			gServo.turn(1.0);
		}else{
			gServo.turn(-1.0);
		}

        ++mServoFightForFreeCount;
        mCurServoState = !mCurServoState;
        Debug::print(LOG_SUMMARY, "[Separating State] Fighting Free Count (%d/%d)\r\n", mServoFightForFreeCount, SEPARATING_FIGHT_FOR_FREE_COUNT);
        if(mServoFightForFreeCount >= SEPARATING_FIGHT_FOR_FREE_COUNT)
        {
            mCurStep = STEP_GET_DISTANCE;
            mCurServoState = true;
            mLastUpdateTime = time;
			Debug::print(LOG_SUMMARY, "[Separating State] Fighting Free Finished\r\n");
        }
        break;
	case STEP_GET_DISTANCE:
		if (Time::dt(time, mLastUpdateTime) < SEPARATING_MOTOR_INTERVAL)return;
        mLastUpdateTime = time;

		TurnSideDirection turn_side_state = gNineAxisSensor.getTurnSideDirection();
		//TurnBackDirection turn_back_state gNineAxisSensor.getTurnBackDirection();

		gServo.wrap(1.0);
		if(turn_side_state == Right){
			gServo.turn(-0.8);
		}else{
			gServo.turn(0.8);
		}

		if(mCurServoState){
			gMotorDrive.drive(100);
		}else{
			gMotorDrive.drive(-100);
		}

        ++mServoGetDistanceCount;
        mCurServoState = !mCurServoState;
        Debug::print(LOG_SUMMARY, "[Separating State] Getting Distance Count (%d/%d)\r\n", mServoGetDistanceCount, SEPARATING_GET_DISTANCE_COUNT);
 
		if(mServoGetDistanceCount >= SEPARATING_GET_DISTANCE_COUNT)
		{
			Debug::print(LOG_SUMMARY, "[Separating State] Getting Distance Finished\r\n");
			nextState();
		}
		break;
	};
}
void SeparatingState::nextState()
{
	setRunMode(false);
    gServo.turn(0.0);

	if (!mNavigatingFlag)
	{
		gTestingState.setRunMode(true);
	}
	else
	{
		gNavigatingState.setRunMode(true);
		gNavigatingState.SetNavigatingFlag(true);
	}


}
void SeparatingState::SetNavigatingFlag(bool flag)
{
	mNavigatingFlag = flag;
}

void SeparatingState::onClean()
{
	Debug::print(LOG_SUMMARY, "[Separating State] Finished\r\n");
}

SeparatingState::SeparatingState() : mCurServoState(false), mServoCount(0), mServoOpenCount(0), mServoGetDistanceCount(0), mServoFightForFreeCount(0)
{
	setName("separating");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
	SetNavigatingFlag(false);
}
SeparatingState::~SeparatingState()
{
}
