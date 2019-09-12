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
#include "../constants.h"
#include "../rover_util/logging.h"
#include "separating_sequence.h"
#include "separating_sequence_constant.h"
#include "testing_sequence.h"
#include "navigating_sequence.h"
#include "../sub_sequence/waking_turnback.h"
#include "../sub_sequence/waking_turnside.h"

#include "../sensor/gps.h"
#include "../sensor/light.h"
#include "../sensor/nineaxis.h"
#include "../sensor/pressure.h"
#include "../sensor/distance.h"
#include "../sensor/lora.h"
#include "../actuator/motor.h"
#include "../actuator/servo.h"
#include "../noisy/buzzer.h"
#include "../noisy/led.h"



SeparatingState gSeparatingState;
bool SeparatingState::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Separating State] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);

	//util
	gSerialCommand.setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	//log
    gUnitedLoggingState.setRunMode(true);
	gMovementLoggingState.setRunMode(true);
	//sensor
	gLightSensor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gNineAxisSensor.setRunMode(true);
	gDistanceSensor.setRunMode(true);
	gLora.setRunMode(true);
	//actuator
	gServo.setRunMode(true);
	gMotorDrive.setRunMode(true);
	//noise
	gLED.setRunMode(true);
	gBuzzer.setRunMode(true);

	init(time);

	return true;
}
void SeparatingState::onUpdate(const struct timespec& time)
{
	if (gWakingFromTurnSide.isActive())return;
	gLora.setSeqName(getName());

	switch (mCurStep)
	{
	case STEP_STABI_OPEN:
		gMotorDrive.drive(0);
		gServo.wrap(1.0);
		gServo.turn(0.0); 

		mCurStep = STEP_WAIT_STABI_OPEN;
		mLastUpdateTime = time;
		break;
	case STEP_WAIT_STABI_OPEN:
		if (Time::dt(time, mLastUpdateTime) > 0.5)
		{
			mLastUpdateTime = time;
			gMotorDrive.drive(0);
			mCurStep = STEP_SEPARATE;
		}
		break;
	case STEP_SEPARATE:
		if (Time::dt(time, mLastUpdateTime) < SEPARATING_SERVO_INTERVAL)return;
		mLastUpdateTime = time;
		gMotorDrive.drive(0);

		mCurServoState = !mCurServoState;

		gServo.wrap(1.0);
		if (mCurServoState)
		{
			//gServo.turn(1.0); 
			gServo.move("direct", 4600); 
		}
		else
		{
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
			gServo.wrap(0.0);
			gServo.turn(0);
            mCurStep = STEP_CHECK_IF_INSIDE;
            mCurServoState = true;
            mLastUpdateTime = time;
			mStartStepTime = time;
			Debug::print(LOG_SUMMARY, "[Separating State] Fighting Free Finished\r\n");
        }
        break;
   case STEP_CHECK_IF_INSIDE:
	   if (Time::dt(time, mLastUpdateTime) < SEPARATING_SERVO_INTERVAL) return;
	   mLastUpdateTime = time;

	   Debug::print(LOG_SUMMARY, "[Separating State] CheckIfInside Abort time: %1.1f/%d\r\n",Time::dt(time, mStartStepTime),SEPARATING_CHECK_INSIDE_ABORT_TIME);
	   if(Time::dt(time, mStartStepTime) > SEPARATING_CHECK_INSIDE_ABORT_TIME)
	   {
		   Debug::print(LOG_SUMMARY, "[Separating State] inside of gaikokkaku :(  RETRY!!!! %d/%d\r\n", mRetryCount,SEPARATING_RETRY_MAX_COUNT);
		   if(mRetryCount < SEPARATING_RETRY_MAX_COUNT){
			   init(time);
			   mRetryCount++;
		   }
		   else {
			   mCurStep = STEP_GET_DISTANCE;
			   mCurMotorStep = STEP_MOTOR_MOVE;
		   }
		   mLastUpdateTime = time;
		   break;
	   }
	   switch(mCurMotorStep) {
		case STEP_MOTOR_MOVE:
			gMotorDrive.drive(-10);
			mCurMotorStep = STEP_MOTOR_STOP;
			mLastMotorMoveTime = time;
			break;
		case STEP_MOTOR_STOP:
			gMotorDrive.drive(0);
			Debug::print(LOG_SUMMARY, "[Separating State] Check Count %d : %d\r\n", gDistanceSensor.getDistance(), 100);
			// Check 
			if (gDistanceSensor.getDistance() > 100)
			{
				mCheckCount++;
			}
			else
			{
				mCheckCount = 0;
			}
			if (Time::dt(time, mLastMotorMoveTime) > 5) {
				mCurMotorStep = STEP_MOTOR_MOVE;
			}
			break;
	   }

	   Debug::print(LOG_SUMMARY, "[Separating State] Check Count %d/%d\r\n",mCheckCount,SEPARATING_CHECK_MAX_COUNT);
	   if(mCheckCount > SEPARATING_CHECK_MAX_COUNT){
		   Debug::print(LOG_SUMMARY, "[Separating State] outside of gaikokkaku :)\r\n");
		   mCurStep = STEP_GET_DISTANCE;
		   mLastUpdateTime = time;
		   //TurnBackDirection turn_back_state gNineAxisSensor.getTurnBackDirection();
	   }

	   break;
	case STEP_GET_DISTANCE:
		if (Time::dt(time, mLastUpdateTime) < SEPARATING_MOTOR_INTERVAL)return;
		// if rover is still inside gaikokkaku

        mLastUpdateTime = time;
		
		gServo.turn(0.0);
		turn_side_state = gNineAxisSensor.getTurnSideDirection();
		if(turn_side_state != Right && turn_side_state != Left){
			Debug::print(LOG_SUMMARY, "[Separating State] STEP_GET_DISTANCE rover is standing\r\n");
		 	gMotorDrive.drive(0);
			gServo.turn(0.0);
		 	gServo.wrap(0.0);
			mCurStep = STEP_RUN_WHILE;
			mStartStepTime = time;
			break;
		}

		switch(mCurMotorStep){
		case STEP_MOTOR_MOVE:
			gServo.wrap(1.0);
			Debug::print(LOG_SUMMARY, "[Separating State] Getting Distance Motor Move\r\n");
			gMotorDrive.drive(-100);
			mCurMotorStep = STEP_MOTOR_STOP;
			break;
		case STEP_MOTOR_STOP:
			Debug::print(LOG_SUMMARY, "[Separating State] Getting Distance servo release\r\n");
			gMotorDrive.drive(0);
			gServo.wrap(-1.0);
			mCurMotorStep = STEP_MOTOR_MOVE;
			break;
		}

        ++mServoGetDistanceCount;
        Debug::print(LOG_SUMMARY, "[Separating State] Getting Distance Count (%d/%d)\r\n", mServoGetDistanceCount, SEPARATING_GET_DISTANCE_COUNT);
 
		if(mServoGetDistanceCount >= SEPARATING_GET_DISTANCE_COUNT)
		{
			Debug::print(LOG_SUMMARY, "[Separating State] Getting Distance Finished\r\n");
			gMotorDrive.drive(0);
			if(mChijikiMode) mCurStep = STEP_MOVE_BY_CHIJIKI;
			else mCurStep = STEP_DECIDE_DIRECTION;
			mStartStepTime = time;
		}
		break;
	case STEP_DECIDE_DIRECTION:
	{
		if(Time::dt(time, mLastUpdateTime) < 0.2) return;
		mLastUpdateTime = time;
		if (Time::dt(time, mStartStepTime) > SEPARATING_DECIDE_DIRECTION_ABORT_TIME) {
			mCurStep = STEP_STABLE_AWAKE_FROM_SIDE;
			mStartStepTime = time;
			gServo.wrap(0.0);
		}
		Debug::print(LOG_SUMMARY, "[Separating State] STEP_DECIDE_DIRECTION abort time %lf/%d\r\n",mStartStepTime,SEPARATING_DECIDE_DIRECTION_ABORT_TIME);
		
		if(!gNineAxisSensor.isTurnSide()){
			Debug::print(LOG_SUMMARY, "[Separating State] STEP_DECIDE_DIRECTION Finished\r\n");
			nextState();
		} 
		gServo.wrap(1);
		gServo.turn(0);
		if(move){
			gMotorDrive.drive(-100);
			mLastMotorMoveTime = time;
			move = !move;
		}
		else{
			gMotorDrive.drive(0);
			//Debug::print(LOG_SUMMARY, "[Separating State] Check_Gaikokkaku Distance:%d \r\n",gDistanceSensor.getDistance());
			if(gDistanceSensor.getDistance() > 150){
				//whenever missing gaikokkaku
				if (mReadyFlag) {
					Debug::print(LOG_SUMMARY, "[Separating State] found safe way to run !\r\n");
					mCurStep = STEP_STABLE_AWAKE_FROM_SIDE;
					mStartStepTime = time;
					gServo.wrap(0.0);
				}
				mDetectedArmorCount = 0;
			}
			else {
				//gaikokkaku mituketenakattara
				if (!mReadyFlag)
				{
					Debug::print(LOG_SUMMARY, "[Separating State] gaikokkaku may at infront count %d/%d\r\n",mDetectedArmorCount,3);
					if (mDetectedArmorCount > 3) {
						Debug::print(LOG_SUMMARY, "[Separating State] gaikokkaku detected\r\n");
						mReadyFlag = true;
					}
					else mDetectedArmorCount++;
				}
			}
			if (Time::dt(time, mLastMotorMoveTime) >= 5) {
				move = !move;
			}
		}
		break;
	}
	case STEP_STABLE_AWAKE_FROM_SIDE:
	{
		Debug::print(LOG_SUMMARY, "[Separating State] STEP_STABLE_AWAKE %lf/%d\r\n",Time::dt(time, mStartStepTime),SEPARATING_AWAKE_ABORT_TIME);
		if (Time::dt(time, mStartStepTime) > SEPARATING_AWAKE_ABORT_TIME) nextState();
		if(!gNineAxisSensor.isTurnSide()){
			mLastUpdateTime = time;
			mStartStepTime = time;
			mCurStep = STEP_RUN_WHILE;
		}
		turn_side_state = gNineAxisSensor.getTurnSideDirection();
		if(turn_side_state == Right) {
			gServo.turn(1);
		}
		else if (turn_side_state == Left){
			gServo.turn(-1);
		}
		gServo.wrap(0.0);
		gMotorDrive.drive(-100);
		/*if (gNineAxisSensor.isTurnSide()) {
			gWakingFromTurnSide.setRunMode(true);
		}
		else {
			nextState();
		}*/
		break;
	}
	case STEP_RUN_WHILE:
		if (Time::dt(time, mLastUpdateTime) < 1) return;
		mLastUpdateTime = time;
		gServo.wrap(0.0);
		if (gNineAxisSensor.isTurnBack()) gMotorDrive.drive(-100);
		else gMotorDrive.drive(100);
		Debug::print(LOG_SUMMARY, "[Separating State] STEP_RUN_WHILE %lf/%d\r\n",Time::dt(time, mStartStepTime),SEPARATIMG_RUN_WHILE_DURATION);
		if (Time::dt(time, mStartStepTime) > SEPARATIMG_RUN_WHILE_DURATION) {
			gMotorDrive.drive(0);
			nextState();
		}
		break;

	};
}
void SeparatingState::nextState()
{
	gLED.clearLED();
	setRunMode(false);
    gServo.turn(0.0);
	gMotorDrive.drive(0);

	if (!mMissionFlag)
	{
		gTestingState.setRunMode(true);
	}
	else
	{
		gNavigatingState.setRunMode(true);
		gNavigatingState.SetMissionFlag(true);
		SetMissionFlag(false);
	}


}
void SeparatingState::SetMissionFlag(bool flag)
{
	mMissionFlag = flag;
}

void SeparatingState::init(const struct timespec& time)
{
	//initialize
	gServo.free();
	gMotorDrive.drive(0);
	gServo.wrap(1.0);
	gServo.turn(-1.0);
	gLED.setColor(0, 255, 255);
	gLora.enableGPSsend(true);

	mLastUpdateTime = time;
	mCurServoState = false;
	mServoCount = 0;
    mServoOpenCount = 0;
    mServoFightForFreeCount = 0;
    mServoGetDistanceCount = 0;
	mCheckCount = 0;
	mDetectedArmorCount = 0;
	mCurStep = STEP_STABI_OPEN;
	//mCurStep = STEP_STABLE_AWAKE_FROM_SIDE;
	mCurMotorStep = STEP_MOTOR_MOVE;
	move = false;
	mReadyFlag = false;
	mChijikiMode = false;
}

void SeparatingState::onClean()
{
	gLED.clearLED();
	Debug::print(LOG_SUMMARY, "[Separating State] Finished\r\n");
}

SeparatingState::SeparatingState() : mCurServoState(false), mServoCount(0), mServoOpenCount(0), mServoGetDistanceCount(0), mServoFightForFreeCount(0)
{
	setName("separating");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
	SetMissionFlag(false);
}
SeparatingState::~SeparatingState()
{
}
