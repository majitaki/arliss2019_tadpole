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

	//initialize
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
	mCurStep = STEP_STABI_OPEN;
	//mCurStep = STEP_DECIDE_DIRECTION;
	mCurMotorStep = STEP_MOTOR_RIGHT;
	move = false;
	return true;
}
void SeparatingState::onUpdate(const struct timespec& time)
{
	TurnSideDirection turn_side_state;
	switch (mCurStep)
	{
	case STEP_STABI_OPEN:
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
			gServo.turn(1.0); 
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
            mCurStep = STEP_GET_DISTANCE;
            mCurServoState = true;
            mLastUpdateTime = time;
			Debug::print(LOG_SUMMARY, "[Separating State] Fighting Free Finished\r\n");
        }
        break;
	case STEP_GET_DISTANCE:
		if (Time::dt(time, mLastUpdateTime) < SEPARATING_MOTOR_INTERVAL)return;
        mLastUpdateTime = time;

		turn_side_state = gNineAxisSensor.getTurnSideDirection();
		//TurnBackDirection turn_back_state gNineAxisSensor.getTurnBackDirection();

		gServo.wrap(1.0);
		if(turn_side_state == Right){
			gServo.turn(-0.8);
		}else if(turn_side_state == Left){
		 	gServo.turn(0.8);
		}else{
			Debug::print(LOG_SUMMARY, "[Separating State] Getting Distance Finished\r\n");
		 	gServo.turn(0.0);
		 	gServo.wrap(0.0);
			gMotorDrive.drive(0);
			//nextState();
			mCurStep = STEP_DECIDE_DIRECTION;
			break;
		}

		switch(mCurMotorStep){
		case STEP_MOTOR_RIGHT:
			Debug::print(LOG_SUMMARY, "[Separating State] Getting Distance Motor Right\r\n");
			gMotorDrive.drive(80);
			mCurMotorStep = STEP_MOTOR_STOP;
			break;
		case STEP_MOTOR_LEFT:
			gMotorDrive.drive(-80);
			Debug::print(LOG_SUMMARY, "[Separating State] Getting Distance Motor Left\r\n");
			mCurMotorStep = STEP_MOTOR_STOP;
			break;
		case STEP_MOTOR_STOP:
			Debug::print(LOG_SUMMARY, "[Separating State] Getting Distance Motor Stop\r\n");
			gMotorDrive.drive(0);
			if(mCurServoState){
				mCurMotorStep = STEP_MOTOR_LEFT;
			}else{
				mCurMotorStep = STEP_MOTOR_RIGHT;
			}
			mCurServoState = !mCurServoState;
			break;
		}

        ++mServoGetDistanceCount;
        Debug::print(LOG_SUMMARY, "[Separating State] Getting Distance Count (%d/%d)\r\n", mServoGetDistanceCount, SEPARATING_GET_DISTANCE_COUNT);
 
		if(mServoGetDistanceCount >= SEPARATING_GET_DISTANCE_COUNT)
		{
			Debug::print(LOG_SUMMARY, "[Separating State] Getting Distance Finished\r\n");
			gMotorDrive.drive(0);
			mCurStep = STEP_DECIDE_DIRECTION;
		}
		break;
	case STEP_DECIDE_DIRECTION:
		if (Time::dt(time, mLastUpdateTime) < 0.5) return;
        mLastUpdateTime = time;
		
		if(!gNineAxisSensor.isTurnSide()){
			Debug::print(LOG_SUMMARY, "[Separating State] STEP_DECIDE_DIRECTION Finished\r\n");
			nextState();
		} 
		gServo.wrap(1);
		gServo.turn(0);
		if(move){
			turn_side_state = gNineAxisSensor.getTurnSideDirection();
			gMotorDrive.drive(-30);
		}
		else{
			gMotorDrive.drive(0);
			if(gDistanceSensor.getDistance()>8000){
				turn_side_state = gNineAxisSensor.getTurnSideDirection();
				//if(turn_side_state == Right) gMotorDrive.drive(-60);
				//else if (turn_side_state == Left) gMotorDrive.drive(60);
				//Debug::print(LOG_SUMMARY, "[Separating State] safe way to run !\r\n");
				mCurStep = STEP_STABLE_AWAKE_FROM_SIDE;
				gServo.wrap(0.0);
			}
		}
		move = !move;
		break;
	case STEP_STABLE_AWAKE_FROM_SIDE:
		if(!gNineAxisSensor.isTurnSide()){
			mLastUpdateTime = time;
			mCurStep = STEP_RUN_WHILE;
		}
		turn_side_state = gNineAxisSensor.getTurnSideDirection();
		if(turn_side_state == Right) {
			gServo.turn(1);
		}
		else if (turn_side_state == Left){
			gServo.turn(-1);
		}
		gMotorDrive.drive(-100);
		break;
	case STEP_RUN_WHILE:
		if (Time::dt(time, mLastUpdateTime) < SEPARATIMG_RUN_WHILE_DURATION || gWakingFromTurnBack.isActive()) return;
		gMotorDrive.drive(0);
		if (gNineAxisSensor.isTurnBack()) {
			gWakingFromTurnBack.setRunMode(true);
		}
		else {
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
