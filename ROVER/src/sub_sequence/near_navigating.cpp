#include <cmath>
#include<stdlib.h>
#include "../rover_util/delayed_execution.h"
#include "../rover_util/utils.h"
#include "../rover_util/serial_command.h"
#include "../constants.h"
#include "near_navigating.h"
#include "near_navigating_constant.h"
#include "../sequence/navigating_sequence_constant.h"
#include "../sequence/navigating_sequence.h"
#include "../sequence/testing_sequence.h"

#include "../sensor/gps.h"
#include "../sensor/nineaxis.h"
#include "../sensor/distance.h"
#include "../actuator/motor.h"
#include "../actuator/servo.h"
#include "../noisy/buzzer.h"
#include "../noisy/led.h"
#include "waking_turnside_constant.h"

NearNavigating gNearNavigating;

bool NearNavigating::onInit(const timespec & time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Near Navigating] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	//if(!gNavigatingState.isActive()) {
	//	Debug::print(LOG_SUMMARY, "[Near] Failed to Begin Near Navigating\r\n");
	//	return false;
	//}

	//initialize
	mLastUpdateTime = time;
	mLastNearNaviTime = time;
	mCheckTime = time;
	mSubState = Initial;
	turn_value = NEAR_NAVIGATING_SERVO_TURN;
	isGoalLeft = (gNavigatingState.getDeltaAngle() > 0) ? true : false;
	mCheckCount = 0;
	mTurnValueChangeFlag = true;
	isInfinityOperation = false;
	isGyroOperation = false;
	updateFlag = true;
	turn_value2 = 0.9;
	turn_value3 = 0.2;
	mInitialYaw = 0;
	mInfinityCount = 0;
	mTurnSideBackCount = 0;
	
	gMotorDrive.drive(100);
	gServo.wrap(-1);
	return true;
}

void NearNavigating::onUpdate(const timespec & time)
{
	double dt = Time::dt(time, mLastUpdateTime);
	if (dt < NEAR_NAVIGATING_UPDATE_INTERVAL_TIME) return;
	mLastUpdateTime = time;

	double dt_near_navi = Time::dt(time, mLastNearNaviTime);
	if (dt_near_navi > NEAR_NAVIGATING_TIMEOUT){
		mSubState = Fail;
	}

	//if (gNineAxisSensor.isTurnSide()) { mSubState = Fail;}
	//if (gNineAxisSensor.isTurnBack()) { mSubState = Fail;}

	if (gNineAxisSensor.isTurnBack() || gNineAxisSensor.isTurnSide()) 
	{ 
		mTurnSideBackCount++;
		gServo.turn(0);
		gServo.wrap(0);
		gMotorDrive.drive(100);
		Debug::print(LOG_SUMMARY, "[Near] Turn Side Back Count %d / %d\r\n", mTurnSideBackCount, NEAR_NAVIGATING_TURN_SIDEBACK_CHECK_COUNT);
		if(mTurnSideBackCount > NEAR_NAVIGATING_TURN_SIDEBACK_CHECK_COUNT)
		{
			mSubState = NearGoal;
		}else{
			return;
		}
	}
	else{
		mTurnSideBackCount = 0;
	}


	switch (mSubState)
	{
		case Initial:
		{
			mCheckTime = time;
			if (isGyroOperation) mSubState = Roll;
			else if (isInfinityOperation){
				mSubState = Infinity;
				mInitialYaw = gNineAxisSensor.getYaw();
			}
			else mSubState = NearGoalNavi;
			break;
		}
		case Infinity:
		{
			double dt = Time::dt(time, mCheckTime);
			if( dt > 1){
				mSubState = CheckGoal;
				mCheckTime = time;
			}
			gMotorDrive.drive(100);
			if(mInitialYaw >= 0){
				if(gNineAxisSensor.getYaw() < (mInitialYaw-180)+2 && gNineAxisSensor.getYaw() > (mInitialYaw-180)-2 && updateFlag)
				{
					updateFlag = false;
					turn_value3 *= -1;
				}
					
				else if(gNineAxisSensor.getYaw() < mInitialYaw+2 && gNineAxisSensor.getYaw() > mInitialYaw -2 && !updateFlag)
				{
					updateFlag = true;
					mInfinityCount++;
					if(mInfinityCount > 3) nextState();
				}
			}
			else {
				if(gNineAxisSensor.getYaw() < (mInitialYaw+180)+2 && gNineAxisSensor.getYaw() > (mInitialYaw+180)-2 && updateFlag)
				{
					updateFlag = false;
					turn_value3 *= -1;
				}
				else if(gNineAxisSensor.getYaw() < mInitialYaw+2 && gNineAxisSensor.getYaw() > mInitialYaw-2 && !updateFlag){
					updateFlag = true;
					mInfinityCount++;
					if(mInfinityCount > 3) nextState();
				}
			}
			//if(abs(gNineAxisSensor.getYaw()) > mInitialYaw && updateFlag)
			//{
			//	updateFlag = false;
			//	turn_value3 *= -1;
			//}
			//else if(abs(gNineAxisSensor.getYaw()) < mInitialYaw+2 && abs(gNineAxisSensor.getYaw()) > mInitial -2 && !updateFlag)
			//{
			//	updateFlag = true;
			//
			//}
			gServo.turn(turn_value3);
			break;
		}
		case Roll:
		{
			double dt = Time::dt(time, mCheckTime);
			if (abs(gNineAxisSensor.getYaw()) < 5) {
				if(updateFlag){
					updateFlag = false;
					gServo.turn(turn_value2);
					if(turn_value  > 0) turn_value2 -= 0.1;
					else nextState();
				}
			}

			else if(abs(gNineAxisSensor.getYaw()) > 170){
				updateFlag = true;	
			}

			if (dt > NEAR_NAVIGATING_ROLL_DURATION) {
				gMotorDrive.drive(0);
				mCheckTime = time;
				mSubState = CheckGoal;
			}

			break;
		}
		case NearGoalNavi:
		{
			double dt = Time::dt(time, mCheckTime);
			if (dt < NEAR_NAVIGATING_RUNNING_INTERVAL_TIME){
				double dt = Time::dt(time, mLastNearNaviTime);
				Debug::print(LOG_SUMMARY, "[Near] Abort Time: %f / %d \r\n",dt , NEAR_NAVIGATING_TIMEOUT);
				navigationNearMode();
				/*if(turn_value < 0.3){
					mSubState = Fail;
				}*/
				break;
			} 
			//mTurnValueChangeFlag = true;
			mCheckTime = time;
			mSubState = CheckGoal;
			break;
		}
		case CheckGoal:
		{
			double dt = Time::dt(time, mCheckTime);
			if (mCheckCount < NEAR_NAVIGATING_CHECK_COUNT){
				gMotorDrive.drive(0);
				int mLaserDistance = gDistanceSensor.getDistance();
				Debug::print(LOG_SUMMARY, "[Near] Laser Distance: %d : %d\r\n", mLaserDistance, NEAR_NAVIGATING_GOAL_DISTANCE_THRESHOLD);

				if( mLaserDistance < NEAR_NAVIGATING_GOAL_DISTANCE_THRESHOLD){
					mSuccessCount++;
				}else{
					mSuccessCount = 0;
				}
				mCheckCount++;
				break;
			} 
			mCheckCount = 0;

			if(mSuccessCount >= NEAR_NAVIGATING_GOAL_CHECK_COUNT){
				Debug::print(LOG_SUMMARY, "[Near] Found Goal !\r\n");
				mSubState = RunWhile;
				mCheckTime = time;
				break;
			}else{
				if(isGyroOperation) mSubState = Roll;
				else if(isInfinityOperation) mSubState = Infinity;
				else mSubState = NearGoalNavi;
				mCheckTime = time;
				break;
			}
		}
		case RunWhile:
		{
			gServo.turn(0);
			gMotorDrive.drive(100);
			double dt = Time::dt(time, mCheckTime);
			if (dt > NEAR_NAVIGATING_RUNWHILE_DURATION)  mSubState = NearGoal;
			break;
		}
		case NearGoal:
		{
			Debug::print(LOG_SUMMARY, "[Near] Near Mode Goal\r\n");
			Debug::print(LOG_SUMMARY, "[Near] Navigating Finish Point:(%f %f)\r\n", gGPSSensor.getPosx(), gGPSSensor.getPosy());
			Time::showNowTime();
			nextState();
			break;
		}
		case Fail:
		{
			Debug::print(LOG_SUMMARY, "[Near] Near Mode Failed\r\n");
			setRunMode(false);
			break;
		}
	}
}

bool NearNavigating::onCommand(const std::vector<std::string>& args)
{
	return false;
}

void NearNavigating::onClean()
{
	Debug::print(LOG_SUMMARY, "[Near Navi] Finished\r\n");
}

void NearNavigating::nextState()
{
	gLED.clearLED();
	gMotorDrive.drive(0);
	gServo.wrap(0.0);
	gServo.turn(0.0);
	gServo.free();

	setRunMode(false);
	gTestingState.setRunMode(true);
}

void NearNavigating::navigationNearMode()
{
	// Debug::print(LOG_SUMMARY, "[Near] Servo Turn: %f ;  -%f \r\n", turn_value, std::log(turn_value/1000+1));
	gMotorDrive.drive(100);
	gServo.wrap(-1);
	double turn_value = gNavigatingState.getTurnValue();
	//if(mTurnValueChangeFlag){
		// turn_value -= NEAR_NAVIGATING_SERVO_EACH_TURN;
		//turn_value -= std::log(turn_value/1000+1);
		// mTurnValueChangeFlag = false;
	//}
	if(turn_value < 0.1) turn_value = 0.2;
	if(turn_value > 0.3) turn_value = 0.3;

	gServo.turn(turn_value);
}

NearNavigating::NearNavigating(): isGoalLeft(false)
{
	setName("near_navigating");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}

NearNavigating::~NearNavigating()
{
}
