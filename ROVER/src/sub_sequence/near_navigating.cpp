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

	switch (mSubState)
	{
		case Initial:
		{
			mCheckTime = time;
			mSubState = NearGoalNavi;
			break;
		}
		case NearGoalNavi:
		{
			double dt = Time::dt(time, mCheckTime);
			if (dt < NEAR_NAVIGATING_RUNNING_INTERVAL_TIME){
				navigationNearMode();
				if(turn_value < 0){
					mSubState = Fail;
				}
				break;
			} 

			mCheckTime = time;
			mSubState = CheckGoal;
			break;
		}
		case CheckGoal:
		{
			double dt = Time::dt(time, mCheckTime);
			if (dt < NEAR_NAVIGATING_STOP_INTERVAL_TIME){
				gMotorDrive.drive(0);
				int mLaserDistance = gDistanceSensor.getDistance();
				Debug::print(LOG_SUMMARY, "[Near] Laser Distance: %d : %d\r\n", mLaserDistance, NEAR_NAVIGATING_GOAL_DISTANCE_THRESHOLD);

				if( mLaserDistance < NEAR_NAVIGATING_GOAL_DISTANCE_THRESHOLD){
					mCheckCount++;
				}else{
					mCheckCount = 0;
				}
				break;
			} 

			if(mCheckCount >= NEAR_NAVIGATING_GOAL_CHECK_COUNT){
				mSubState = NearGoal;
				mCheckTime = time;
				break;
			}else{
				mSubState = NearGoalNavi;
				mCheckTime = time;
				break;
			}
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
	gMotorDrive.drive(100);
	turn_value -= NEAR_NAVIGATING_SERVO_EACH_TURN;

	gServo.wrap(0.0);
	if(isGoalLeft) {
		gServo.turn(turn_value * -1);
	}else{
		gServo.turn(turn_value);
	}
	Debug::print(LOG_SUMMARY, "[Near] Servo Turn: %f / %f (%s)\r\n", turn_value, NEAR_NAVIGATING_SERVO_TURN, isGoalLeft ? "Left" : "Right");
}

NearNavigating::NearNavigating(): isGoalLeft(false)
{
	setName("near_navigating");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}

NearNavigating::~NearNavigating()
{
}
