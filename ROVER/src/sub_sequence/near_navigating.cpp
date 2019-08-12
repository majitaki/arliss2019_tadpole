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

	if(!gNavigatingState.isActive()) {
		Debug::print(LOG_SUMMARY, "[Near] Failed to Begin Near Navigating\r\n");
		return false;
	}

	//initialize
	mLastUpdateTime = time;
	mLastNearNaviTime = time;
	mSubState = NearGoalNavi;
	isGoalLeft = (gNavigatingState.getDeltaAngle() > 0) ? true : false;
	return true;
}

void NearNavigating::onUpdate(const timespec & time)
{
	double dt = Time::dt(time, mLastUpdateTime);
	if (dt < NEAR_NAVIGATING_UPDATE_INTERVAL_TIME)return;
	mLastUpdateTime = time;

	switch (mSubState)
	{
		case NearGoalNavi:
		{
			double dt_near_navi = Time::dt(time, mLastNearNaviTime);
			if (dt_near_navi > NEAR_NAVIGATING_MAX_NAVI_TIME){
				setRunMode(false);
				return;
			}

			Debug::print(LOG_SUMMARY, "[Near] NearGoalNavi\r\n");
			navigationNearMode();
			mSubState = CheckGoal;
			break;
		}
		case CheckGoal:
		{
			int mLaserDistance = gDistanceSensor.getDistance();
			if(mLaserDistance <= NEAR_NAVIGATING_GOAL_DISTANCE_THRESHOLD){
				mSubState = NearGoal;
				break;
			}
			mSubState = NearGoalNavi;
			break;
		}
		case NearGoal:
		{
			gMotorDrive.drive(0);
			gServo.wrap(0.0);
			Debug::print(LOG_SUMMARY, "[Near] Near Mode Goal\r\n");
			Debug::print(LOG_SUMMARY, "[Near] Navigating Finish Point:(%f %f)\r\n", gGPSSensor.getPosx(), gGPSSensor.getPosy());
			Time::showNowTime();
			nextState();
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
	double turn_value = NEAR_NAVIGATING_SERVO_TURN;
	if(isGoalLeft) turn_value *= -1;
	gServo.turnp(turn_value);
}

NearNavigating::NearNavigating(): isGoalLeft(false)
{
	setName("near_navigating");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}

NearNavigating::~NearNavigating()
{
}
