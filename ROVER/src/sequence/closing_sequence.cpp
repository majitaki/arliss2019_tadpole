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
#include "../rover_util/logging.h"
#include "closing_sequence.h"
#include "./navigating_sequence.h"
#include "../sensor/distance.h"
#include "../actuator/servo.h"
#include "../actuator/motor.h"
#include "testing_sequence.h"
ClosingState gClosingState;


bool ClosingState::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Closing State] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	mStartTime = time;
	mLastUpdateTime = mClosingStartTime = time;
	mDistToGoal = 9999;
	mDirection = 1;
	mState = Rotate;

	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);

	gDistanceSensor.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gServo.setRunMode(true);

	return true;
}

void ClosingState::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) < CLOSING_STATE_UPDATE_INTERVAL_TIME) return;
	mLastUpdateTime = time;
	double dt = Time::dt(time, mClosingStartTime);
	mDistToGoal = gDistanceSensor.getDistance();
	Debug::print(LOG_SUMMARY, "Goal Distance :%d\r\n",mDistToGoal);

 	 //finish
	if (mDistToGoal < 50)
	{
		nextState();
		return;
	}

  	//timeout
	if (dt > CLOSING_ABORT_TIME_FOR_LAST)
	{
		Debug::print(LOG_SUMMARY, "Closing Timeout\r\n");
		nextState();
		return;
	}
	
	//
	switch(mState)
	{
		case Initial:
			//no usage
			gMotorDrive.drive(100);
			break;	
		case Rotate:
			Debug::print(LOG_SUMMARY, "Rotate State\r\n");
			//if goal position detected
			if(mDistToGoal!=8191)
			{
				//gMotorDrive.drive(0);
				//mState = Approach;
			}
			//gMotorDrive.drive(80);
			//gServo.turn(1);
			break;
		case Snaky:
			Debug::print(LOG_SUMMARY, "Snaky State\r\n");
			//time out
			if(Time::dt(time, mSnakyStartedTime) > SNAKY_ABORT_TIME_FOR_LAST)
			{
				mState = Rotate;
			}
			//if goal position detected
			else if(mDistToGoal != 8191)
			{
				mState = Approach;
			}
			mDirection *= -1;
			//gServo.turn(mDirection);
			//gMotorDrive.drive(80);	
			break;
		case Approach:
			Debug::print(LOG_SUMMARY, "Approach State\r\n");
			//if goal position is unclear
			if(mDistToGoal != 8191)
			{
				//gServo.turn(0);
				//gMotorDrive.drive(100);

			}
			else
			{
				mState = Snaky;
				mSnakyStartedTime = time;
			}
			break;	
	}
}

bool ClosingState::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	switch (args.size())
	{

	}

	Debug::print(LOG_PRINT, "Failed Command\r\n");
	return false;
}
void ClosingState::onClean()
{
	Debug::print(LOG_SUMMARY, "Goal! \r\n");
	Debug::print(LOG_SUMMARY, "[Closing State] Finished\r\n");

}

void ClosingState::nextState()
{
	//Debug::print(LOG_SUMMARY, "[Closing State] Finished\r\n");
	setRunMode(false);
	gTestingState.setRunMode(true);

}
void ClosingState::SetNavigatingFlag(bool flag)
{
	mNavigatingFlag = flag;
}
ClosingState::ClosingState()
{
	setName("closing");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
	SetNavigatingFlag(false);
}
ClosingState::~ClosingState()
{
}
