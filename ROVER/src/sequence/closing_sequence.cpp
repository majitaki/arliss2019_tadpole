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
#include "navigating_sequence.h"
#include "testing_sequence.h"
#include "../sub_sequence/waking_turnside.h"
#include "../sub_sequence/waking_turnback.h"

#include "../sensor/gps.h"
#include "../sensor/light.h"
#include "../sensor/nineaxis.h"
#include "../sensor/pressure.h"
#include "../sensor/distance.h"
#include "../actuator/motor.h"
#include "../actuator/servo.h"
#include "../noisy/buzzer.h"
#include "../noisy/led.h"



ClosingState gClosingState;


bool ClosingState::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Closing State] Start\r\n");
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
	//actuator
	gServo.setRunMode(true);
	gMotorDrive.setRunMode(true);
	//noise
	gLED.setRunMode(true);
	gBuzzer.setRunMode(true);

	//initialize
	mStartTime = time;
	mLastUpdateTime = mClosingStartTime = time;
	mStateUpdateTime = time;
	mDistToGoal = 9999;
	mDirection = 1;
	mState = Rotate;
	return true;
}

void ClosingState::onUpdate(const struct timespec& time)
{
	double dt = Time::dt(time, mLastUpdateTime);
	if (dt < CLOSING_STATE_UPDATE_INTERVAL_TIME) return;
	mLastUpdateTime = time;

	mDistToGoal = gDistanceSensor.getDistance();
	Debug::print(LOG_SUMMARY, "Goal Distance :%d\r\n",mDistToGoal);
    
    if (gWakingFromTurnSide.isActive())return;
	if (gWakingFromTurnBack.isActive())return;

 	//finish
	if (mDistToGoal < 50)
	{
		gMotorDrive.drive(0);
		gServo.free();
		nextState();
		return;
	}

    	//turn side
    	if (gNineAxisSensor.isTurnSide())
    	{
        	gWakingFromTurnSide.setRunMode(true);
    	}
    	// turn back
    	if (gNineAxisSensor.isTurnBack())
    	{
        	gWakingFromTurnBack.setRunMode(true);
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
		case Stop:
			gMotorDrive.drive(0);
			if (Time::dt(time, mStateUpdateTime) < STOP_LAST_TIME)
			{
				break;
			}
			if (mDistToGoal < 8000)
			{
				Debug::print(LOG_SUMMARY, "[Closing] Stop: Goal Detected\r\n");
				mStateUpdateTime = time;
				mState = Approach;
				Debug::print(LOG_SUMMARY, "[Closing] Approach State\r\n");
			}
			else {
				Debug::print(LOG_SUMMARY, "[Closing] Stop: Goal Missing\r\n");
				mStateUpdateTime = time;
				mState = Rotate;
				Debug::print(LOG_SUMMARY, "[Closing] Rotate State\r\n");
			}

			break;	
		case Rotate:
			//Debug::print(LOG_SUMMARY, "[Closing] Rotate State\r\n");
			//if goal position detected
			if (Time::dt(time, mStateUpdateTime) > ROTATE_LAST_TIME) {
				mStateUpdateTime = time;
				mState = Stop;
				Debug::print(LOG_SUMMARY, "Stop State\r\n");
			}
			gMotorDrive.drive(100);
			gServo.wrap(0);
			gServo.turn(0.6);
			break;
		case Snaky:
			Debug::print(LOG_SUMMARY, "Snaky State\r\n");
			//time out
			if(Time::dt(time, mSnakyStartedTime) > SNAKY_ABORT_TIME_FOR_LAST)
			{
				mState = Rotate;
			}
			//if goal position detected
			else if(mDistToGoal < 8000)
			{
				mState = Approach;
			}
            		if(Time::dt(time, mSnakyLastUpdateTime) > SNAKY_UPDATE_INTERVAL_TIME)
            		{
                		mSnakyLastUpdateTime = time ;
                		mDirection *= -1;
                		gServo.turn(mDirection);
            		}
			gMotorDrive.drive(80);	
			break;
		case Approach:
			//if goal position is unclear
			if (Time::dt(time, mLastUpdateTime) > APPROACH_LAST_TIME) {
				mStateUpdateTime = time;
				mState = Stop;
				Debug::print(LOG_SUMMARY, "Stop State\r\n");
			}
			
			if(mDistToGoal < 8000)
			{
				gServo.turn(0);
				gMotorDrive.drive(60);

			}
			else
			{
				Debug::print(LOG_SUMMARY,"[Closing] Goal Missing\r\n");
				mState = Rotate;
				Debug::print(LOG_SUMMARY,"[Closing] Snaky State\r\n");
				mSnakyStartedTime = time;
				mSnakyLastUpdateTime = time;
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
