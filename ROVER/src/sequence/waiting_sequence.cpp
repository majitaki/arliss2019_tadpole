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
#include "testing_sequence.h"
#include "waiting_sequence.h"
#include "falling_sequence.h"

#include "../sensor/gps.h"
#include "../sensor/light.h"
#include "../sensor/nineaxis.h"
#include "../sensor/pressure.h"
#include "../sensor/distance.h"
#include "../actuator/motor.h"
#include "../actuator/servo.h"
#include "../noisy/buzzer.h"
#include "../noisy/led.h"

WaitingState gWaitingState;
bool WaitingState::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Waiting State] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	mStartTime = time;

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
	//noise
	gLED.setRunMode(true);
	gBuzzer.setRunMode(true);

	//initialize
	mLastUpdateTime = mWaitingStartTime = time;
	mContinuousLightCount = 0;
	mLightCountSuccessFlag = false;
	mMaxAltitude = 0;
	gLED.setColor(255, 0, 255);

	//gServo.waitingHoldPara();
	gServo.wrap(1.0);
	gServo.turn(-1.0);

	//wifi stop
	// if (mNavigatingFlag)
	// {
	// 	Debug::print(LOG_SUMMARY, "[Waiting State] Wifi Stop\r\n");
	// 	system("sudo ip link set wlan0 down");
	// }

	return true;
}

void WaitingState::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) < WAITING_STATE_UPDATE_INTERVAL_TIME) return;
	mLastUpdateTime = time;

	CheckLightCount(time);

	//update max altitude
	double currentAlt = gPressureSensor.getAltitude();
	if (currentAlt > mMaxAltitude) mMaxAltitude = currentAlt;

	if (mLightCountSuccessFlag)
	{
		nextState();
		return;
	}

	double dt = Time::dt(time, mWaitingStartTime);
	Debug::print(LOG_SUMMARY, "[Waiting State] Abort Check %1.1f / %d(Sub) %d(Last)\r\n", dt, WAITING_ABORT_TIME_FOR_SUB_GOAL, WAITING_ABORT_TIME_FOR_LAST);

	if (dt > WAITING_ABORT_TIME_FOR_SUB_GOAL) {
		Debug::print(LOG_SUMMARY, "[Waiting State] Altitude Check %f / %d\r\n", mMaxAltitude, WAITING_STATE_PRESSURE_THRESHOLD_FOR_SUB_GOAL);
		if (mMaxAltitude > WAITING_STATE_PRESSURE_THRESHOLD_FOR_SUB_GOAL)
		{
			Debug::print(LOG_SUMMARY, "Waiting Second Check Success\r\n");
			nextState();
			return;
		}
	}

	if (dt > WAITING_ABORT_TIME_FOR_LAST)
	{
		Debug::print(LOG_SUMMARY, "Waiting Timeout\r\n");
		nextState();
		return;
	}
}

bool WaitingState::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	switch (args.size())
	{
	case 1:
		Debug::print(LOG_PRINT,
			"\r\n\
waiting wifistop        : wifi stop\r\n\
");
		return true;
	case 2:
		if (args[1].compare("wifistop") == 0)
		{
			Debug::print(LOG_SUMMARY, "[Waiting State] Wifi Stop\r\n");
			//system("sudo ip l set wlan0 down");//������on -> off��
            system("sudo ifconfig wlan0 down");
            system("sudo systemctl stop create_ap");
			return true;
		}
	}

	Debug::print(LOG_PRINT, "Failed Command\r\n");
	return false;
}
void WaitingState::onClean()
{	
    //wifi stop
	if (mNavigatingFlag)
	{
	 	Debug::print(LOG_SUMMARY, "[Waiting State] Wifi Restart\r\n");
		system("sudo ifconfig wlan0 up");
		system("sudo systemctl restart create_ap");
	}

	Debug::print(LOG_SUMMARY, "[Waiting State] Finished\r\n");
}

void WaitingState::CheckLightCount(const timespec & time)
{
	if (mLightCountSuccessFlag)return;

	if (mContinuousLightCount == 0)
	{
		mStartLightCheckTime = time;
		mContinuousLightCount++;
		Debug::print(LOG_SUMMARY, "[Waiting State] Light Check Initialize\r\n");
	}
	else
	{

		bool light_react = gLightSensor.get();

		if (light_react)
		{
			int diff_time = Time::dt(time, mStartLightCheckTime);


			if (diff_time > LIGHT_COUNT_TIME)
			{
				Debug::print(LOG_SUMMARY, "[Waiting State] Light Check Success.\r\n");
				mLightCountSuccessFlag = true;
				mContinuousLightCount = 0;
				gBuzzer.start(70, 1);
				return;
			}

			gLED.brink(0.2);
			gBuzzer.start(25, 2);

			mContinuousLightCount++;
			Debug::print(LOG_SUMMARY, "[Waiting State] Light Check Update %d(s) / %d(s)\r\n", diff_time, LIGHT_COUNT_TIME);
			return;

		}
		else
		{
			Debug::print(LOG_SUMMARY, "[Waiting State] Light Check Failed.\r\n");
			mContinuousLightCount = 0;
			gBuzzer.start(50);
			gLED.stopBrink();
			gLED.setColor(255, 0, 255);
			return;
		}
	}
}

void WaitingState::nextState()
{
	gLED.clearLED();
	setRunMode(false);
	if (!mNavigatingFlag)
	{
		gTestingState.setRunMode(true);
	}
	else
	{
		gServo.wrap(1.0);
		gServo.turn(-1.0);
		
		gFallingState.setRunMode(true);
		gFallingState.SetNavigatingFlag(true);
	}
}
void WaitingState::SetNavigatingFlag(bool flag)
{
	mNavigatingFlag = flag;
}
WaitingState::WaitingState()
{
	setName("waiting");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
	SetNavigatingFlag(false);
}
WaitingState::~WaitingState()
{
}
