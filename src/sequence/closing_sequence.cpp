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
#include "closing_sequence.h"
#include "./navigating_sequence.h"
#include "../sensor/distance.h"

ClosingState gClosingState;


bool ClosingState::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Closing State] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);

	gDistanceSensor.setRunMode(true);

	return true;
}

void ClosingState::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) < CLOSING_STATE_UPDATE_INTERVAL_TIME) return;
	mLastUpdateTime = time;
	mDistToGoal = gDistanceSensor.getDistance();

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
	Debug::print(LOG_SUMMARY, "[Closing State] Finished\r\n");
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