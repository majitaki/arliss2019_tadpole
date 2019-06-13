#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <functional>
#include <stdarg.h>
#include <wiringPi.h>

#include "testing_sequence.h"
#include "waiting_sequence.h"
#include "falling_sequence.h"
#include "separating_sequence.h"
#include "navigating_sequence.h"
#include "../rover_util/utils.h"

#include "arliss_sequence.h"

ArlissState gArlissState;
bool ArlissState::onInit(const timespec & time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Arliss State] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);

	//‰Šú‰»
	gWaitingState.SetNavigatingFlag(true);
	gFallingState.SetNavigatingFlag(true);
	gSeparatingState.SetNavigatingFlag(true);
	gNavigatingState.SetNavigatingFlag(true);
	nextState();
	return true;
}

bool ArlissState::onCommand(const std::vector<std::string>& args)
{
	return false;
}

void ArlissState::onClean()
{
}

void ArlissState::nextState()
{
	//Ÿ‚Ìó‘Ô‚ğİ’è
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "Arliss Mission Start\r\n");
	Debug::print(LOG_SUMMARY, "Good Luck!!!!!!!!!!!!!\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	gWaitingState.setRunMode(true);
	gWaitingState.SetNavigatingFlag(true);
	setRunMode(false);
}

ArlissState::ArlissState()
{
	setName("arliss");
	setPriority(UINT_MAX, UINT_MAX);
}

ArlissState::~ArlissState()
{
}
