#include "./lora.h"
#include "./lora_constant.h"
#include "../rover_util/utils.h"
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

Lora gLora;
bool Lora::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	return true;
}

void Lora::onClean()
{
	Debug::print(LOG_SUMMARY, "Lora is Finished\r\n");
}

bool Lora::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	switch (args.size())
	{
	case 1:
		return true;
	}
	Debug::print(LOG_PRINT, "Failed Command\r\n");
	return false;
}

void Lora::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) < LORA_UPDATE_INTERVAL_TIME)
	{
		return;
	}
}

Lora::Lora(): mLastUpdateTime()
{
	setName("lora");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_SENSOR);
}

Lora::~Lora()
{
}
