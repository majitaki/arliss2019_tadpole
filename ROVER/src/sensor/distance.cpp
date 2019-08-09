#include "./distance.h"
#include "./distance_constant.h"
#include "../rover_util/utils.h"
// #include "razor_constant.h"
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <tof.h> 

DistanceSensor gDistanceSensor;
bool DistanceSensor::onInit(const struct timespec& time)
{
	mInit = tofInit(1, 0x29, 1);
	if(mInit != 1){
		Debug::print(LOG_SUMMARY, "Failed to Begin Distance Sensor\r\n");
		return false;
	}
	Debug::print(LOG_SUMMARY, "Distance Sensor is Ready!\r\n");

	mLastUpdateTime = time;
	return true;
}

void DistanceSensor::onClean()
{
	Debug::print(LOG_SUMMARY, "Distance Sensor is Finished\r\n");
}

bool DistanceSensor::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	switch (args.size())
	{
	case 1:
		Debug::print(LOG_PRINT,
			"distance show  switch show mode\r\n");
		showData();
		return true;
		break;
	case 2:
		if (args[1].compare("show") == 0){
			isShowMode = !isShowMode;
			return true;
		}
		break;
	}
	Debug::print(LOG_PRINT, "Failed Command\r\n");

	return false;
}

void DistanceSensor::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) < DISTANCE_UPDATE_INTERVAL_TIME)
	{
		return;
	}

	if(isShowMode){
		showData();
	}

	mDistance = tofReadDistance();
	if(mDistance == 34815 || mDistance == 65535){
		Debug::print(LOG_SUMMARY, "Distance Sensor is not working!\r\n");
		this->onInit(time);
	}
}

int DistanceSensor::getDistance()
{
	return mDistance;
}


void DistanceSensor::showData()
{
	if(isParalysised){
		Debug::print(LOG_PRINT, "Distance Sensor is not working!\r\n");
		return;
	}

	Debug::print(LOG_PRINT, "Distance = %dmm\r\n", mDistance);
}

DistanceSensor::DistanceSensor(): mInit(-1), mDistance(-1), mLastUpdateTime(), isShowMode(false), isParalysised(false)
{
	setName("distance");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_SENSOR);
}

DistanceSensor::~DistanceSensor()
{
}
