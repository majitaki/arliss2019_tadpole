#include <time.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <deque>
#include "./nineaxis.h"
#include "../rover_util/utils.h"
#include  "./nineaxis_constant.h"
#include  "./nineaxis.h"
#include <RTIMULib.h>


using namespace std;

NineAxisSensor gNineAxisSensor;

bool NineAxisSensor::onInit(const struct timespec& time)
{
	RTIMUSettings *settings = new RTIMUSettings(".", "RTIMULib");

    imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() != RTIMU_TYPE_MPU9250)) {
		Debug::print(LOG_SUMMARY, "Failed to Begin NineAxis Sensor\r\n");
		//setRunMode(false);
		return false;
    }

	imu->IMUInit();
	imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);
	Debug::print(LOG_SUMMARY, "NineAxis Sensor is Ready!\r\n");

	mLastUpdateTime = time;
	return true;
}

void NineAxisSensor::onUpdate(const struct timespec& time)
{
	//It is necessary to read nineaxis sensor as much as possible. If not, you will get fifo warning.
	while (imu->IMURead()) {
		mIMUData = imu->getIMUData();
	}

	if (Time::dt(time, mLastUpdateTime) < NINEAXIS_UPDATE_INTERVAL_TIME)
	{
		return;
	}

	if(isShowMode){
		showData(true, true, true, true);
	}
	
	mLastUpdateTime = time;
}

void NineAxisSensor::onClean()
{
	Debug::print(LOG_SUMMARY, "NineAxis Sensor is Finished\r\n");
}

bool NineAxisSensor::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	switch (args.size())
	{
	case 1:
		Debug::print(LOG_PRINT, "nineaxis show :switch show mode\r\n");
		showData(true, true, true, true);
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

void NineAxisSensor::showData(bool enableAccel, bool enableGyro, bool enableCompass, bool enableFusionPoss)
{
	if(mIMUData.accelValid && enableAccel){
		Debug::print(LOG_PRINT, "%s", RTMath::displayRadians("accel", mIMUData.accel));
	}
	if(mIMUData.gyroValid && enableGyro){
		Debug::print(LOG_PRINT, "%s", RTMath::displayRadians("gyro", mIMUData.gyro));
	}
	if(mIMUData.compassValid && enableCompass){
		Debug::print(LOG_PRINT, "%s", RTMath::displayRadians("compass", mIMUData.compass));
	}
	if(mIMUData.fusionPoseValid && enableFusionPoss){
		Debug::print(LOG_PRINT, "%s\r\n", RTMath::displayDegrees("fusion_poss", mIMUData.fusionPose));
	}
	//Debug::print(LOG_PRINT, "\r\n");
}


NineAxisSensor::NineAxisSensor() : mLastUpdateTime(), mIMUData(), imu(), isShowMode(false)
{
	setName("nineaxis");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_SENSOR);
}
NineAxisSensor::~NineAxisSensor()
{
}
