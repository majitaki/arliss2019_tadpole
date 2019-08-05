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
	RTIMUSettings *settings = new RTIMUSettings(SETTING_FOLDER.c_str(), "RTIMULib");
    imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL) || !imu->IMUInit()) {
		Debug::print(LOG_SUMMARY, "Failed to Begin NineAxis Sensor\r\n");
		//setRunMode(false);
		return false;
    }

	//imu->IMUInit();
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
		else if(args[1].compare("side") == 0){
			Debug::print(LOG_PRINT, "nineaxis back :checking isTurnSide\r\n");
			if(isTurnSide()) Debug::print(LOG_PRINT, "true\r\n");
			else Debug::print(LOG_PRINT, "false\r\n");
		}
		else if(args[1].compare("back") == 0){
			Debug::print(LOG_PRINT, "nineaxis back :checking isTurnBack\r\n");
			if(isTurnBack()) Debug::print(LOG_PRINT, "true\r\n");
			else Debug::print(LOG_PRINT, "false\r\n");
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
        double ax = gNineAxisSensor.getAccel().x();
        double ay = gNineAxisSensor.getAccel().y();
        double az = gNineAxisSensor.getAccel().z();
        double l2_accel = std::sqrt(pow(ax, 2) + std::pow(ay, 2) + std::pow(az, 2));
		Debug::print(LOG_PRINT, "l2_accel: %f\r\n", l2_accel);
	}
	if(mIMUData.fusionPoseValid && enableFusionPoss){
		Debug::print(LOG_PRINT, "%s\r\n", RTMath::displayDegrees("fusion_poss", mIMUData.fusionPose));
	}
	//Debug::print(LOG_PRINT, "\r\n");
}

RTVector3 NineAxisSensor::getAccel() {
	return mIMUData.accel;
}
RTVector3 NineAxisSensor::getGyro() {
	return mIMUData.gyro;
}
RTVector3 NineAxisSensor::getMagnet() {
	return mIMUData.compass;
}
RTVector3 NineAxisSensor::getFusionPose() {
	return mIMUData.fusionPose;
}
double NineAxisSensor::getYaw(){
	return mIMUData.fusionPose.z() * RTMATH_RAD_TO_DEGREE;
}
bool NineAxisSensor::isTurnSide() const{
	if(abs(mIMUData.fusionPose.y()*RTMATH_RAD_TO_DEGREE) > TURNSIDE_DEGREE_THRESHOLD)
		return true;
	return false;
}

//int NineAxisSensor::whichSide() const{
TurnSideDirection NineAxisSensor::getTurnSideDirection() const{
	if(abs(mIMUData.fusionPose.y()*RTMATH_RAD_TO_DEGREE) > TURNSIDE_DEGREE_THRESHOLD){
		if(mIMUData.fusionPose.y()*RTMATH_RAD_TO_DEGREE < 0)
			// left
			//return -1;
			return Left;
		else
			//right
			return Right;
			//return 1;
	}else{
		return Center;
	}

    // if(mIMUData.fusionPose.y()*RTMATH_RAD_TO_DEGREE < 0)
    //     // left
    //     //return -1;
    //     return TurnSideDirection.Left;
    // else
    //     //right
    //     return TurnSideDirection.Right;
    //     //return 1;
}

bool NineAxisSensor::isTurnBack() const{
	if(abs(mIMUData.fusionPose.x()*RTMATH_RAD_TO_DEGREE) > TURNBACK_DEGREE_THRESHOLD)
		return true;
	return false;
}
double NineAxisSensor::normalizeAngle(double pos)
{
	while (pos >= 180 || pos < -180)pos += (pos > 0) ? -360 : 360;
	return pos;
}

NineAxisSensor::NineAxisSensor() : mLastUpdateTime(), mIMUData(), imu(), isShowMode(false)
{
	setName("nineaxis");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_SENSOR);
}
NineAxisSensor::~NineAxisSensor()
{
}
