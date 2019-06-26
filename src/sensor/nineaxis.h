#pragma once
#include <pthread.h>
#include <list>
#include <ctime>
#include <iostream>
#include <deque>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"
#include <RTIMULib.h>

class NineAxisSensor : public TaskBase
{
private:
	struct timespec mLastUpdateTime;
	RTIMU_DATA mIMUData;
	RTIMU *imu;
	bool isShowMode;
	void showData(bool enableAccel, bool enableGyro, bool enableCompass, bool enableFusionPoss);
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onUpdate(const struct timespec& time);
public:
	RTIMU_DATA getNewImuData();
	NineAxisSensor();
	~NineAxisSensor();
};

extern NineAxisSensor gNineAxisSensor;



