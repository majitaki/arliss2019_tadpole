﻿#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"


const static double SENSORLOGGING_UPDATE_INTERVAL_TIME = 0.1;
const static double MOVEMENTLOGGING_UPDATE_INTERVAL_TIME = 1;

class SensorLogging : public TaskBase
{
	struct timespec mLastUpdateTime;
	std::string mFilenameGPS, mFilenameGyro, mFilenamePressure, mFilenameEncoder, mFilenameAccel, mFilenameMagnet;
	unsigned long long mLastEncL, mLastEncR;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);

	void write(const std::string& filename, const char* fmt, ...);
public:
	SensorLogging();
	~SensorLogging();
};


class MovementLogging : public TaskBase
{
	struct timespec mLastUpdateTime;
	std::string  mFilenameActuator;

	double mPrevPowerL, mPrevPowerR, mPrevPowerB;
	int mPrevParaServo, mPrevDirectServo;

	unsigned long long mPrevDeltaPulseL, mPrevDeltaPulseR;

	bool mPrintFlag;	
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);

	void write(const std::string& filename, const char* fmt, ...);
public:
	MovementLogging();
	~MovementLogging();
};

extern SensorLogging gSensorLoggingState;
extern MovementLogging gMovementLoggingState;