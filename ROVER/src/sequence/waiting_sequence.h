#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"
#include "../sensor/pressure.h"


const static float WAITING_STATE_UPDATE_INTERVAL_TIME = 1;
const static unsigned int WAITING_ABORT_TIME_FOR_SUB_GOAL = 3600;
const static unsigned int WAITING_ABORT_TIME_FOR_LAST = 360;
//light
const static int LIGHT_COUNT_TIME = 10; 
const static int WAITING_STATE_PRESSURE_THRESHOLD_FOR_SUB_GOAL = 2000; //meter
//distance
const static int DISTANCE_COUNT_TIME = 2;
const static int WAITING_DISTANCE_THRESHOLD = 200;

class WaitingState : public TaskBase
{
private:
	bool mLightCountSuccessFlag;
	bool mDistanceCountSuccessFlag;

	struct timespec mLastUpdateTime;
	struct timespec mWaitingStartTime;
	struct timespec mStartLightCheckTime;

	unsigned int mContinuousLightCount;
	unsigned int mContinuousDistanceCount;

	void CheckLightCount(const struct timespec& time);
	void CheckDistanceCount(const struct timespec& time);
	bool mMissionFlag; 
	bool isWifiFlag;
	bool isLoraFlag;
	bool mCoupleFlag;
	double mMaxAltitude;
	struct timespec mStartTime;

protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();
	void nextState();

public:
	void SetMissionFlag(bool flag);
	WaitingState();
	~WaitingState();
};

extern WaitingState gWaitingState;
