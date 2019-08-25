#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"


class WakingFromTurnSide : public TaskBase
{
private:
	enum SubState{Lying, Rolling,Bridging, Checking};
	enum SubState mSubState;
	struct timespec mLastUpdateTime;
	struct timespec mCheckTime;
	double mCurrentPower;
	double mSpeedupPeriod;
	int mWakeRetryCount;
	PID mWakePID;
	bool mBridging;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();

public:
	WakingFromTurnSide();
	~WakingFromTurnSide();
};

extern WakingFromTurnSide gWakingFromTurnSide;