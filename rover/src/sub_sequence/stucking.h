#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"


class Stucking : public TaskBase
{
private:
	struct timespec mLastUpdateTime;
	enum SubState { Initial_Checking, Initial, Random_Initial, Random, Random_Checking, Back, Back_Checking, IncWorm, IncWorm_Shrink, IncWorm_Extend, IncWorm_Checking, Avoid_Stucking,Avoid_Stucking_Checking, Final };
	enum SubState mSubState;
	struct timespec mCheckTime;
	int mStuckRetryCount;
	int mRandomCount;
	int mInchWormCount;
	int mInchWormLoopCount;
	int mCheckStuckCount;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();

public:
	Stucking();
	~Stucking();
};

extern Stucking gStucking;