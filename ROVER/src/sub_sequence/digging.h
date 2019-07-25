#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"


class Digging : public TaskBase
{
private:
	struct timespec mLastUpdateTime;
	enum SubState { IncWorm, IncWorm_Shrink, IncWorm_Extend, IncWorm_Checking};
	enum SubState mCurStep;
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
	Digging();
	~Digging();
};

extern Digging gDigging;
