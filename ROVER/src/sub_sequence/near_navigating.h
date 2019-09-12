#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"


class NearNavigating : public TaskBase
{
private:
	struct timespec mLastUpdateTime;
	struct timespec mLastNearNaviTime;
	struct timespec mCheckTime;
	double turn_value;
	double turn_value2;
	double turn_value3;
	double mInitialYaw;
	int mCheckCount;
	int mSuccessCount;
	int mInfinityCount;
	int count;
	int mTurnSideBackCount;
	enum SubState
	{
		Initial, Infinity, Roll, NearGoalNavi, CheckGoal, RunWhile, NearGoal, Fail
	};
	enum SubState mSubState;
	bool isGoalLeft;
	bool isGyroOperation;
	bool isInfinityOperation;
	bool mTurnValueChangeFlag;
	bool updateFlag;
	void navigationNearMode();
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();
	void nextState();

public:
	NearNavigating();
	~NearNavigating();
};

extern NearNavigating gNearNavigating;