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
	int mCheckCount;
	int mSuccessCount;
	enum SubState
	{
		Initial, Roll, NearGoalNavi, CheckGoal, RunWhile, NearGoal, Fail
	};
	enum SubState mSubState;
	bool isGoalLeft;
	bool isGyroOperation;
	bool mTurnValueChangeFlag;

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