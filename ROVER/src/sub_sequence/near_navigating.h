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
	enum SubState
	{
		NearGoalNavi, CheckGoal, NearGoal
	};
	enum SubState mSubState;
	bool isGoalLeft;

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