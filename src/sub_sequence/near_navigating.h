#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"


class NearNavigating : public TaskBase
{
private:
	struct timespec mLastUpdateTime;
	PID mNearModePID;
	enum SubState
	{
		Initial, EstimateInitialPosition, RunLittle, EstimateDirection, RunForGoal, CheckGoal, NearGoal
	};
	enum SubState mSubState;
	enum GoalIs
	{
		Null, FarFront, Front, CloseFront, FarSide, Side, CloseSide, FarBack, Back, CloseBack
	};
	enum GoalIs mGoalIs;
	struct timespec mCheckTime;
	VECTOR3 mInitPos;
	VECTOR3 mEstPos;
	int mNearNaviRetryCount;

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