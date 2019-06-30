#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"


const static float CLOSING_STATE_UPDATE_INTERVAL_TIME = 1;
const static unsigned int CLOSING_ABORT_TIME_FOR_SUB_GOAL = 3600;
const static unsigned int CLOSING_ABORT_TIME_FOR_LAST = 5400;


class CLOSINGState : public TaskBase
{
private:

	struct timespec mLastUpdateTime;
	struct timespec mCLOSINGStartTime;

	bool mNavigatingFlag;

	unsigned int mContinuousLightCount;

protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();

	void nextState();

public:
	void SetNavigatingFlag(bool flag);
	ClosingState();
	~ClosingState();
};

extern ClosingState gClosingState;
