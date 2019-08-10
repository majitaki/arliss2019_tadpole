#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"


const static float CLOSING_STATE_UPDATE_INTERVAL_TIME = 0.5;
const static unsigned int CLOSING_ABORT_TIME_FOR_SUB_GOAL = 3600;
const static unsigned int CLOSING_ABORT_TIME_FOR_LAST = 5400;
const static unsigned int SNAKY_ABORT_TIME_FOR_LAST = 3000;
const static unsigned int SNAKY_UPDATE_INTERVAL_TIME = 1;
const static float ROTATE_LAST_TIME = 1;
const static unsigned int STOP_LAST_TIME = 2;
const static unsigned int APPROACH_LAST_TIME = 2;

class ClosingState : public TaskBase
{
private:

	enum State {Stop, Rotate, Snaky, Approach};
	enum State mState;
	struct timespec mSnakyLastUpdateTime;
	struct timespec mLastUpdateTime;
	struct timespec mStateUpdateTime;
	struct timespec mClosingStartTime;
	struct timespec mStartTime;
	struct timespec mSnakyStartedTime;

	bool mMissionFlag;

	unsigned int mContinuousLightCount;
	int mDistToGoal; 
	int mDirection;

protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();

	void nextState();

public:
	void SetMissionFlag(bool flag);
	ClosingState();
	~ClosingState();
};

extern ClosingState gClosingState;
