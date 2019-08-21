#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"


class WakingFromTurnBack : public TaskBase
{
private:
	struct timespec mLastUpdateTime;
	enum STEP { STEP_CHECK_LIE, STEP_WAIT_LIE, STEP_START, STEP_CHANGE, STEP_STOP, STEP_DEACCELERATE, STEP_VERIFY };
	enum STEP mCurStep;
	double mAngleOnBegin;
	unsigned int mWakeRetryCount;
	int mStartPower;
	double mAngleThreshold;
	double mDeaccelerateDuration;
	int count;
	int tmp;
	bool mChangeFlag;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();

public:
	WakingFromTurnBack();
	~WakingFromTurnBack();
};

extern WakingFromTurnBack gWakingFromTurnBack;