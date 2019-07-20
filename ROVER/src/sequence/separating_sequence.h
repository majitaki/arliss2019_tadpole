#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"

class SeparatingState : public TaskBase
{
private:
	struct timespec mLastUpdateTime;
	bool mCurServoState;			
	unsigned int mServoCount;		
	enum STEP { STEP_STABI_OPEN = 0, STEP_WAIT_STABI_OPEN, STEP_SEPARATE, CHECK_STAND, FAIL_STAND, STEP_PRE_PARA_JUDGE, STEP_PARA_JUDGE, STEP_PARA_DODGE, STEP_GO_FORWARD };
	enum STEP mCurStep;
	bool mNavigatingFlag; 
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);

	//���̏�ԂɈڍs
	void nextState();
public:
	void SetNavigatingFlag(bool flag);
	SeparatingState();
	~SeparatingState();
};

extern SeparatingState gSeparatingState;