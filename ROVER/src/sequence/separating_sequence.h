#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"

class SeparatingState : public TaskBase
{
private:
	struct timespec mStartStepTime;
	struct timespec mLastUpdateTime;
	struct timespec mLastMotorMoveTime;
	bool mCurServoState;			
	unsigned int mServoCount;		
    unsigned int mServoOpenCount;
    unsigned int mServoFightForFreeCount;
    unsigned int mServoGetDistanceCount;
	unsigned int mRetryCount;
	unsigned int mCheckCount;
	unsigned int mDetectedArmorCount;
	enum STEP { STEP_STABI_OPEN = 0, STEP_WAIT_STABI_OPEN, STEP_SEPARATE, CHECK_STAND,STEP_CHECK_IF_INSIDE, FAIL_STAND, STEP_PRE_PARA_JUDGE, STEP_PARA_JUDGE, STEP_PARA_DODGE, STEP_GO_FORWARD,STEP_SEPARATE_OPEN, STEP_FIGHT_FOR_FREE, STEP_GET_DISTANCE,STEP_MOVE_BY_CHIJIKI, STEP_DECIDE_DIRECTION, STEP_STABLE_AWAKE_FROM_SIDE,STEP_RUN_WHILE};
	enum STEP mCurStep;
	enum MOTOR_STEP{STEP_MOTOR_MOVE, STEP_MOTOR_STOP};
	enum MOTOR_STEP mCurMotorStep;
	bool mMissionFlag; 
	bool move;
	bool mReadyFlag;
	bool mChijikiMode;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual void onClean();

	//���̏�ԂɈڍs
	void init(const struct timespec& time);
	void nextState();
public:
	void SetMissionFlag(bool flag);
	SeparatingState();
	~SeparatingState();
};

extern SeparatingState gSeparatingState;
