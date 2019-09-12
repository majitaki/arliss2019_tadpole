#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"


//�S�[���ւ̈ړ���
class NavigatingState : public TaskBase
{
private:
	enum SubState { InitialRunWhile, Initial, TurningSide, TurningBack, Stucking, Digging, CheckGoalInfo, EstimateDistanceToGoal, CheckDistance, NearGoalNavi, FarGoalNavi, CheckGoal, FarGoal};
	enum SubState mSubState;

	struct timespec mLastUpdateTime;
	struct timespec mLastAbortUpdateTime;
	struct timespec mInitialRunWhileTime;
	struct timespec mStuckTime;
	struct timespec mWakeTime;
	struct timespec mFreezeTime;
	struct timespec mNaviStartTime;
	struct timespec mNaviSequenceStartTime;

	PID mFarModePID;
	bool FreezeFlag;

	VECTOR3 mGoalPos;
	VECTOR3 mStartPos;
	bool mIsGoalPos;
	bool mIsStartPos;
	double mDistanceToGoal;
    double mMidDistanceToGoal;
	int mSamples;
	double mPrevPIDValue;
	int mCheckStuckCount;
	int mNearNaviCount;
	unsigned int mNaviAbortTime;
	bool isNearNaviStart;
	double mDeltaAngle;
    bool enableMiddleMode;
	bool enableNearNaviMode;
	bool mMissionFlag; //mean mission flag
	bool mInitialRunWhileFlag;
	double mTurnValue;
	void onCheckGoalInfo();
	void onEstDistance();
    double onEstMidDistance();
	double getStartDistance();
	void navigationFarMode();
	void setStart();
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();
	void nextState();
public:
	void setGoal(const VECTOR3& pos);
	bool getGoal(VECTOR3& pos);
	void SetMissionFlag(bool flag);
	double getDeltaAngle();
	double getTurnValue();

	NavigatingState();
	~NavigatingState();
};

extern NavigatingState gNavigatingState;
