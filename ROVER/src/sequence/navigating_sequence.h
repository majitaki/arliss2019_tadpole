#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"


//�S�[���ւ̈ړ���
class NavigatingState : public TaskBase
{
private:
	enum SubState { InitialRunWhile, Initial, TurningSide, TurningBack, Stucking, Digging, CheckGoalInfo, EstimateDistanceToGoal, CheckDistance, NearGoalNavi, FarGoalNavi, CheckGoal, FarGoal };
	enum SubState mSubState;

	struct timespec mLastUpdateTime;
	struct timespec mInitialRunWhileTime;
	struct timespec mStuckTime;
	struct timespec mWakeTime;
	struct timespec mFreezeTime;

	PID mFarModePID;
	bool FreezeFlag;

	//�S�[���ʒu
	VECTOR3 mGoalPos;
	bool mIsGoalPos;
	double mDistanceToGoal;
    double mMidDistanceToGoal;
	int mSamples;
	double mPrevPIDValue;
	int mCheckStuckCount;
	int mNearNaviCount;

    bool enableMiddleMode;
	bool enableNearNaviMode;
	//bool mNavigatingFlag; //mean mission flag
	bool mMissionFlag; //mean mission flag
	bool mNearNaviFlag; // mean near navi flag
	void onCheckGoalInfo();
	void onEstDistance();
    double onEstMidDistance();
	void navigationFarMode();
	void navigationNearMode();
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

	NavigatingState();
	~NavigatingState();
};

extern NavigatingState gNavigatingState;
