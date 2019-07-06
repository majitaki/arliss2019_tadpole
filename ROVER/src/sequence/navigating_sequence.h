#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"


//�S�[���ւ̈ړ���
class NavigatingState : public TaskBase
{
private:
	enum SubState { InitialRunWhile, Initial, TurningSide, TurningBack, Stucking, CheckGoalInfo, EstimateDistanceToGoal, CheckDistance, NearGoalNavi, FarGoalNavi, CheckGoal, FarGoal };
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
	int mSamples;
	double mPrevPIDValue;
	int mCheckStuckCount;
	int mNearNaviCount;

	bool mNavigatingFlag; //�i�r�Q�[�V�����̒��Ŏ��s����Ă��邩�D
	void onCheckGoalInfo();
	void onEstDistance();
	void navigationFarMode();
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();
	void nextState();
public:
	void setGoal(const VECTOR3& pos);
	bool getGoal(VECTOR3& pos);
	void SetNavigatingFlag(bool flag);

	NavigatingState();
	~NavigatingState();
};

extern NavigatingState gNavigatingState;