#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"
#include "../sensor/pressure.h"


const static float WAITING_STATE_UPDATE_INTERVAL_TIME = 1;
const static unsigned int WAITING_ABORT_TIME_FOR_SUB_GOAL = 3600;
const static unsigned int WAITING_ABORT_TIME_FOR_LAST = 5400;
//light
const static int LIGHT_COUNT_TIME = 10; 
const static int WAITING_STATE_PRESSURE_THRESHOLD_FOR_SUB_GOAL = 2000; //meter


class WaitingState : public TaskBase
{
private:
	bool mLightCountSuccessFlag;

	struct timespec mLastUpdateTime;
	struct timespec mWaitingStartTime;//��ԊJ�n����
	struct timespec mStartLightCheckTime;

	unsigned int mContinuousLightCount;//���Z���T�[�̔�������������

	void CheckLightCount(const struct timespec& time);
	bool mNavigatingFlag; //�i�r�Q�[�V�����̒��Ŏ��s����Ă��邩�D
	double mMaxAltitude;
	struct timespec mStartTime;//��ԊJ�n����

protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();
	//���̏�ԂɈڍs
	void nextState();

public:
	void SetNavigatingFlag(bool flag);
	WaitingState();
	~WaitingState();
};

extern WaitingState gWaitingState;
