#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"

const static float FALLING_STATE_UPDATE_INTERVAL_TIME = 1;
const static unsigned int FALLING_SUBGOAL_TIME = 900;
const static unsigned int FALLING_ABORT_TIME = 1200;

//gyro
const static int GYRO_COUNT_TIME = 10; //ジャイロカウントの継続時間(秒)
const static float GYRO_THRESHOLD = 15;
//pressure
const static int PRESSURE_COUNT_TIME = 10; //大気圧カウントの継続時間(秒)
const static double PRESSURE_THRESHOLD = 0.8;

class FallingState : public TaskBase
{
private:
	bool mGyroCountSuccessFlag;
	bool mPressureCountSuccessFlag;

	struct timespec mLastUpdateTime;
	struct timespec mFallingStartTime;//状態開始時刻
	struct timespec mStartGyroCheckTime;
	struct timespec mStartPressureCheckTime;

	float mLastPressure;//前回の気圧

	unsigned int mContinuousPressureCount;//気圧が閾値以下の状態が続いた回数
	unsigned int mCoutinuousGyroCount;//角速度が閾値以下の状態が続いた回数

	void CheckGyroCount(const struct timespec& time);
	void CheckPressureCount(const struct timespec& time);

	bool mNavigatingFlag; //ナビゲーションの中で実行されているか．
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual void onClean();

	//次の状態に移行
	void nextState();
public:
	void SetNavigatingFlag(bool flag);
	FallingState();
	~FallingState();
};

extern FallingState gFallingState;