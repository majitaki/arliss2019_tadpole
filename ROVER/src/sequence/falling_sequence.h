#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"

const static float FALLING_STATE_UPDATE_INTERVAL_TIME = 1;
const static unsigned int FALLING_SUBGOAL_TIME = 900;
const static unsigned int FALLING_ABORT_TIME = 1200;

//gyro
const static int GYRO_COUNT_TIME = 10; //�W���C���J�E���g�̌p������(�b)
const static float GYRO_THRESHOLD = 1;
//pressure
const static int PRESSURE_COUNT_TIME = 5; //��C���J�E���g�̌p������(�b)
<<<<<<< HEAD
const static double PRESSURE_THRESHOLD = 0.4;
=======
const static double PRESSURE_THRESHOLD = 1.0;
>>>>>>> 0949995c9205616afd8ad18b6a17bc12488ae98e

class FallingState : public TaskBase
{
private:
	bool mGyroCountSuccessFlag;
	bool mPressureCountSuccessFlag;

	struct timespec mLastUpdateTime;
	struct timespec mFallingStartTime;//��ԊJ�n����
	struct timespec mStartGyroCheckTime;
	struct timespec mStartPressureCheckTime;

	float mLastPressure;//�O��̋C��

	unsigned int mContinuousPressureCount;//�C����臒l�ȉ��̏�Ԃ���������
	unsigned int mCoutinuousGyroCount;//�p���x��臒l�ȉ��̏�Ԃ���������

	void CheckGyroCount(const struct timespec& time);
	void CheckPressureCount(const struct timespec& time);

	bool mMissionFlag; //�i�r�Q�[�V�����̒��Ŏ��s����Ă��邩�D
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual void onClean();

	//���̏�ԂɈڍs
	void nextState();
public:
	void SetMissionFlag(bool flag);
	FallingState();
	~FallingState();
};

extern FallingState gFallingState;
