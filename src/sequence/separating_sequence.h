#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"

//�p���������(�T�[�{�𓮂����ăp����؂藣��)
class SeparatingState : public TaskBase
{
private:
	struct timespec mLastUpdateTime;//�O��T�[�{�̌������X�V��������
	bool mCurServoState;			//���݂̃T�[�{�̌���(true = 1,false = 0)
	unsigned int mServoCount;		//�T�[�{�̌�����ύX������
	enum STEP { STEP_STABI_OPEN = 0, STEP_WAIT_STABI_OPEN, STEP_SEPARATE, CHECK_STAND, FAIL_STAND, STEP_PRE_PARA_JUDGE, STEP_PARA_JUDGE, STEP_PARA_DODGE, STEP_GO_FORWARD };
	enum STEP mCurStep;
	bool mNavigatingFlag; //�i�r�Q�[�V�����̒��Ŏ��s����Ă��邩�D
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