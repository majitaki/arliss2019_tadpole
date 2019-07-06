#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"

//パラ分離状態(サーボを動かしてパラを切り離す)
class SeparatingState : public TaskBase
{
private:
	struct timespec mLastUpdateTime;//前回サーボの向きを更新した時間
	bool mCurServoState;			//現在のサーボの向き(true = 1,false = 0)
	unsigned int mServoCount;		//サーボの向きを変更した回数
	enum STEP { STEP_STABI_OPEN = 0, STEP_WAIT_STABI_OPEN, STEP_SEPARATE, CHECK_STAND, FAIL_STAND, STEP_PRE_PARA_JUDGE, STEP_PARA_JUDGE, STEP_PARA_DODGE, STEP_GO_FORWARD };
	enum STEP mCurStep;
	bool mNavigatingFlag; //ナビゲーションの中で実行されているか．
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);

	//次の状態に移行
	void nextState();
public:
	void SetNavigatingFlag(bool flag);
	SeparatingState();
	~SeparatingState();
};

extern SeparatingState gSeparatingState;