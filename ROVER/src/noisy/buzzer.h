#pragma once
#include <pthread.h>
#include <list>
#include "../rover_util/task.h"

const static int PIN_BUZZER = 2;             

class Buzzer : public TaskBase
{
private:
	int mPin;
	int mOnPeriodMemory;	//鳴らす時間を保持
	int mOnPeriod;			//0以上なら鳴らす、負なら鳴らさない
	int mOffPeriodMemory;	//鳴らさない時間を保持
	int mOffPeriod;			//0以上なら鳴らさない
	int mCount;				//ブザーを鳴らす回数
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onUpdate(const struct timespec& time);
	virtual void restart();

public:
	//特に指定しない場合のブザーの間隔
	const static int DEFAULT_OFF_PERIOD = 500;

	//ブザーをperiod[ms]だけ鳴らす(長さは厳密ではありません！)
	void start(int period);

	//(鳴らす時間[ms], 鳴らす回数) ブザーを複数回数鳴らしたい場合に使用
	void start(int on_period, int count);

	//(鳴らす時間[ms], 鳴らさない時間[ms], 鳴らす回数)
	void start(int on_period, int off_period, int count);
	//ブザーを止める
	void stop();

	Buzzer();
	~Buzzer();
};

extern Buzzer gBuzzer;
