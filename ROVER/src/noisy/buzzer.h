#pragma once
#include <pthread.h>
#include <list>
#include "../rover_util/task.h"

const static int PIN_BUZZER = 2;             

class Buzzer : public TaskBase
{
private:
	int mPin;
	int mOnPeriodMemory;	//�炷���Ԃ�ێ�
	int mOnPeriod;			//0�ȏ�Ȃ�炷�A���Ȃ�炳�Ȃ�
	int mOffPeriodMemory;	//�炳�Ȃ����Ԃ�ێ�
	int mOffPeriod;			//0�ȏ�Ȃ�炳�Ȃ�
	int mCount;				//�u�U�[��炷��
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onUpdate(const struct timespec& time);
	virtual void restart();

public:
	//���Ɏw�肵�Ȃ��ꍇ�̃u�U�[�̊Ԋu
	const static int DEFAULT_OFF_PERIOD = 500;

	//�u�U�[��period[ms]�����炷(�����͌����ł͂���܂���I)
	void start(int period);

	//(�炷����[ms], �炷��) �u�U�[�𕡐��񐔖炵�����ꍇ�Ɏg�p
	void start(int on_period, int count);

	//(�炷����[ms], �炳�Ȃ�����[ms], �炷��)
	void start(int on_period, int off_period, int count);
	//�u�U�[���~�߂�
	void stop();

	Buzzer();
	~Buzzer();
};

extern Buzzer gBuzzer;
