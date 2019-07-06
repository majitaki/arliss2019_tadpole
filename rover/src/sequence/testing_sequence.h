#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"

//�e�X�g�p���
class TestingState : public TaskBase
{
private:
	//�S�[���ʒu
	VECTOR3 mGoalPos;
	VECTOR3 mCurrentPos;
	bool mIsGoalPos;
	bool mIsCurrentPos;
	double distance;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();
public:
	void testAll();
	TestingState();
	~TestingState();
};

extern TestingState gTestingState;