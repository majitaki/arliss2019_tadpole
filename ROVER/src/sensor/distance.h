#pragma once
#include "../rover_util/task.h"
#include <ctime>

class DistanceSensor : public TaskBase
{
private:
	struct timespec mLastUpdateTime;
	int mInit;
	int mDistance;
	bool isShowMode;
	bool isParalysised;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onUpdate(const struct timespec& time);
public:
	int getDistance();
	void showData();
	bool checkWorking();
	DistanceSensor();
	~DistanceSensor();
};

extern DistanceSensor gDistanceSensor;
