#pragma once
#include "../rover_util/task.h"
#include <ctime>

class Lora : public TaskBase
{
private:
	struct timespec mLastUpdateTime;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onUpdate(const struct timespec& time);
public:
	Lora();
	~Lora();
};

extern Lora gLora;
