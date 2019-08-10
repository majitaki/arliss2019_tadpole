#pragma once
#include "../rover_util/task.h"
#include <ctime>

class Lora : public TaskBase
{
private:
	int fd;
	struct timespec mLastUpdateTime;
	bool enableGPSsendFlag;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onUpdate(const struct timespec& time);
public:
	void send(const std::string & str);
	void sleep(bool enableSleepMode);
	void enableGPSsend(bool flag);
	Lora();
	~Lora();
};

extern Lora gLora;
