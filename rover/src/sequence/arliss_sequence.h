#pragma once
#include <time.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"

class ArlissState : public TaskBase
{
private:
protected:
	virtual bool onInit(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();
	//Ÿ‚Ìó‘Ô‚ÉˆÚs
	void nextState();
public:
	ArlissState();
	~ArlissState();
};

extern ArlissState gArlissState;