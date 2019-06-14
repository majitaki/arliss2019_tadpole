#pragma once
#include <pthread.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"
#include "servo_constant.h"

class Servo : public TaskBase
{
private:
	int fd;

	PID mParaServoPID;
	PID mDirectServoPID;
	struct timespec mLastUpdateTime;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual void onUpdate(const timespec& time) override;
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	void wrap(double value);	
	void turn(double value);	
	void move(int id, int raw_value);	
	void move(std::string servo_name, int raw_value);	
	void free(int id);	
	void free(std::string servo_name);	
	void free();	
	int getServoID(std::string name);
	Servo();
	~Servo();
};

extern Servo gServo;
