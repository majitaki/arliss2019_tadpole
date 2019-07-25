#pragma once
#include <pthread.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"
#include "servo_constant.h"

struct ServoRawData{
	int neck;
	int direct;
	int waist;
	int stabi;
};

class Servo : public TaskBase
{
private:
	int fd;
	PID mParaServoPID;
	PID mDirectServoPID;
	ServoRawData mServoRawData;
	struct timespec mLastUpdateTime;
	double translateToRange(int raw_value, int end_value, int center_value, double end_range);
	int translateToRawValue(double range, int end_value, int center_value, double end_range);
	void registRangeData(int id, int raw_value);
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual void onUpdate(const timespec& time) override;
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	void wrap(double range);	
	void wrapWithoutDirect(double range);
	void wrap(std::string servo_name, double range);
	void turn(double range);	
	void move(int id, int raw_value);	
	void move(std::string servo_name, int raw_value);	
    void holdPara();
	void waitingHoldPara();
    void releasePara();
	void free(int id);	
	void free(std::string servo_name);	
	void free();	
	int getServoID(std::string name);
	int getServoOuterValue(std::string name);
	int getServoInnerValue(std::string name);
	int getServoCenterValue(std::string name);
	std::string getServoName(int id);
	void showRangeData();
	Servo();
	~Servo();
};

extern Servo gServo;
