#pragma once
#include <pthread.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"

class Servo : public TaskBase
{
private:
	int mServoPi;
	int mParaServoPin;
	int mDirectServoPin;
	bool isParaServoPidMode;
	bool isDirectServoPidMode;
	int mReleasePulseWidth;
	int mHoldPulseWidth;
	int mRightPulseWidth;
	int mLeftPulseWidth;
	int mCenterPulseWidth;
	int mParaServoTargetPulseWidth;
	int mDirectServoTargetPulseWidth;
	PID mParaServoPID;
	PID mDirectServoPID;
	struct timespec mLastUpdateTime;
	void setParaServoPulseWidth(int pulse_width);
	void setParaServoPulseWidth(int pulse_width, PID &pid);
	void setDirectServoPulseWidth(int pulse_width);
	void setDirectServoPulseWidth(int pulse_width, PID &pid);
protected:
	//èâä˙âª
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual void onUpdate(const timespec& time) override;
	virtual bool onCommand(const std::vector<std::string>& args);

public:
	int mParaServoPulseWidth;
	int mDirectServoPulseWidth;
	void releasePara();
	void holdPara();
	void rightDirect();
	void leftDirect();
	void centerDirect();
	void ParaServo(double value);
	void DirectServo(double value);
	void stopPara();
	void stopDirect();
	int getRelasePulseWidth();
	int getHoldPulseWidth();
	int getRightPulseWidth();
	int getCenterPulseWidth();
	int getLeftPulseWidth();


	Servo();
	~Servo();
};

extern Servo gServo;
