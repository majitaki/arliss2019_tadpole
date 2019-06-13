#include <time.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <wiringSerial.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <pigpiod_if2.h>

#include "../rover_util/utils.h"
#include "servo_constant.h"
#include "servo.h"


Servo gServo;
bool Servo::onInit(const struct timespec& time)
{
	mParaServoPin = PIN_PARA_SERVO;
	mDirectServoPin = PIN_DIRECT_SERVO;
	isParaServoPidMode = false;
	isDirectServoPidMode = false;

	mServoPi = pigpio_start(NULL, NULL);
	if (mServoPi < 0) return false;
	return true;
}
void Servo::onClean()
{
	stopDirect();
	stopPara();
	pigpio_stop(mServoPi);
}
void Servo::onUpdate(const timespec & time)
{
	if (!isParaServoPidMode && !isDirectServoPidMode) return;

	double dt = Time::dt(time, mLastUpdateTime);
	if (dt < SERVO_UPDATE_INTERVAL_TIME)return;
	mLastUpdateTime = time;

	if (isParaServoPidMode)
	{
		int inc = mParaServoPID.calculate(mParaServoTargetPulseWidth, mParaServoPulseWidth);
		//Debug::print(LOG_PRINT, "current: %d target: %d inc: %d\r\n", mParaServoPulseWidth, mParaServoTargetPulseWidth, inc);
		mParaServoPulseWidth += inc;
		setParaServoPulseWidth(mParaServoPulseWidth);
		int diff = abs(mParaServoPulseWidth - mParaServoTargetPulseWidth);
		if (diff == 0 || inc == 0) isParaServoPidMode = false;
	}

	if (isDirectServoPidMode)
	{
		int inc = mDirectServoPID.calculate(mDirectServoTargetPulseWidth, mDirectServoPulseWidth);
		//Debug::print(LOG_PRINT, "current: %d target: %d inc: %d\r\n", mDirectServoPulseWidth, mDirectServoTargetPulseWidth, inc);
		mDirectServoPulseWidth += inc;
		setDirectServoPulseWidth(mDirectServoPulseWidth);
		int diff = abs(mDirectServoPulseWidth - mDirectServoTargetPulseWidth);
		if (diff == 0 || inc == 0) isDirectServoPidMode = false;
	}


}
bool Servo::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	Debug::print(LOG_PRINT, "current para:%d direct:%d\r\n", mParaServoPulseWidth, mDirectServoPulseWidth);

	switch (args.size())
	{
	case 1:
		Debug::print(LOG_PRINT,
			"\r\n\
servo para                      : set para servo pulse width (hold:%d, release:%d)\r\n\
servo direct                    : set direct servo pulse width (left:%d, center;%d right:%d)\r\n\
servo para p                    : set para servo pulse width with PID\r\n\
servo direct p                  : set direct servo pulse width with PID\r\n\
servo release                   : release para and center direct\r\n\
servo hold                      : hold para and center direct\r\n\
servo right                     : right direct\r\n\
servo left                      : left direct\r\n\
servo center                    : center direct\r\n\
servo stop                      : stop para and direct\r\n\
servo stop para                 : stop para\r\n\
servo stop direct               : stop direct\r\n\
servo set release [pulse_width] : set relase pulse\r\n\
servo set hold [pulse_width]    : set hold pulse\r\n\
servo set center [pulse_width]  : set center pulse\r\n\
servo set right [pulse_width]   : set right pulse\r\n\
servo set left  [pulse_width]   : set left pulse\r\n\
", mHoldPulseWidth, mReleasePulseWidth, mLeftPulseWidth, mCenterPulseWidth, mRightPulseWidth);
		return true;
	case 2:
		if (args[1].compare("release") == 0)
		{
			Debug::print(LOG_PRINT, "Servo Release\r\n");
			releasePara();
			centerDirect();
			return true;
		}
		else if (args[1].compare("hold") == 0)
		{
			Debug::print(LOG_PRINT, "Servo Hold\r\n");
			holdPara();
			centerDirect();
			return true;
		}
		else if (args[1].compare("right") == 0)
		{
			Debug::print(LOG_PRINT, "Servo Right\r\n");
			rightDirect();
			return true;
		}
		else if (args[1].compare("left") == 0)
		{
			Debug::print(LOG_PRINT, "Servo Left\r\n");
			leftDirect();
			return true;
		}
		else if (args[1].compare("center") == 0)
		{
			Debug::print(LOG_PRINT, "Servo Center\r\n");
			centerDirect();
			return true;
		}
		else if (args[1].compare("stop") == 0)
		{
			Debug::print(LOG_PRINT, "All Servo Stop\r\n");
			stopPara();
			stopDirect();
			return true;
		}
		break;
	case 3:
		if (args[1].compare("stop") == 0)
		{
			if (args[2].compare("para") == 0)
			{
				Debug::print(LOG_PRINT, "Para Servo Stop\r\n");
				stopPara();
				return true;
			}
			else if (args[2].compare("direct") == 0)
			{
				Debug::print(LOG_PRINT, "Direct Servo Stop\r\n");
				stopDirect();
				return true;
			}
			break;
		}
		else if (args[1].compare("para") == 0)
		{
			int pulse_width = 0;
			pulse_width = atoi(args[2].c_str());
			setParaServoPulseWidth(pulse_width);
			return true;
		}
		else if (args[1].compare("direct") == 0)
		{
			int pulse_width = 0;
			pulse_width = atoi(args[2].c_str());
			setDirectServoPulseWidth(pulse_width);
			return true;
		}
		break;
	case 4:
		if (args[1].compare("para") == 0 && args[2].compare("p") == 0)
		{
			int pulse_width = 0;
			pulse_width = atoi(args[3].c_str());
			PID pid = PID(SERVO_UPDATE_INTERVAL_TIME, 100, -100, 0.1, 0.01, 0);
			setParaServoPulseWidth(pulse_width, pid);
			return true;
		}
		else if (args[1].compare("direct") == 0 && args[2].compare("p") == 0)
		{
			int pulse_width = 0;
			pulse_width = atoi(args[3].c_str());
			PID pid = PID(SERVO_UPDATE_INTERVAL_TIME, 100, -100, 0.1, 0.01, 0);
			setDirectServoPulseWidth(pulse_width, pid);
			return true;
		}
		else if (args[1].compare("set") == 0 && args[2].compare("release") == 0)
		{
			int pulse_width = 0;
			pulse_width = atoi(args[3].c_str());
			mReleasePulseWidth = pulse_width;
			Debug::print(LOG_PRINT, "Release Pulse is %d\r\n", mReleasePulseWidth);
			return true;
		}
		else if (args[1].compare("set") == 0 && args[2].compare("hold") == 0)
		{
			int pulse_width = 0;
			pulse_width = atoi(args[3].c_str());
			mHoldPulseWidth = pulse_width;
			Debug::print(LOG_PRINT, "Hold Pulse is %d\r\n", mHoldPulseWidth);
			return true;
		}
		else if (args[1].compare("set") == 0 && args[2].compare("center") == 0)
		{
			int pulse_width = 0;
			pulse_width = atoi(args[3].c_str());
			mCenterPulseWidth = pulse_width;
			Debug::print(LOG_PRINT, "Center Pulse is %d\r\n", mCenterPulseWidth);
			return true;
		}
		else if (args[1].compare("set") == 0 && args[2].compare("right") == 0)
		{
			int pulse_width = 0;
			pulse_width = atoi(args[3].c_str());
			mRightPulseWidth = pulse_width;
			Debug::print(LOG_PRINT, "Right Pulse is %d\r\n", mRightPulseWidth);
			return true;
		}
		else if (args[1].compare("set") == 0 && args[2].compare("left") == 0)
		{
			int pulse_width = 0;
			pulse_width = atoi(args[3].c_str());
			mLeftPulseWidth = pulse_width;
			Debug::print(LOG_PRINT, "Left Pulse is %d\r\n", mLeftPulseWidth);
			return true;
		}
		break;
	}
	Debug::print(LOG_PRINT, "Failed Command\r\n");
	return false;
}

void Servo::setParaServoPulseWidth(int pulse_width)
{
	//mParaServoPulseWidth = pulse_width < mHoldPulseWidth ? mHoldPulseWidth : pulse_width;
	//mParaServoPulseWidth = pulse_width > mReleasePulseWidth ? mReleasePulseWidth : pulse_width;
	mParaServoPulseWidth = pulse_width;
	set_servo_pulsewidth(mServoPi, mParaServoPin, mParaServoPulseWidth);
}

void Servo::setParaServoPulseWidth(int pulse_width, PID &pid)
{
	isParaServoPidMode = true;
	mParaServoPID = pid;
	mParaServoTargetPulseWidth = pulse_width < mHoldPulseWidth ? mHoldPulseWidth : pulse_width;
	mParaServoTargetPulseWidth = pulse_width > mReleasePulseWidth ? mReleasePulseWidth : pulse_width;
}

void Servo::setDirectServoPulseWidth(int pulse_width)
{
	//mDirectServoPulseWidth = pulse_width < mRightPulseWidth ? mRightPulseWidth : pulse_width;
	//mDirectServoPulseWidth = pulse_width > mLeftPulseWidth ? mLeftPulseWidth : pulse_width;
	mDirectServoPulseWidth = pulse_width;
	set_servo_pulsewidth(mServoPi, mDirectServoPin, mDirectServoPulseWidth);
}

void Servo::setDirectServoPulseWidth(int pulse_width, PID &pid)
{
	isDirectServoPidMode = true;
	mDirectServoPID = pid;
	mDirectServoTargetPulseWidth = pulse_width < mRightPulseWidth ? mRightPulseWidth : pulse_width;
	mDirectServoTargetPulseWidth = pulse_width > mLeftPulseWidth ? mLeftPulseWidth : pulse_width;
}
void Servo::releasePara()
{
	setParaServoPulseWidth(mReleasePulseWidth);
}
void Servo::holdPara()
{
	setParaServoPulseWidth(mHoldPulseWidth);
}
void Servo::rightDirect()
{
	//PID pid = PID(SERVO_UPDATE_INTERVAL_TIME, 100, -100, 0.1, 0.01, 0);
	setDirectServoPulseWidth(mRightPulseWidth);
}
void Servo::leftDirect()
{
	setDirectServoPulseWidth(mLeftPulseWidth);
}
void Servo::centerDirect()
{
	setDirectServoPulseWidth(mCenterPulseWidth);
}
void Servo::ParaServo(double value)// if 0, hold if 1, release
{
	int pulse = mHoldPulseWidth + value * (mReleasePulseWidth - mHoldPulseWidth);
	setParaServoPulseWidth(pulse);
}
void Servo::DirectServo(double value)//if plus, move left, if minus, move right
{
	value = value > 1.0 ? 1.0 : value;
	value = value < -1.0 ? -1.0 : value;
	int pulse = 0;

	if (value > 0)
	{
		pulse = mCenterPulseWidth + value * (mLeftPulseWidth - mCenterPulseWidth);
	}
	else
	{
		pulse = mCenterPulseWidth + value * (mCenterPulseWidth - mRightPulseWidth);
	}

	//int pulse = RIGHT_PULSEWIDTH_DIRECT_SERVO + value * (LEFT_PULSEWIDTH_DIRECT_SERVO - RIGHT_PULSEWIDTH_DIRECT_SERVO);
	setDirectServoPulseWidth(pulse);
}
void Servo::stopPara()
{
	setParaServoPulseWidth(0);
}
void Servo::stopDirect()
{
	setDirectServoPulseWidth(0);
}
int Servo::getRelasePulseWidth()
{
	return mReleasePulseWidth;
}
int Servo::getHoldPulseWidth()
{
	return  mHoldPulseWidth;
}
int Servo::getRightPulseWidth()
{
	return mRightPulseWidth;
}
int Servo::getCenterPulseWidth()
{
	return mCenterPulseWidth;
}
int Servo::getLeftPulseWidth()
{
	return mLeftPulseWidth;
}
Servo::Servo() :mParaServoPulseWidth(0), mDirectServoPulseWidth(0), mParaServoPID(), mDirectServoPID(), mReleasePulseWidth(RELEASE_PULSEWIDTH_PARA_SERVO), mHoldPulseWidth(HOLD_PULSEWIDTH_PARA_SERVO), mRightPulseWidth(RIGHT_PULSEWIDTH_DIRECT_SERVO), mLeftPulseWidth(LEFT_PULSEWIDTH_DIRECT_SERVO), mCenterPulseWidth(CENTER_PULSEWIDTH_DIRECT_SERVO)
{
	setName("servo");
	//setPriority(TASK_PRIORITY_ACTUATOR, UINT_MAX);
	setPriority(TASK_PRIORITY_ACTUATOR, TASK_INTERVAL_MOTOR);
}
Servo::~Servo()
{
}
