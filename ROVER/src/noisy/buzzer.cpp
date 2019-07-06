#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <time.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <unistd.h>
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
#include "../rover_util/utils.h"
#include "buzzer.h"

Buzzer gBuzzer;
bool Buzzer::onInit(const struct timespec& time)
{
	pinMode(mPin, OUTPUT);
	digitalWrite(mPin, LOW);
	return true;
}
void Buzzer::onClean()
{
	digitalWrite(mPin, LOW);
}
bool Buzzer::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	int period, on_period, off_period, count;
	switch (args.size())
	{
	case 2:
		if (args[1].compare("stop") == 0)		//buzzer stop
		{
			if (mOnPeriod == 0 && mOffPeriod == 0 && mCount == 0)
			{
				Debug::print(LOG_PRINT, "Buzzer is already stopping\r\n");
			}
			else
			{
				Debug::print(LOG_PRINT, "Stop Command Executed!\r\n");
				mOffPeriod = 0;
				mCount = 1;
				stop();
			}
			return true;
		}
		else									//buzzer [period]
		{
			Debug::print(LOG_PRINT, "Start Command Executed!\r\n");
			period = atoi(args[1].c_str());
			start(period);
			return true;
		}
		break;

	case 3:										//buzzer [period] [count]
		Debug::print(LOG_PRINT, "Start Command Executed!\r\n");
		period = atoi(args[1].c_str());
		count = atoi(args[2].c_str());
		start(period, count);
		return true;
		break;

	case 4:									//buzzer [on oeriod] [off period] [count]
		Debug::print(LOG_PRINT, "Start Command Executed!\r\n");
		on_period = atoi(args[1].c_str());
		off_period = atoi(args[2].c_str());
		count = atoi(args[3].c_str());
		start(on_period, off_period, count);
		return true;
		break;
	default:
		break;
	}

	Debug::print(LOG_PRINT, 
"buzzer [period]: wake buzzer while period\r\n\
buzzer [period] [count]: wake buzzer several times (COUNT)\r\n\
buzzer [on period] [off period] [count]: wake buzzer several times (COUNT)\r\n\
buzzer stop: stop buzzer\r\n");
	return true;
}
void Buzzer::onUpdate(const struct timespec& time)
{
	if (mOffPeriod == 1)		//–Â‚ç‚³‚È‚¢ŽžŠÔ‚ÌI—¹
	{
		restart();
	}
	else if (mOffPeriod > 0)	//–Â‚ç‚³‚È‚¢ŽžŠÔ
	{
		--mOffPeriod;
		return;
	}

	if (mOnPeriod == 1)		//–Â‚ç‚·ŽžŠÔ‚ÌI—¹
	{
		stop();
	}
	else if (mOnPeriod > 0)	//–Â‚ç‚·ŽžŠÔ
	{
		--mOnPeriod;
	}
}
void Buzzer::start(int period)
{
	start(period, 1, 1);
}
void Buzzer::start(int on_period, int count)
{
	start(on_period, DEFAULT_OFF_PERIOD, count);
}
void Buzzer::start(int on_period, int off_period, int count)
{
	if (mOnPeriod == 0 && on_period >= 1 && off_period >= 1 && count >= 1)
	{
		mOnPeriodMemory = on_period;
		mOnPeriod = on_period;
		mOffPeriodMemory = off_period;
		mOffPeriod = 0;
		mCount = count;
		digitalWrite(mPin, HIGH);
	}
}
void Buzzer::restart()
{
	digitalWrite(mPin, HIGH);
	mOnPeriod = mOnPeriodMemory;
	mOffPeriod = 0;
}
void Buzzer::stop()
{
	mOnPeriod = 0;
	digitalWrite(mPin, LOW);

	if (mCount == 1)
	{
		mCount = 0;
	}
	else if (mCount > 0)
	{
		mOffPeriod = mOffPeriodMemory;
		--mCount;
	}
}
Buzzer::Buzzer() :mPin(PIN_BUZZER), mOnPeriodMemory(0), mOnPeriod(0), mOffPeriodMemory(0), mOffPeriod(0), mCount(0)
{
	setName("buzzer");
	setPriority(TASK_PRIORITY_ACTUATOR, TASK_INTERVAL_ACTUATOR);
}
Buzzer::~Buzzer()
{
}