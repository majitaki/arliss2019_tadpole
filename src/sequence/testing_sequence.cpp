#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <functional>
#include <stdarg.h>
#include <wiringPi.h>

#include "../rover_util/delayed_execution.h"
#include "testing_sequence.h"
#include "../rover_util/utils.h"
#include "../rover_util/serial_command.h"
#include "../sensor/gps.h"
#include "../sensor/light.h"
#include "../sensor/nineaxis.h"
#include "../sensor/pressure.h"
#include "../pwm/motor.h"
#include "../constants.h"
#include "../rover_util/logging.h"
#include "../manager/accel_manager.h"
#include "../noisy/buzzer.h"
#include "../noisy/led.h"
#include "../pwm/servo.h"
//#include "../sub_sequence/demo.h"

TestingState gTestingState;

bool TestingState::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Testing State] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gAccelManager.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gLightSensor.setRunMode(true);
	gServo.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
	gMovementLoggingState.setRunMode(true);
	gBuzzer.setRunMode(true);
	gLED.setRunMode(true);
	gNineAxisSensor.setRunMode(true);

	//èâä˙âª
	gServo.stopPara();
	gServo.stopDirect();

	std::function<void()> f = [&]()
	{
	};
	auto pExecutable = new DelayedExecutableFunction(f, 1000);
	gDelayedExecutor.add(std::shared_ptr<DelayedExecutable>(pExecutable));

	return true;
}

bool TestingState::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;
	//if (gDemo.isActive()) return true;

	if (args.size() == 2)
	{
		if (args[1].compare("info") == 0)
		{
			Debug::print(LOG_SUMMARY, "*** info ***\r\n");
		}


		if (args[1].compare("sensor") == 0)
		{
			Debug::print(LOG_SUMMARY, "*** Sensor states ***\r\n");

			VECTOR3 vec;
			gGPSSensor.get(vec);
			if (gGPSSensor.isActive())Debug::print(LOG_SUMMARY, " GPS      (%f %f %f)\r\n", vec.x, vec.y, vec.z);
			else Debug::print(LOG_SUMMARY, " GPS is NOT working\r\n");

			if (gPressureSensor.isActive())Debug::print(LOG_SUMMARY, " Pressure (%f) hPa\r\n", gPressureSensor.getPressure());
			else Debug::print(LOG_SUMMARY, " Pressure is NOT working\r\n");



			return true;
		}
		else if (args[1].compare("waking") == 0)
		{
			//gWakingState.setRunMode(true);
		}
		else if (args[1].compare("time") == 0)
		{
			Time::showNowTime();
			return true;
		}
		else if (args[1].compare("version") == 0)
		{
			Debug::print(LOG_SUMMARY, "Version: %d\r\n", VERSION);
			return true;
		}
		else if (args[1].compare("demo") == 0)
		{
			gGPSSensor.isAlive();
			gLightSensor.isAlive();
			gNineAxisSensor.isAlive();
			gPressureSensor.isAlive();
			//gDemo.setRunMode(true);
			return true;
		}
	}
	if (args.size() == 3)
	{
		if (args[1].compare("start") == 0)
		{
			TaskBase* pTask = TaskManager::getInstance()->get(args[2]);
			if (pTask != NULL)
			{
				Debug::print(LOG_SUMMARY, "Start %s\r\n", args[2].c_str());
				pTask->setRunMode(true);
				return true;
			}
			else Debug::print(LOG_SUMMARY, "%s Not Found\r\n", args[2].c_str());
			return false;
		}
		else if (args[1].compare("stop") == 0)
		{
			TaskBase* pTask = TaskManager::getInstance()->get(args[2]);
			if (pTask != NULL)
			{
				Debug::print(LOG_SUMMARY, "Stop %s\r\n", args[2].c_str());
				pTask->setRunMode(false);
				return true;
			}
			else Debug::print(LOG_SUMMARY, "%s Not Found\r\n", args[2].c_str());
			return false;
		}
	}
	if (args.size() == 4)
	{
		if (args[1].compare("pin") == 0)
		{
			if ((int)atof(args[3].c_str()) == 0 || (int)atof(args[3].c_str()) == 1)
			{
				pinMode((int)atof(args[2].c_str()), OUTPUT);
				digitalWrite((int)atof(args[2].c_str()), (int)atof(args[3].c_str()));
				Debug::print(LOG_SUMMARY, "digitalWrite(%d,%d)\r\n", (int)atof(args[2].c_str()), (int)atof(args[3].c_str()));
				return true;
			}
		}
	}
	Debug::print(LOG_PRINT, "testing [start/stop] [task name]  : enable/disable task\r\n\
							testing time                      : show current time\r\n\
							testing sensor                    : check sensor values\r\n\
							testing pin [PinNum] [output(0|1)]: digitalWrite(PinNum,output)\r\n\
							testing version                   : show program version\r\n");

	return true;
}

void TestingState::onClean()
{
	Debug::print(LOG_SUMMARY, "[Testing State] Finished\r\n");
}
TestingState::TestingState()
{
	setName("testing");
	setPriority(UINT_MAX, UINT_MAX);
}
TestingState::~TestingState()
{
}