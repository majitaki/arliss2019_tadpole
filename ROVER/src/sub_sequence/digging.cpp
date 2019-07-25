#include "../rover_util/delayed_execution.h"
#include "../rover_util/utils.h"
#include "../rover_util/serial_command.h"
#include "../actuator/motor.h"
#include "../actuator/motor_constant.h"
#include "../constants.h"
#include "../rover_util/logging.h"
#include "../manager/accel_manager.h"
#include "../sequence/testing_sequence.h"
#include "../sequence/navigating_sequence.h"
#include "../sensor/gps.h"
#include "../actuator/servo.h"
//#include "../actuator/servo_constant.h"
#include "digging.h"
#include "digging_constant.h"

Digging gDigging;
bool Digging::onInit(const timespec & time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Digging] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	//initialize
	mCurStep = IncWorm;
	gServo.setRunMode(true);
    gMotorDrive.setRunMode(true);
	mLastUpdateTime = time;
	return true;
}

void Digging::onUpdate(const timespec & time)
{
    double dt = Time::dt(time, mLastUpdateTime);
    if (dt < DIGGING_UPDATE_INTERVAL_TIME)return;
	mLastUpdateTime = time;
    gMotorDrive.drive(0);


	switch (mCurStep)
	{
	case IncWorm:
		Debug::print(LOG_SUMMARY, "[Digging] Digging Start\r\n");

		mInchWormLoopCount = 0;
		mCurStep = IncWorm_Shrink;
		mCheckTime = time;

		break;
	case IncWorm_Shrink:
		//gServo.holdPara();
		gServo.turn(0.0);
		gServo.wrap(1.0);
		//gServo.centerDirect();
		if (mInchWormLoopCount > DIGGING_INCHWORM_LOOP_COUNT)
		{
            setRunMode(false);
            Debug::print(LOG_SUMMARY, "[Digging] Finished !\r\n");
			break;
		}
		dt = Time::dt(time, mCheckTime);
		if (dt < DIGGING_INCHWORM_SHRINK_TIME) break;

		Debug::print(LOG_SUMMARY, "[Digging] IncWorm Loop Count %d / %d\r\n", mInchWormLoopCount++, DIGGING_INCHWORM_LOOP_COUNT);
		mCurStep = IncWorm_Extend;
		mCheckTime = time;
		break;
	case IncWorm_Extend:
		gServo.turn(0.0);
		gServo.wrap(-1.0);
		//gServo.releasePara();
		//gServo.centerDirect();
		dt = Time::dt(time, mCheckTime);
		if (dt < DIGGING_INCHWORM_EXTEND_TIME) break;
		mCurStep = IncWorm_Shrink;
		mCheckTime = time;
		break;
	}
}

bool Digging::onCommand(const std::vector<std::string>& args)
{
	return false;
}

void Digging::onClean()
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Digging] Finished\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
}

Digging::Digging()
{
	setName("digging");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}

Digging::~Digging()
{
}
