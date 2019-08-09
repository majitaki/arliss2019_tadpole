#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <functional>
#include <stdarg.h>
#include <wiringPi.h>

#include "../rover_util/delayed_execution.h"
#include "../rover_util/utils.h"
#include "../rover_util/serial_command.h"
#include "../constants.h"
#include "../rover_util/logging.h"
#include "falling_sequence.h"
#include "separating_sequence.h"
#include "testing_sequence.h"
#include "waiting_sequence.h"

#include "../sensor/gps.h"
#include "../sensor/light.h"
#include "../sensor/nineaxis.h"
#include "../sensor/pressure.h"
#include "../sensor/distance.h"
#include "../actuator/motor.h"
#include "../actuator/servo.h"
#include "../noisy/buzzer.h"
#include "../noisy/led.h"

FallingState gFallingState;

bool FallingState::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Falling State] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);

	//util
	gSerialCommand.setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	//log
    gUnitedLoggingState.setRunMode(true);
	gMovementLoggingState.setRunMode(true);
	//sensor
	gLightSensor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gNineAxisSensor.setRunMode(true);
	gDistanceSensor.setRunMode(true);
	//actuator
	gServo.setRunMode(true);
	gMotorDrive.setRunMode(true);
	//noise
	gLED.setRunMode(true);
	gBuzzer.setRunMode(true);

	//initialize
	gLED.setColor(255, 255, 0);
	gLED.clearLED();
	mLastUpdateTime = mFallingStartTime = time;
	mLastPressure = gPressureSensor.getPressure();
	mCoutinuousGyroCount = mContinuousPressureCount = 0;
	mGyroCountSuccessFlag = mPressureCountSuccessFlag = false;

	gServo.wrap(1.0);
	gServo.turn(-1.0);

	//if (mNavigatingFlag)
	//{
	//	Debug::print(LOG_SUMMARY, "[Falling State] Wifi Start\r\n");
	//	system("sudo ip link set wlan0 up");
	//}
	return true;
}

void FallingState::onUpdate(const struct timespec& time)
{

	if (Time::dt(time, mLastUpdateTime) < FALLING_STATE_UPDATE_INTERVAL_TIME) return;
	mLastUpdateTime = time;

	CheckGyroCount(time);
	CheckPressureCount(time);
	Debug::print(LOG_SUMMARY, "[Falling State] Duration %f(s) / Sub %d(s) / Abort %d(s)\r\n", Time::dt(time, mFallingStartTime), FALLING_SUBGOAL_TIME, FALLING_ABORT_TIME);


	if (mGyroCountSuccessFlag && mPressureCountSuccessFlag)
	{
		nextState();
		return;
	}
	if (Time::dt(time, mFallingStartTime) > FALLING_SUBGOAL_TIME)
	{
		if (mGyroCountSuccessFlag || mPressureCountSuccessFlag)
		{

			nextState();
			return;
		}
	}

	if (Time::dt(time, mFallingStartTime) > FALLING_ABORT_TIME)
	{
		Debug::print(LOG_SUMMARY, "Falling Timeout\r\n");
		nextState();
		return;
	}
}

void FallingState::onClean()
{
	Debug::print(LOG_SUMMARY, "[Falling State] Finished\r\n");
}

void FallingState::CheckGyroCount(const struct timespec& time)
{
	if (mGyroCountSuccessFlag)return;

	if (mCoutinuousGyroCount == 0)
	{
		mStartGyroCheckTime = time;
		mCoutinuousGyroCount++;
		Debug::print(LOG_SUMMARY, "Gyro Check Initialize\r\n");
	}
	else
	{
		float ax = gNineAxisSensor.getGyro().x();
        float ay = gNineAxisSensor.getGyro().y();
        float az = gNineAxisSensor.getGyro().z();
        float l2_accel = std::sqrt(pow(ax, 2) + std::pow(ay, 2) + std::pow(az, 2));

		if (l2_accel < GYRO_THRESHOLD)
		{
			int diff_time = Time::dt(time, mStartGyroCheckTime);

			if (diff_time > 2) gLED.brink(0.2);

			if (diff_time > GYRO_COUNT_TIME)
			{
				Debug::print(LOG_SUMMARY, "Gyro Check Success.  (Gyro Diff: %f < %f)\r\n", l2_accel, GYRO_THRESHOLD);
				mGyroCountSuccessFlag = true;
				mCoutinuousGyroCount = 0;
				return;
			}
			mCoutinuousGyroCount++;
			Debug::print(LOG_SUMMARY, "Gyro Check Update %d(s) / %d(s)  (Gyro Diff: %f < %f)\r\n", diff_time, GYRO_COUNT_TIME, l2_accel, GYRO_THRESHOLD);
			return;

		}
		else
		{
			Debug::print(LOG_SUMMARY, "Gyro Check Failed.  (Gyro Diff: %f > %f)\r\n", l2_accel, GYRO_THRESHOLD);
			mCoutinuousGyroCount = 0;
			gLED.stopBrink();
			gLED.setColor(255, 255, 0);
			return;
		}
	}
}

void FallingState::CheckPressureCount(const timespec & time)
{
	if (mPressureCountSuccessFlag)return;

	if (mContinuousPressureCount == 0)
	{
		mStartPressureCheckTime = time;
		mContinuousPressureCount++;
		Debug::print(LOG_SUMMARY, "Pressure Check Initialize\r\n");

	}
	else
	{
		float new_pressure = gPressureSensor.getPressure();
		float diff_pressure = abs(new_pressure - mLastPressure);
		mLastPressure = new_pressure;

		if (diff_pressure < PRESSURE_THRESHOLD)
		{
			int diff_time = Time::dt(time, mStartPressureCheckTime);

			if (diff_time > PRESSURE_COUNT_TIME)
			{
				gBuzzer.start(70, 1);
				mPressureCountSuccessFlag = true;
				Debug::print(LOG_SUMMARY, "Pressure Check Success\r\n");
				mContinuousPressureCount = 0;
				return;
			}
			gBuzzer.start(25, 1);

			mContinuousPressureCount++;
			Debug::print(LOG_SUMMARY, "Pressure Check Update %d(s) / %d(s) (Pressure Diff: %f < %f)\r\n", diff_time, PRESSURE_COUNT_TIME, diff_pressure, PRESSURE_THRESHOLD);
			return;
		}
		else
		{
			Debug::print(LOG_SUMMARY, "Pressure Check Failed.  (Pressure Diff: %f > %f)\r\n", diff_pressure, PRESSURE_THRESHOLD);
			mContinuousPressureCount = 0;
			return;
		}
	}
}

void FallingState::nextState()
{
	gLED.clearLED();
	setRunMode(false);

	if (!mNavigatingFlag)
	{
		gTestingState.setRunMode(true);
	}
	else
	{
		gSeparatingState.setRunMode(true);
		gSeparatingState.SetNavigatingFlag(true);
	}
}
void FallingState::SetNavigatingFlag(bool flag)
{
	mNavigatingFlag = flag;
}
FallingState::FallingState()
{
	setName("falling");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
	SetNavigatingFlag(false);
}
FallingState::~FallingState()
{
}
