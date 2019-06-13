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
#include "../pwm/motor.h"
#include "../constants.h"
#include "../rover_util/logging.h"
#include "../manager/accel_manager.h"
#include "../sensor/pressure.h"
#include "../sensor/gps.h"
#include "./falling_sequence.h"
#include "./separating_sequence.h"
#include "testing_sequence.h"
#include "../pwm/servo.h"
#include "../noisy/led.h"
#include "../noisy/buzzer.h"

FallingState gFallingState;

bool FallingState::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Falling State] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	//必要なタスクを使用できるようにする
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
	gServo.setRunMode(true);
	gAccelManager.setRunMode(true);
	gLED.setRunMode(true);
	gLED.setColor(255, 255, 0);
	gBuzzer.setRunMode(true);

	//初期化
	mLastUpdateTime = mFallingStartTime = time;
	mLastPressure = gPressureSensor.getPressure();
	mCoutinuousGyroCount = mContinuousPressureCount = 0;
	mGyroCountSuccessFlag = mPressureCountSuccessFlag = false;

	//サーボ位置初期化
	gServo.holdPara();
	gServo.centerDirect();

//	//	if (mNavigatingFlag)
//	//	{
//	Debug::print(LOG_SUMMARY, "[Falling State] Wifi Start\r\n");
//	system("sudo ip l set wlan0 up");//無線をon -> offに
////	}
	return true;
}

void FallingState::onUpdate(const struct timespec& time)
{

	if (Time::dt(time, mLastUpdateTime) < FALLING_STATE_UPDATE_INTERVAL_TIME) return;
	mLastUpdateTime = time;

	CheckGyroCount(time);
	CheckPressureCount(time);
	Debug::print(LOG_SUMMARY, "[Falling State] Duration %f(s) / Sub %d(s) / Abort %d(s)\r\n", Time::dt(time, mFallingStartTime), FALLING_SUBGOAL_TIME, FALLING_ABORT_TIME);


	//ジャイロカウント，大気圧カウント成功したら次の状態
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

	//一定時間で強制的に次の状態
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
	//ジャイロの値が閾値以下の状態が一定時間継続したらローバーは着地状態と予測

	if (mGyroCountSuccessFlag)return;

	//ジャイロカウント初期状態
	if (mCoutinuousGyroCount == 0)
	{
		mStartGyroCheckTime = time;
		mCoutinuousGyroCount++;
		Debug::print(LOG_SUMMARY, "Gyro Check Initialize\r\n");
	}
	else
	{
		float g_x = gAccelManager.getGx();
		float g_y = gAccelManager.getGy();
		float g_z = gAccelManager.getGz();
		float mean_square_gyro = sqrt(pow(g_x, 2.0) + pow(g_y, 2.0) + pow(g_z, 2.0));

		//ジャイロカウント更新成功(閾値以下)
		if (mean_square_gyro < GYRO_THRESHOLD)
		{
			int diff_time = Time::dt(time, mStartGyroCheckTime);

			//LED制御
			if (diff_time > 2) gLED.brink(0.2);

			//ジャイロカウント継続更新成功

			if (diff_time > GYRO_COUNT_TIME)
			{
				Debug::print(LOG_SUMMARY, "Gyro Check Success.  (Gyro Diff: %f < %f)\r\n", mean_square_gyro, GYRO_THRESHOLD);
				mGyroCountSuccessFlag = true;
				mCoutinuousGyroCount = 0;
				return;
			}
			mCoutinuousGyroCount++;
			Debug::print(LOG_SUMMARY, "Gyro Check Update %d(s) / %d(s)  (Gyro Diff: %f < %f)\r\n", diff_time, GYRO_COUNT_TIME, mean_square_gyro, GYRO_THRESHOLD);
			return;

		}
		//ジャイロカウント更新失敗したため，ジャイロカウントをリセット
		else
		{
			Debug::print(LOG_SUMMARY, "Gyro Check Failed.  (Gyro Diff: %f > %f)\r\n", mean_square_gyro, GYRO_THRESHOLD);
			mCoutinuousGyroCount = 0;
			gLED.stopBrink();
			gLED.setColor(255, 255, 0);
			return;
		}
	}
}

void FallingState::CheckPressureCount(const timespec & time)
{
	//大気圧の差分が閾値以下の状態が一定時間継続したらローバーは着地状態と予測

	if (mPressureCountSuccessFlag)return;

	//大気圧カウント初期状態
	if (mContinuousPressureCount == 0)
	{
		mStartPressureCheckTime = time;
		mContinuousPressureCount++;
		Debug::print(LOG_SUMMARY, "Pressure Check Initialize\r\n");

	}
	else
	{
		//大気圧の差分
		float new_pressure = gPressureSensor.getPressure();
		float diff_pressure = abs(new_pressure - mLastPressure);
		mLastPressure = new_pressure;

		//大気圧更新成功
		if (diff_pressure < PRESSURE_THRESHOLD)
		{
			int diff_time = Time::dt(time, mStartPressureCheckTime);

			//大気圧カウント継続更新成功
			if (diff_time > PRESSURE_COUNT_TIME)
			{
				gBuzzer.start(70, 1);
				mPressureCountSuccessFlag = true;
				Debug::print(LOG_SUMMARY, "Pressure Check Success\r\n");
				mContinuousPressureCount = 0;
				return;
			}
			//ブザー制御
			gBuzzer.start(25, 2);

			mContinuousPressureCount++;
			Debug::print(LOG_SUMMARY, "Pressure Check Update %d(s) / %d(s) (Pressure Diff: %f < %f)\r\n", diff_time, PRESSURE_COUNT_TIME, diff_pressure, PRESSURE_THRESHOLD);
			return;
		}
		//大気圧カウント更新失敗
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
	//この状態を終了
	setRunMode(false);

	//次の状態を設定
	//ナビゲーション中でなければtestingに戻る
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