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
#include "../sensor/light.h"
#include "../manager/accel_manager.h"
#include "../constants.h"
#include "../rover_util/logging.h"
#include "./testing_sequence.h"
#include "./waiting_sequence.h"
#include "./falling_sequence.h"
#include "../pwm/servo.h"
#include "../noisy/led.h"
#include "../noisy/buzzer.h"

WaitingState gWaitingState;


bool WaitingState::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Waiting State] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	//現在の時刻を保存
	mStartTime = time;

	//必要なタスクを使用できるようにする
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gLightSensor.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	gAccelManager.setRunMode(true);
	gServo.setRunMode(true);
	gLED.setRunMode(true);
	gLED.setColor(255, 0, 255);
	gBuzzer.setRunMode(true);
	gPressureSensor.setRunMode(true);

	//初期化
	mLastUpdateTime = mWaitingStartTime = time;
	mContinuousLightCount = 0;
	mLightCountSuccessFlag = false;
	mMaxAltitude = 0;

	//サーボ位置初期化
	gServo.holdPara();
	gServo.centerDirect();

	////wifi stop
	//if (mNavigatingFlag)
	//{
	//	Debug::print(LOG_SUMMARY, "[Waiting State] Wifi Stop\r\n");
	//	system("sudo ip l set wlan0 down");//無線をon -> offに
	//}

	return true;
}

void WaitingState::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) < WAITING_STATE_UPDATE_INTERVAL_TIME) return;
	mLastUpdateTime = time;

	CheckLightCount(time);

	//update max altitude
	double currentAlt = gPressureSensor.getAltitude();
	if (currentAlt > mMaxAltitude) mMaxAltitude = currentAlt;

	if (mLightCountSuccessFlag)
	{
		nextState();
		return;
	}

	double dt = Time::dt(time, mWaitingStartTime);
	Debug::print(LOG_SUMMARY, "[Waiting State] Abort Check %1.1f / %d(Sub) %d(Last)\r\n", dt, WAITING_ABORT_TIME_FOR_SUB_GOAL, WAITING_ABORT_TIME_FOR_LAST);

	if (dt > WAITING_ABORT_TIME_FOR_SUB_GOAL) {
		Debug::print(LOG_SUMMARY, "[Waiting State] Altitude Check %f / %d\r\n", mMaxAltitude, WAITING_STATE_PRESSURE_THRESHOLD_FOR_SUB_GOAL);
		if (mMaxAltitude > WAITING_STATE_PRESSURE_THRESHOLD_FOR_SUB_GOAL)
		{
			Debug::print(LOG_SUMMARY, "Waiting Second Check Success\r\n");
			nextState();
			return;
		}
	}

	if (dt > WAITING_ABORT_TIME_FOR_LAST)
	{
		Debug::print(LOG_SUMMARY, "Waiting Timeout\r\n");
		nextState();
		return;
	}
}

bool WaitingState::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	switch (args.size())
	{
	case 1:
		Debug::print(LOG_PRINT,
			"\r\n\
waiting wifistop        : wifi stop\r\n\
");
		return true;
	case 2:
		if (args[1].compare("wifistop") == 0)
		{
			Debug::print(LOG_SUMMARY, "[Waiting State] Wifi Stop\r\n");
			system("sudo ip l set wlan0 down");//無線をon -> offに
			return true;
		}
	}

	Debug::print(LOG_PRINT, "Failed Command\r\n");
	return false;
}
void WaitingState::onClean()
{
	Debug::print(LOG_SUMMARY, "[Falling State] Wifi Start\r\n");
	system("sudo ip l set wlan0 up");//無線をon -> offに
	Debug::print(LOG_SUMMARY, "[Waiting State] Finished\r\n");
}

void WaitingState::CheckLightCount(const timespec & time)
{
	//光センサーが反応する状態が一定時間継続したらローバーは放出状態と予測

	if (mLightCountSuccessFlag)return;

	//ジャイロカウント初期状態
	if (mContinuousLightCount == 0)
	{
		mStartLightCheckTime = time;
		mContinuousLightCount++;
		Debug::print(LOG_SUMMARY, "[Waiting State] Light Check Initialize\r\n");
	}
	else
	{

		bool light_react = gLightSensor.get();

		//ジャイロカウント更新成功(閾値以下)
		if (light_react)
		{
			int diff_time = Time::dt(time, mStartLightCheckTime);



			//gBuzzer.start(10, 10/(LIGHT_COUNT_TIME - diff_time));

			//ジャイロカウント継続更新成功

			if (diff_time > LIGHT_COUNT_TIME)
			{
				Debug::print(LOG_SUMMARY, "[Waiting State] Light Check Success.\r\n");
				mLightCountSuccessFlag = true;
				mContinuousLightCount = 0;
				gBuzzer.start(70, 1);
				return;
			}
			//LED制御
			gLED.brink(0.2);
			//ブザー
			gBuzzer.start(25, 2);

			mContinuousLightCount++;
			Debug::print(LOG_SUMMARY, "[Waiting State] Light Check Update %d(s) / %d(s)\r\n", diff_time, GYRO_COUNT_TIME);
			return;

		}
		//光センサーカウント更新失敗したため，光センサーカウントをリセット
		else
		{
			Debug::print(LOG_SUMMARY, "[Waiting State] Light Check Failed.\r\n");
			mContinuousLightCount = 0;
			gBuzzer.start(50);
			gLED.stopBrink();
			gLED.setColor(255, 0, 255);
			return;
		}
	}
}

void WaitingState::nextState()
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
		gFallingState.setRunMode(true);
		gFallingState.SetNavigatingFlag(true);
	}
}
void WaitingState::SetNavigatingFlag(bool flag)
{
	mNavigatingFlag = flag;
}
WaitingState::WaitingState()
{
	setName("waiting");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
	SetNavigatingFlag(false);
}
WaitingState::~WaitingState()
{
}