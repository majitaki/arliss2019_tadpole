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

	//���݂̎�����ۑ�
	mStartTime = time;

	//�K�v�ȃ^�X�N���g�p�ł���悤�ɂ���
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

	//������
	mLastUpdateTime = mWaitingStartTime = time;
	mContinuousLightCount = 0;
	mLightCountSuccessFlag = false;
	mMaxAltitude = 0;

	//�T�[�{�ʒu������
	gServo.holdPara();
	gServo.centerDirect();

	////wifi stop
	//if (mNavigatingFlag)
	//{
	//	Debug::print(LOG_SUMMARY, "[Waiting State] Wifi Stop\r\n");
	//	system("sudo ip l set wlan0 down");//������on -> off��
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
			system("sudo ip l set wlan0 down");//������on -> off��
			return true;
		}
	}

	Debug::print(LOG_PRINT, "Failed Command\r\n");
	return false;
}
void WaitingState::onClean()
{
	Debug::print(LOG_SUMMARY, "[Falling State] Wifi Start\r\n");
	system("sudo ip l set wlan0 up");//������on -> off��
	Debug::print(LOG_SUMMARY, "[Waiting State] Finished\r\n");
}

void WaitingState::CheckLightCount(const timespec & time)
{
	//���Z���T�[�����������Ԃ���莞�Ԍp�������烍�[�o�[�͕��o��ԂƗ\��

	if (mLightCountSuccessFlag)return;

	//�W���C���J�E���g�������
	if (mContinuousLightCount == 0)
	{
		mStartLightCheckTime = time;
		mContinuousLightCount++;
		Debug::print(LOG_SUMMARY, "[Waiting State] Light Check Initialize\r\n");
	}
	else
	{

		bool light_react = gLightSensor.get();

		//�W���C���J�E���g�X�V����(臒l�ȉ�)
		if (light_react)
		{
			int diff_time = Time::dt(time, mStartLightCheckTime);



			//gBuzzer.start(10, 10/(LIGHT_COUNT_TIME - diff_time));

			//�W���C���J�E���g�p���X�V����

			if (diff_time > LIGHT_COUNT_TIME)
			{
				Debug::print(LOG_SUMMARY, "[Waiting State] Light Check Success.\r\n");
				mLightCountSuccessFlag = true;
				mContinuousLightCount = 0;
				gBuzzer.start(70, 1);
				return;
			}
			//LED����
			gLED.brink(0.2);
			//�u�U�[
			gBuzzer.start(25, 2);

			mContinuousLightCount++;
			Debug::print(LOG_SUMMARY, "[Waiting State] Light Check Update %d(s) / %d(s)\r\n", diff_time, GYRO_COUNT_TIME);
			return;

		}
		//���Z���T�[�J�E���g�X�V���s�������߁C���Z���T�[�J�E���g�����Z�b�g
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
	//���̏�Ԃ��I��
	setRunMode(false);
	//���̏�Ԃ�ݒ�
	//�i�r�Q�[�V�������łȂ����testing�ɖ߂�
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