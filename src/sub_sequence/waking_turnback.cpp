#include "../rover_util/delayed_execution.h"
#include "../rover_util/utils.h"
#include "../rover_util/serial_command.h"
#include "../pwm/motor.h"
#include "../pwm/motor_constant.h"
#include "../constants.h"
#include "../rover_util/logging.h"
#include "../manager/accel_manager.h"
#include "../sequence/testing_sequence.h"
#include "../sequence/navigating_sequence.h"
#include "../sensor/gps.h"
#include "../pwm/servo.h"
//#include "../pwm/servo_constant.h"
#include "./waking_turnside.h"
#include "waking_turnback.h"
#include "waking_turnback_constant.h"

WakingFromTurnBack gWakingFromTurnBack;
bool WakingFromTurnBack::onInit(const timespec & time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[WakingTurnBack] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	//initialize
	mCurStep = STEP_CHECK_LIE;
	mStartPower = 50;
	gMotorDrive.setRunMode(true);
	gServo.setRunMode(true);
	gAccelManager.setRunMode(true);
	mWakeRetryCount = 0;
	mLastUpdateTime = time;
	return true;
}

void WakingFromTurnBack::onUpdate(const timespec & time)
{
	double power;
	const static double WAKING_THRESHOLD = 200;

	switch (mCurStep)
	{
	case STEP_CHECK_LIE:
		if (gAccelManager.isTurnSide())
		{
			gWakingFromTurnSide.setRunMode(true);
			mCurStep = STEP_WAIT_LIE;
			return;
		}
		gServo.holdPara();//角度調節
	case STEP_WAIT_LIE:
		if (gWakingFromTurnSide.isActive())return;
		gMotorDrive.drive(mStartPower);
		mLastUpdateTime = time;
		mCurStep = STEP_START;
		break;
	case STEP_STOP:
		if (Time::dt(time, mLastUpdateTime) > 2)
		{
			Debug::print(LOG_SUMMARY, "Waking Timeout : unable to land\r\n");
			setRunMode(false);
			gMotorDrive.drive(0);
		}
		if (!gAccelManager.isTurnBack())
		{
			Debug::print(LOG_SUMMARY, "Waking Landed!\r\n");
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0);
		}
		break;

		double dt;
	case STEP_START:
		if (Time::dt(time, mLastUpdateTime) > 0.5)//���莞�ԉ��]�����m�����Ȃ��ꍇ�����]�s�\�Ɣ��f
		{
			Debug::print(LOG_SUMMARY, "Waking Timeout : unable to spin\r\n");
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0);
		}
		if (gAccelManager.isTurnBack())
		{
			Debug::print(LOG_SUMMARY, "Waking Detected Rotation!\r\n");
			mLastUpdateTime = time;
			mCurStep = STEP_DEACCELERATE;
		}
		break;

	case STEP_DEACCELERATE:	//�������茸������
		dt = Time::dt(time, mLastUpdateTime);
		if (dt > mDeaccelerateDuration)
		{
			Debug::print(LOG_SUMMARY, "Waking Deaccelerate finished!\r\n");
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0);
		}
		else
		{
			int tmp_power = std::max((int)((1 - dt / mDeaccelerateDuration) * (mStartPower / 2/*2�Ŋ���*/)), 0);
			gMotorDrive.drive(tmp_power);
		}
		break;

	case STEP_VERIFY:
		if (Time::dt(time, mLastUpdateTime) <= 2.5)
		{
			return;
		}

		if (!gAccelManager.isTurnBack())
		{
			gServo.releasePara();
			Debug::print(LOG_SUMMARY, "Waking Successed!\r\n");
			setRunMode(false);
			return;
		}
		else
		{
			mLastUpdateTime = time;
			//mCurStep = STEP_START;
			mCurStep = STEP_VERIFY;
			power = std::min((unsigned int)100, mStartPower + ((mWakeRetryCount + 1) * 5));	//���s�񐔂��ƂɃ��[�^�o�͂��グ��
																							//gMotorDrive.drive(power);
			gMotorDrive.drive(power, power, 0);

			if (++mWakeRetryCount > WAKING_TURN_BACK_RETRY_COUNT)
			{
				Debug::print(LOG_SUMMARY, "Waking Failed!\r\n");
				setRunMode(false);
				return;
			}

			gServo.holdPara();
			Debug::print(LOG_SUMMARY, "Waking will be retried (%d / %d) by power %f\r\n", mWakeRetryCount, WAKING_TURN_BACK_RETRY_COUNT, power);
		}
		break;
	}
}

bool WakingFromTurnBack::onCommand(const std::vector<std::string>& args)
{
	return false;
}

void WakingFromTurnBack::onClean()
{
	gServo.holdPara();
	gMotorDrive.drive(0);
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[WakingTurnBack] Finished\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
}

WakingFromTurnBack::WakingFromTurnBack()
{
	setName("waking_turnback");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}

WakingFromTurnBack::~WakingFromTurnBack()
{
}
