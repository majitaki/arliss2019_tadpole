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

Digging gDigging;
bool Digging::onInit(const timespec & time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Digging] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	//initialize
	mCurStep = STEP_CHECK_LIE;
	gMotorDrive.setRunMode(true);
	gServo.setRunMode(true);
	mLastUpdateTime = time;
	return true;
}

void Digging::onUpdate(const timespec & time)
{
	switch (mCurStep)
	{
	case CHECK_STATUS:
		if (gNineAxisSensor.isTurnSide() || gNineAxisSensor.isTurnBack())
		{
            setRunMode(false);
			return;
		}
        else
        {
            mLastUpdateTime = time;
            Debug::print(LOG_SUMMARY, "digging!");
            mCurStep = DIG;
        }
    case DIG_OPEN:
        if(mCurDigCount<DIGGIG_COUNT)
        {
            Debug::print(LOG_SUMMARY,"[%d/%d] Digging Count",mCurDigCount,DIGGING_COUNT);
            gServo.move(STABI_ID,STABI_OUTER);
            mCurStep = DIG_CLOSE;
        }
        else
        {
            mLastUpdateTime = time;
            mCurStep = CHECK_STATUS;
        }
        break;
	case DIG_CLOSE:
        gServo.move(STABI_ID,STABI_INNER);
        mCurDigCount++;
        mCurStep = DIG_OPEN;1
		break;
	case :
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

bool Digging::onCommand(const std::vector<std::string>& args)
{
	return false;
}

void Digging::onClean()
{
	gMotorDrive.drive(0);
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Digging] Finished\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
}

Digging::WakingFromTurnBack()
{
	setName("digging");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}

Digging::~Digging()
{
}
