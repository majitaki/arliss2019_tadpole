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
#include "../actuator/motor.h"
#include "../constants.h"
#include "../rover_util/logging.h"
#include "separating_sequence.h"
#include "separating_sequence_constant.h"
//#include "../manager/accel_manager.h"
#include "testing_sequence.h"
#include "navigating_sequence.h"
#include "../actuator/servo.h"
#include "../sensor/gps.h"
#include "../sensor/pressure.h"
#include "../sensor/nineaxis.h"

SeparatingState gSeparatingState;
bool SeparatingState::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Separating State] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	gServo.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gNineAxisSensor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gUnitedLoggingState.setRunMode(true);
	gMovementLoggingState.setRunMode(true);

	gServo.waitingHoldPara();
	//gServo.centerDirect();

	mLastUpdateTime = time;
	mCurServoState = false;
	mServoCount = 0;
    mServoOpenCount = 0;
    mServoGetDistanceCount = 0;
	mCurStep = STEP_STABI_OPEN;

	return true;
}
void SeparatingState::onUpdate(const struct timespec& time)
{
	switch (mCurStep)
	{
	case STEP_STABI_OPEN:
		gServo.holdPara();
		//gServo.centerDirect();

		mCurStep = STEP_WAIT_STABI_OPEN;
		mLastUpdateTime = time;
		break;
	case STEP_WAIT_STABI_OPEN:
		if (Time::dt(time, mLastUpdateTime) > 0.5)//�X�^�r�W�J�����ҋ@����
		{
			//����ԂɑJ��
			mLastUpdateTime = time;
			mCurStep = STEP_SEPARATE;
		}
		break;
	case STEP_SEPARATE:
		//�p���V���[�g��؂藣��
		if (Time::dt(time, mLastUpdateTime) < SEPARATING_SERVO_INTERVAL)return;
		mLastUpdateTime = time;

		mCurServoState = !mCurServoState;

		if (mCurServoState)
		{
			gServo.releasePara();
			//gServo.centerDirect();
		}
		else
		{
			gServo.holdPara();
			//gServo.centerDirect();
		}

		++mServoCount;
		Debug::print(LOG_SUMMARY, "Separating...(%d/%d)\r\n", mServoCount, SEPARATING_SERVO_COUNT);

		if (mServoCount >= SEPARATING_SERVO_COUNT)//�T�[�{���K��񐔓�������
		{
			//����ԂɑJ��
            //gServo.holdPara();
            //gServo.wrap(-1.0);
			mLastUpdateTime = time;
            mCurServoState = true;
            mCurStep = STEP_SEPARATE_OPEN;
			//gWakingState.setRunMode(true);
			//nextState();
		}
		break;
   case STEP_SEPARATE_OPEN:
        if (Time::dt(time, mLastUpdateTime) < SEPARATING_SERVO_INTERVAL)return;
        mLastUpdateTime = time;
        if(mCurServoState)
        {
            ++mServoOpenCount;
            gServo.holdPara();
        }
        else
        {
            gServo.wrap(-1.0);
        }
        mCurServoState = !mCurServoState;
        Debug::print(LOG_SUMMARY, "\rOpening...(%d/%d)", mServoOpenCount,SEPARATING_OPEN_COUNT);
        if (mServoOpenCount >= SEPARATING_OPEN_COUNT)
        {
            mCurStep = STEP_GET_DISTANCE;
            mCurServoState = true;
            mLastUpdateTime = time;
            gServo.wrap(0.0);
        }
        break;
   case STEP_GET_DISTANCE:
        if (Time::dt(time, mLastUpdateTime) < SEPARATING_SERVO_INTERVAL)return;
        mLastUpdateTime = time;
        if(gNineAxisSensor.isTurnSide())
        {
            if(mCurServoState)
            {
                if(gNineAxisSensor.whichSide()==1)
                    gServo.turn(1.0);
                else if(gNineAxisSensor.whichSide()==-1)
                    gServo.turn(-1.0);
            }
            else
            {
                gServo.turn(0.0);
            }
        }
        else if(gNineAxisSensor.isTurnBack())
        {
            if(mCurServoState)
            {
                gServo.wrap(-1);
            }
            else
            {
                gServo.holdPara();
            }
        }
        ++mServoGetDistanceCount;
        mCurServoState = !mCurServoState;
        Debug::print(LOG_SUMMARY, "\rOpening...(%d/%d)", mServoGetDistanceCount,SEPARATING_GET_DISTANCE_COUNT);
        if(mServoGetDistanceCount >= SEPARATING_GET_DISTANCE_COUNT)
        {
            Debug::print(LOG_SUMMARY, "separting finished !");
            nextState();
        }
        break;
	};
}
void SeparatingState::nextState()
{
	//���̏�Ԃ��I��
    //gServo.holdPara();
    //gMotorDrive.drive(100);
	setRunMode(false);
    gServo.turn(0.0);

	if (!mNavigatingFlag)
	{
		gTestingState.setRunMode(true);
	}
	else
	{
		gNavigatingState.setRunMode(true);
		gNavigatingState.SetNavigatingFlag(true);
	}


}
void SeparatingState::SetNavigatingFlag(bool flag)
{
	mNavigatingFlag = flag;
}
SeparatingState::SeparatingState() : mCurServoState(false), mServoCount(0)
{
	setName("separating");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
	SetNavigatingFlag(false);
}
SeparatingState::~SeparatingState()
{
}
