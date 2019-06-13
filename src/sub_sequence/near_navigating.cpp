#include <cmath>
#include<stdlib.h>
#include "../rover_util/delayed_execution.h"
#include "../rover_util/utils.h"
#include "../rover_util/serial_command.h"
#include "../pwm/motor.h"
#include "../pwm/motor_constant.h"
#include "../constants.h"
#include "../manager/accel_manager.h"
#include "../pwm/servo.h"
#include "../sensor/gps.h"
#include "../pwm/servo_constant.h"
#include "near_navigating.h"
#include "near_navigating_constant.h"
#include "../sequence/navigating_sequence_constant.h"
#include "../sequence/navigating_sequence.h"
#include "../sequence/testing_sequence.h"

NearNavigating gNearNavigating;

bool NearNavigating::onInit(const timespec & time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Near Navigating] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();

	//initialize
	mLastUpdateTime = time;
	mCheckTime = time;
	mGoalIs = Null;
	mSubState = Initial;
	mNearModePID = PID(NEAR_NAVIGATING_UPDATE_INTERVAL_TIME, NAVIGATING_MAX_DELTA_ANGLE, -NAVIGATING_MAX_DELTA_ANGLE, 1, 0, 0);
	mNearNaviRetryCount = 0;
	return true;
}

void NearNavigating::onUpdate(const timespec & time)
{
	double dt = Time::dt(time, mLastUpdateTime);
	if (dt < NEAR_NAVIGATING_UPDATE_INTERVAL_TIME)return;
	mLastUpdateTime = time;

	VECTOR3 ave_pos;
	VECTOR3 new_pos;
	double distance_ave_and_new = 0;

	switch (mSubState)
	{
	case Initial:
		if (mNearNaviRetryCount++ > NEAR_NAVIGATING_RETRY_COUNT_LIMIT + 1)
		{
			Debug::print(LOG_SUMMARY, "[Near Navi] Failed\r\n");
			setRunMode(false);
			return;
		}
		Debug::print(LOG_SUMMARY, "[Near Navi] Retry Count %d / %d\r\n", mNearNaviRetryCount, NEAR_NAVIGATING_RETRY_COUNT_LIMIT);
		mSubState = EstimateInitialPosition;
		gGPSSensor.clearSamples();
		break;
	case EstimateInitialPosition:
		gMotorDrive.drive(0);
		gServo.releasePara();
		gServo.centerDirect();

		dt = Time::dt(time, mCheckTime);
		if (dt > NEAR_NAVIGATING_INITIAL_TIME_LIMIT)
		{
			Debug::print(LOG_SUMMARY, "[Near Navi] Estimate Initial Position Failed\r\n");
			setRunMode(false);
		}

		if (!gGPSSensor.getAvePos(ave_pos) || !gGPSSensor.get(new_pos)) break;

		distance_ave_and_new = VECTOR3::calcDistanceXY(new_pos, ave_pos);
		Debug::print(LOG_SUMMARY, "[Near Navi] New Ave Distance %3.3f Threshold: %f\r\n", distance_ave_and_new, NEAR_NAVIGATING_ESTIMATE_POSITION_THRESHOLD);

		if (distance_ave_and_new < NEAR_NAVIGATING_ESTIMATE_POSITION_THRESHOLD)
		{
			Debug::print(LOG_SUMMARY, "[Near Navi] Estimate Initial Position Success\r\n");
			mInitPos = new_pos;
			mSubState = RunLittle;
			mCheckTime = time;
		}

		break;
	case RunLittle:
		dt = Time::dt(time, mCheckTime);
		Debug::print(LOG_SUMMARY, "[Near Navi] Run Little %1.1f / %1.1f\r\n", dt, NEAR_NAVIGATING_RUN_LITTLE_INTERVAL_TIME);
		switch (mGoalIs)
		{
		case CloseFront:
		case CloseSide:
		case CloseBack:
			gMotorDrive.drive(50);
			break;
		default:
			gMotorDrive.drive(100);
			break;

		}
		if (dt < NEAR_NAVIGATING_RUN_LITTLE_INTERVAL_TIME) break;

		mSubState = EstimateDirection;
		mCheckTime = time;
		gGPSSensor.clearSamples();
		break;
	case EstimateDirection:
		gMotorDrive.drive(0);
		gServo.releasePara();
		gServo.centerDirect();

		dt = Time::dt(time, mCheckTime);
		if (dt > NEAR_NAVIGATING_EST_DIRECTION_TIME_LIMIT)
		{
			Debug::print(LOG_SUMMARY, "[Near Navi] Estimate Direction Position Failed\r\n");
			setRunMode(false);
		}

		if (!gGPSSensor.getAvePos(ave_pos) || !gGPSSensor.get(new_pos)) break;

		distance_ave_and_new = VECTOR3::calcDistanceXY(new_pos, ave_pos);
		Debug::print(LOG_SUMMARY, "[Near Navi] New Ave Distance %3.3f Threshold: %3.3f\r\n", distance_ave_and_new, NEAR_NAVIGATING_ESTIMATE_POSITION_THRESHOLD);

		if (distance_ave_and_new < NEAR_NAVIGATING_ESTIMATE_POSITION_THRESHOLD)
		{
			Debug::print(LOG_SUMMARY, "[Near Navi] Estimate Initial Position Success\r\n");
			mEstPos = new_pos;
			mSubState = RunForGoal;
			mCheckTime = time;
		}
		break;
	case RunForGoal:
		dt = Time::dt(time, mCheckTime);

		double run_interval_time;
		switch (mGoalIs)
		{
		case FarFront:
		case FarBack:
		case FarSide:
			run_interval_time = NEAR_NAVIGATING_RUN_FOR_GOAL_FAR_INTERVAL_TIME;
			gMotorDrive.drive(100);
			break;
		case Front:
		case Side:
		case Back:
			run_interval_time = NEAR_NAVIGATING_RUN_FOR_GOAL_NORMAL_INTERVAL_TIME;
			gMotorDrive.drive(50);
			break;
		case CloseFront:
		case CloseSide:
		case CloseBack:
			run_interval_time = NEAR_NAVIGATING_RUN_FOR_GOAL_CLOSE_INTERVAL_TIME;
			gMotorDrive.drive(30);
			break;
		default:
			run_interval_time = NEAR_NAVIGATING_RUN_FOR_GOAL_NORMAL_INTERVAL_TIME;
			gMotorDrive.drive(50);
			break;
		}
		Debug::print(LOG_SUMMARY, "[Near Navi] Run For Goal %1.1f / %1.1f\r\n", dt, run_interval_time);

		navigationNearMode();
		if (dt < run_interval_time) break;

		mSubState = CheckGoal;
		mCheckTime = time;
		break;

	case CheckGoal:
	{
		VECTOR3 current_pos;
		double distance_to_goal;
		VECTOR3 goal_pos;

		if (!gNavigatingState.getGoal(goal_pos))
		{
			Debug::print(LOG_SUMMARY, "[Near Navi] Not Goal\r\n");
			setRunMode(false);
		}

		if (!gGPSSensor.getAvePos(current_pos))
		{
			distance_to_goal = -1;
			return;
		}

		distance_to_goal = VECTOR3::calcDistanceXY(mEstPos, goal_pos);
		if (distance_to_goal <= NEAR_NAVIGATING_GOAL_DISTANCE_THRESHOLD)
		{
			mSubState = NearGoal;
		}
		else
		{
			mSubState = Initial;
		}
		break;
	}
	case NearGoal:
		gMotorDrive.drive(0);
		gServo.holdPara();
		Debug::print(LOG_SUMMARY, "[Near Navi]Near Mode Goal\r\n");
		Debug::print(LOG_SUMMARY, "Near Navigating Finish Point:(%f %f)\r\n", gGPSSensor.getPosx(), gGPSSensor.getPosy());
		Time::showNowTime();
		nextState();
		break;
	}
}

bool NearNavigating::onCommand(const std::vector<std::string>& args)
{
	return false;
}

void NearNavigating::onClean()
{
	gMotorDrive.drive(0);
	gServo.holdPara();
	gServo.centerDirect();
	Debug::print(LOG_SUMMARY, "[Near Navi] Finished\r\n");
}

void NearNavigating::nextState()
{
	Debug::print(LOG_SUMMARY, "Near Navigating Finised!\r\n");
	setRunMode(false);
	gTestingState.setRunMode(true);
}

void NearNavigating::navigationNearMode()
{
	double roverAngle;
	double goalAngle;
	double distance_to_goal;
	VECTOR3 goal_pos;
	if (!gNavigatingState.getGoal(goal_pos))
	{
		Debug::print(LOG_SUMMARY, "[Near Navi] Not Goal\r\n");
		setRunMode(false);
	}

	roverAngle = VECTOR3::calcAngleXY(mInitPos, mEstPos);
	goalAngle = VECTOR3::calcAngleXY(mEstPos, goal_pos);
	distance_to_goal = VECTOR3::calcDistanceXY(mEstPos, goal_pos);

	double deltaAngle = 0;
	deltaAngle = gAccelManager.normalize(roverAngle - goalAngle);//ŠÔ‚ÌŠp“x
	//deltaAngle = gAccelManager.normalize(goalAngle - roverAngle);//ŠÔ‚ÌŠp“x
	double max_angle = NAVIGATING_MAX_DELTA_ANGLE;
	deltaAngle = std::max(std::min(deltaAngle, max_angle), -1 * max_angle);

	double currentSpeed = gGPSSensor.getSpeed();
	Debug::print(LOG_SUMMARY, "Speed: %f \r\n", currentSpeed);
	Debug::print(LOG_SUMMARY, "Distance: %f \r\n", distance_to_goal);
	Debug::print(LOG_SUMMARY, "Goal Angle: %f Rover Angle: %f Delta Angle: %f(%s)\r\n", goalAngle, roverAngle, deltaAngle, deltaAngle > 0 ? "Left" : "Right");

	//PID pid = PID(NAVIGATING_UPDATE_INTERVAL_TIME, max_angle, -max_angle, 1, 0, 0);
	double inc = mNearModePID.calculate(0, deltaAngle) / -max_angle;
	gServo.DirectServo(inc);

	Debug::print(LOG_SUMMARY, "current: %f target: %f inc: %f\r\n", deltaAngle, 0.0, inc);
}

NearNavigating::NearNavigating() :mInitPos()
{
	setName("near_navigating");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}

NearNavigating::~NearNavigating()
{
}
