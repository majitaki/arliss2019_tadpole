#include <math.h>
#include "../rover_util/delayed_execution.h"
#include "../rover_util/utils.h"
#include "../rover_util/serial_command.h"
#include "../actuator/motor.h"
#include "../constants.h"
#include "../rover_util/logging.h"
//#include "../manager/accel_manager.h"
#include "testing_sequence.h"
#include "navigating_sequence.h"
#include "../sensor/gps.h"
#include "../sensor/nineaxis.h"
#include "../actuator/servo.h"
#include "navigating_sequence_constant.h"
#include "../sub_sequence/waking_turnside.h"
#include "../sub_sequence/waking_turnback.h"
#include "../sub_sequence/stucking.h"
//#include "../sub_sequence/near_navigating.h"
#include "../sub_sequence/digging.h"


NavigatingState gNavigatingState;
//�S�[���ւ̈ړ���
bool NavigatingState::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Navigating State] Start\r\n");
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Time::showNowTime();


	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gNineAxisSensor.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gUnitedLoggingState.setRunMode(true);
	gServo.setRunMode(true);

	gServo.wrap(0.0);
	//gGPSSensor.clearSample();
	mSubState = InitialRunWhile;
	mDistanceToGoal = 999999;
	mLastUpdateTime = time;
	mInitialRunWhileTime = time;
	mFarModePID = PID(NAVIGATING_UPDATE_INTERVAL_TIME, NAVIGATING_MAX_DELTA_ANGLE, -NAVIGATING_MAX_DELTA_ANGLE, 1, 3, 0.8);
	mCheckStuckCount = 0;
	mNearNaviCount = 0;
	mStuckTime = time;
	mFreezeTime = time;
	FreezeFlag = false;
    
    mMidDistanceToGoal = onEstMidDistance(); 
    enableMiddleMode = true;
    Debug::print(LOG_SUMMARY, "mMidDistance = %lf\r\n",mMidDistanceToGoal);


	return true;
}
void NavigatingState::onUpdate(const struct timespec& time)
{
	double dt = Time::dt(time, mLastUpdateTime);
	if (dt < NAVIGATING_UPDATE_INTERVAL_TIME)return;
	mLastUpdateTime = time;

	switch (mSubState)
	{
	case InitialRunWhile:
		double dt = Time::dt(time, mInitialRunWhileTime);
		if (dt > NAVIGATING_INITIAL_RUN_WHILE_TIME) mSubState = Initial;
		//gGPSSensor.clearSamples();
		gMotorDrive.drive(100);
		gServo.wrap(0.0);
		return;
	}

	if (gWakingFromTurnSide.isActive())return;
	if (gWakingFromTurnBack.isActive())return;
	if (gStucking.isActive())return;
    if (gDigging.isActive())return;
	//if (gNearNavigating.isActive())return;

	switch (mSubState)
	{
	case Initial:
	{
		//Debug::print(LOG_SUMMARY, "[Navi]Initial\r\n");
		if (!mIsGoalPos) { mSubState = CheckGoalInfo; break; }

		if (FreezeFlag) {
			FreezeFlag = false;
			mFreezeTime = time;
			Debug::print(LOG_SUMMARY, "[Navi] Freeze\r\n");
		}
		double dt_freeze = Time::dt(time, mFreezeTime);
		if (dt_freeze > NAVIGATING_FREEZE_TIME)
		{
			if (gNineAxisSensor.isTurnSide()) { mSubState = TurningSide;  FreezeFlag = true; break; }
			if (gNineAxisSensor.isTurnBack()) { mSubState = TurningBack; FreezeFlag = true; break; }
			if (gGPSSensor.isStuckGPS()) { mSubState = Stucking; FreezeFlag = true; break; }

		}

		mCheckStuckCount = 0;
		mSubState = EstimateDistanceToGoal;
		break;
	}
	case CheckGoalInfo:
		//Debug::print(LOG_SUMMARY, "[Navi]CheckGoalInfo\r\n");
		onCheckGoalInfo();
		break;
	case TurningSide:
		Debug::print(LOG_SUMMARY, "[Navi]TurningSide\r\n");
		gWakingFromTurnSide.setRunMode(true);
		mSubState = InitialRunWhile;
		FreezeFlag = true;
		break;
	case TurningBack:
		Debug::print(LOG_SUMMARY, "[Navi]TurningBack\r\n");
		gWakingFromTurnBack.setRunMode(true);
		mSubState = InitialRunWhile;
		FreezeFlag = true;
		break;
	case Stucking:
		Debug::print(LOG_SUMMARY, "[Navi]Stuck Checking: %d / %d\r\n", mCheckStuckCount, NAVIGATING_STUCK_COUNT);
		if (mCheckStuckCount++ > NAVIGATING_STUCK_COUNT)
		{
			gStucking.setRunMode(true);
			mSubState = InitialRunWhile;
		}
		FreezeFlag = true;
		break;
    case Digging:
        enableMiddleMode = false; 
        Debug::print(LOG_SUMMARY,"[Navi]Digging");
        gDigging.setRunMode(true);
        mSubState = InitialRunWhile;
        FreezeFlag = true;
        break;
	case EstimateDistanceToGoal:
		//Debug::print(LOG_SUMMARY, "[Navi]EstDistance\r\n");
		onEstDistance();
		mSubState = CheckDistance;
		break;
	case CheckDistance:
		//Debug::print(LOG_SUMMARY, "[Navi]CheckDistance\r\n");
		if (mDistanceToGoal < 0)
		{
			gMotorDrive.drive(100);
			gServo.wrapWithoutDirect(0.0);
			mSubState = Initial;
			Debug::print(LOG_SUMMARY, "[Navi]No GPS\r\n");
			break;
		}
		Debug::print(LOG_SUMMARY, "distance;%f middistance;%f \r\n", mDistanceToGoal, mMidDistanceToGoal);
        if(mDistanceToGoal < mMidDistanceToGoal && enableMiddleMode)
        {
            Debug::print(LOG_SUMMARY, "NAVIGATING : MidPoint reached!\r\n");
            mSubState = Digging;
            break;
        }

		if (mDistanceToGoal >= NAVIGATING_GOAL_FAR_DISTANCE_THRESHOLD || mNearNaviCount > NAVIGATING_NEAR_MODE_LIMIT)
		{
			mSubState = FarGoalNavi;
		}
		else
		{
			mSubState = FarGoalNavi;
			//mSubState = NearGoalNavi;
		}
		break;
	case FarGoalNavi:
		gMotorDrive.drive(100);
		gServo.wrapWithoutDirect(0.0);
		//gServo.releasePara();

		Debug::print(LOG_SUMMARY, "[Navi]FarGoal\r\n");
		navigationFarMode();
		mSubState = CheckGoal;
		break;
	case NearGoalNavi:
		//gMotorDrive.drive(50);
		//gServo.releasePara();

		Debug::print(LOG_SUMMARY, "[Navi]NearGoal %d / %d\r\n", mNearNaviCount++, NAVIGATING_NEAR_MODE_LIMIT);
		//navigationFarMode();
		//gNearNavigating.setRunMode(true);
		mSubState = Initial;
		break;
	case CheckGoal:
		//Debug::print(LOG_SUMMARY, "[Navi]CheckGoal\r\n");
		if (mDistanceToGoal <= NAVIGATING_GOAL_FAR_DISTANCE_THRESHOLD)
		{
			mSubState = FarGoal;
		}
		else
		{
			mSubState = Initial;
		}
		break;
	case FarGoal:
		gMotorDrive.drive(0);
		gServo.wrap(0.0);
		Debug::print(LOG_SUMMARY, "[Navi]Far Mode Goal\r\n");
		Debug::print(LOG_SUMMARY, "Navigating Finish Point:(%f %f)\r\n", gGPSSensor.getPosx(), gGPSSensor.getPosy());
		Time::showNowTime();
		nextState();
		break;
	default:
		break;

	}
}

bool NavigatingState::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	switch (args.size())
	{
	case 1:
		Debug::print(LOG_PRINT, "navigating                 : get goal\r\n\
							navigating [pos x] [pos y] : set goal at specified position\r\n\
							navigating here            : set goal at current position\r\n\
							navigating goal            : call nextState\r\n\n");

		if (mIsGoalPos)
		{
			Debug::print(LOG_PRINT, "Current Goal (%f %f)\r\n", mGoalPos.x, mGoalPos.y);
		}
		else
            enableMiddleMode = true;
		{
			Debug::print(LOG_PRINT, "NO Goal\r\n");
		}
		return true;
	case 2:
		if (args[1].compare("here") == 0)
		{
			VECTOR3 pos;
			if (!gGPSSensor.get(pos))
			{
				Debug::print(LOG_PRINT, "Unable to get current position!\r\n");
				return true;
			}

			setGoal(pos);
			return true;
		}
		else if (args[1].compare("goal") == 0)
		{
			nextState();
			return true;
		}
		return true;
	case 3:
		VECTOR3 pos;
		pos.x = atof(args[1].c_str());
		pos.y = atof(args[2].c_str());
		setGoal(pos);
		return true;
	}
}
void NavigatingState::onClean()
{
	gMotorDrive.drive(0);
	gServo.wrap(0.0);
	gServo.turn(0.0);
	gServo.free();
	Debug::print(LOG_SUMMARY, "[Navigating State] Finished\r\n");
}
void NavigatingState::onCheckGoalInfo()
{
	Debug::print(LOG_SUMMARY, "NAVIGATING : Please set goal!\r\n");
	gMotorDrive.drive(0);
	gServo.wrap(0.0);
	gServo.turn(0.0);
	gServo.free();
	nextState();
	return;
}
void NavigatingState::onEstDistance()
{
	VECTOR3 current_pos;
	if (!gGPSSensor.getAvePos(current_pos))
	{
		mDistanceToGoal = -1;
		return;
	}

	mDistanceToGoal = VECTOR3::calcDistanceXY(current_pos, mGoalPos);
}
double NavigatingState::onEstMidDistance()
{
    onEstDistance();
    return mDistanceToGoal*3/4;
}
void NavigatingState::navigationFarMode()
{
	double roverAngle;
	double goalAngle;

	VECTOR3 currentPos;
	gGPSSensor.get(currentPos, false);

	goalAngle = VECTOR3::calcAngleXY(currentPos, mGoalPos);//�S�[���̕���
	gGPSSensor.getDirectionAngle(roverAngle);

	double deltaAngle = 0;
	deltaAngle = gNineAxisSensor.normalizeAngle(roverAngle - goalAngle);//�Ԃ̊p�x
	auto max_angle = NAVIGATING_MAX_DELTA_ANGLE;
	deltaAngle = std::max(std::min(deltaAngle, max_angle), -1 * max_angle);

	double currentSpeed = gGPSSensor.getSpeed();
	Debug::print(LOG_SUMMARY, "Speed: %f \r\n", currentSpeed);
	Debug::print(LOG_SUMMARY, "Distance: %f \r\n", mDistanceToGoal);
	Debug::print(LOG_SUMMARY, "Goal Angle: %f Rover Angle: %f Delta Angle: %f(%s)\r\n", goalAngle, roverAngle, deltaAngle, deltaAngle > 0 ? "Left" : "Right");

	double inc = mFarModePID.calculate(0, deltaAngle) / max_angle;
	inc *= NAVIGATING_TURN_SLOPE;
	inc = inc > NAVIGATING_TURN_SLOPE ? 2 * NAVIGATING_TURN_SLOPE : inc;
	inc = inc < -1 * NAVIGATING_TURN_SLOPE ? -1 * NAVIGATING_TURN_SLOPE : inc;
	gServo.turn(inc);
	Debug::print(LOG_SUMMARY, "current: %f target: %f inc: %f\r\n", deltaAngle, 0.0, inc);
}

//���̏�ԂɈڍs
void NavigatingState::nextState()
{
	Debug::print(LOG_SUMMARY, "Navigating Finised!\r\n");
	setRunMode(false);
	gTestingState.setRunMode(true);
}
void NavigatingState::setGoal(const VECTOR3& pos)
{
	mIsGoalPos = true;
	mGoalPos = pos;
	Debug::print(LOG_SUMMARY, "Set Goal ( %f %f )\r\n", mGoalPos.x, mGoalPos.y);
}
bool NavigatingState::getGoal(VECTOR3 & pos)
{
	if (!mIsGoalPos)return false;
	pos = mGoalPos;
	return true;
}
void NavigatingState::SetNavigatingFlag(bool flag)
{
	mNavigatingFlag = flag;
}
NavigatingState::NavigatingState() :mFarModePID()
{
	setName("navigating");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
	SetNavigatingFlag(false);
}
NavigatingState::~NavigatingState()
{
}
