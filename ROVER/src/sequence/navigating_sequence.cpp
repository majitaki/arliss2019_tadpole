#include <math.h>
#include "../rover_util/delayed_execution.h"
#include "../rover_util/utils.h"
#include "../rover_util/serial_command.h"
#include "../constants.h"
#include "../rover_util/logging.h"
#include "testing_sequence.h"
#include "navigating_sequence.h"
#include "navigating_sequence_constant.h"
//#include "closing_sequence.h"
#include "../sub_sequence/waking_turnside.h"
#include "../sub_sequence/waking_turnback.h"
#include "../sub_sequence/stucking.h"
#include "../sub_sequence/digging.h"
#include "../sub_sequence/near_navigating.h"

#include "../sensor/gps.h"
#include "../sensor/light.h"
#include "../sensor/nineaxis.h"
#include "../sensor/pressure.h"
#include "../sensor/distance.h"
#include "../sensor/lora.h"
#include "../actuator/motor.h"
#include "../actuator/servo.h"
#include "../noisy/buzzer.h"
#include "../noisy/led.h"



NavigatingState gNavigatingState;

bool NavigatingState::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "-------------------------\r\n");
	Debug::print(LOG_SUMMARY, "[Navigating State] Start\r\n");
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
	gLora.setRunMode(true);
	//actuator
	gServo.setRunMode(true);
	gMotorDrive.setRunMode(true);
	//noise
	gLED.setRunMode(true);
	gBuzzer.setRunMode(true);

	//initialize
	gLED.setColor(100, 100, 100);
	gServo.wrap(0.0);
	//gGPSSensor.clearSample();
	mSubState = InitialRunWhile;
	mDistanceToGoal = 999999;
	mLastUpdateTime = mNaviStartTime = mNaviSequenceStartTime = mLastUpdateTime = time;
	//mFarModePID = PID(NAVIGATING_UPDATE_INTERVAL_TIME, NAVIGATING_MAX_DELTA_ANGLE, -NAVIGATING_MAX_DELTA_ANGLE, 3, 0.2, 0.0);
	mCheckStuckCount = 0;
	mNearNaviCount = 0;
	isNearNaviStart = false;
	mNaviAbortTime = NAVIGATING_ABORT_TIME;
	mStuckTime = time;
	mFreezeTime = time;
	FreezeFlag = false;
	mInitialRunWhileFlag = true;
	gLora.enableGPSsend(true);
	mMidDistanceToGoal = onEstMidDistance();
	enableMiddleMode = true;
	enableNearNaviMode = true;
	mIsStartPos = false;
	setStart();
	mTurnValue = 0.0;
	return true;
}
void NavigatingState::onUpdate(const struct timespec& time)
{
	double dt = Time::dt(time, mLastUpdateTime);
	if (dt < NAVIGATING_UPDATE_INTERVAL_TIME)return;
	mLastUpdateTime = time;
	gLora.setSeqName(getName());

	if(!mIsStartPos){
		setStart();
	}else{
		double start_dt = Time::dt(time, mNaviSequenceStartTime);
		if (start_dt > NAVIGATING_START_ABORT_TIME)
		{
			if(getStartDistance() < NAVIGATING_START_ABORT_DISTANCE)
			{
				gMotorDrive.drive(0);
				gServo.free();
				Debug::print(LOG_SUMMARY, "[Navi] Start Abort. Reboot Now\r\n");
				system("sudo reboot now");
				return;
			}
		}
	}


	if (mMidDistanceToGoal < 0 && enableMiddleMode) {
		mMidDistanceToGoal = onEstMidDistance();
		//Debug::print(LOG_SUMMARY, "[Navi] mMidDistance = %lf\r\n", mMidDistanceToGoal);
	}


	if (enableNearNaviMode && isNearNaviStart){
		mNaviStartTime = time;
		isNearNaviStart = false;
		mNaviAbortTime = NAVIGATING_NEAR_ABORT_TIME;
	}

	double abort_update_dt = Time::dt(time, mLastAbortUpdateTime);
	if (abort_update_dt > NAVIGATING_ABORT_UPDATE_INTERVAL_TIME)
	{
		double abort_dt = Time::dt(time, mNaviStartTime);
		Debug::print(LOG_SUMMARY, "[Navi] Abort Check %1.1f / %d\r\n", abort_dt, mNaviAbortTime);

		if (abort_dt > mNaviAbortTime)
		{
			Debug::print(LOG_SUMMARY, "Navigating Timeout\r\n");
			mSubState = FarGoal;
		}
		mLastAbortUpdateTime = time;
	}

	//switch (mSubState)
	//{
	// case InitialRunWhile:
	// 	if(mInitialRunWhileFlag){
	// 		mInitialRunWhileFlag = false;
	// 		mInitialRunWhileTime = time;
	// 		Debug::print(LOG_SUMMARY, "[Navi] InitialRunWhile...\r\n");
	// 	}
	// 	double dt = Time::dt(time, mInitialRunWhileTime);
	// 	if (dt > NAVIGATING_INITIAL_RUN_WHILE_TIME) {
	// 		mSubState = Initial;
	// 		//mInitialRunWhileFlag = true;
	// 	}
	// 	//gGPSSensor.clearSamples();
	// 	gMotorDrive.drive(100);
	// 	gServo.wrap(0.0);
	// 	gServo.turn(0.0);
	// 	return;
	// }

	if (gWakingFromTurnSide.isActive())return;
	if (gWakingFromTurnBack.isActive())return;
	if (gStucking.isActive())return;
    if (gDigging.isActive())return;
	if (gNearNavigating.isActive())return;

	switch (mSubState)
	{
	case InitialRunWhile:
	{
		if(mInitialRunWhileFlag){
			mInitialRunWhileFlag = false;
			mInitialRunWhileTime = time;
			Debug::print(LOG_SUMMARY, "[Navi] InitialRunWhile...\r\n");
		}
		double dt = Time::dt(time, mInitialRunWhileTime);
		if (dt > NAVIGATING_INITIAL_RUN_WHILE_TIME) {
			mSubState = Initial;
		}
		//gGPSSensor.clearSamples();
		gMotorDrive.drive(100);
		gServo.wrap(0.0);
		gServo.turn(0.0);
		return;
	}	
	case Initial:
	{
		//Debug::print(LOG_SUMMARY, "[Navi]Initial\r\n");
		if (!mIsGoalPos) { mSubState = CheckGoalInfo; break; }

		if (FreezeFlag) {
			FreezeFlag = false;
			mFreezeTime = time;
			gMotorDrive.drive(100);
			gServo.wrap(0.0);
			gServo.turn(0.0);
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
		Debug::print(LOG_SUMMARY, "[Navi] TurningSide\r\n");
		gWakingFromTurnSide.setRunMode(true);
		mSubState = Initial;
		FreezeFlag = true;
		break;
	case TurningBack:
		Debug::print(LOG_SUMMARY, "[Navi] TurningBack\r\n");
		gWakingFromTurnBack.setRunMode(true);
		mSubState = Initial;
		FreezeFlag = true;
		break;
	case Stucking:
		Debug::print(LOG_SUMMARY, "[Navi] Stuck Checking: %d / %d\r\n", mCheckStuckCount, NAVIGATING_STUCK_COUNT);
		if (mCheckStuckCount++ > NAVIGATING_STUCK_COUNT)
		{
			gStucking.setRunMode(true);
			mSubState = Initial;
		}
		FreezeFlag = true;
		break;
    case Digging:
        enableMiddleMode = false; 
        Debug::print(LOG_SUMMARY,"[Navi] Digging\r\n");
        gDigging.setRunMode(true);
		mSubState = Initial;
		FreezeFlag = true;
        //mSubState = InitialRunWhile;
        //mInitialRunWhileFlag = true;
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
			gServo.wrap(0.0);
			mSubState = Initial;
			Debug::print(LOG_SUMMARY, "[Navi] No GPS\r\n");
			break;
		}
		//Debug::print(LOG_SUMMARY, "[Navi] distance;%f middistance;%f \r\n", mDistanceToGoal, mMidDistanceToGoal);
        if(mDistanceToGoal < mMidDistanceToGoal && enableMiddleMode)
        {
            Debug::print(LOG_SUMMARY, "[Navi] MidPoint reached!\r\n");
            mSubState = Digging;
            break;
        }

		if(enableNearNaviMode){
			if (mDistanceToGoal >= NAVIGATING_GOAL_FAR_DISTANCE_THRESHOLD || mNearNaviCount > NAVIGATING_NEAR_MODE_LIMIT)
			{
				mSubState = FarGoalNavi;
			}
			else
			{
				isNearNaviStart = true;
				Debug::print(LOG_SUMMARY, "[Navi] Near Navi Count Max\r\n");
				Debug::print(LOG_SUMMARY, "[Navi] Far Mode Goal\r\n");
				Debug::print(LOG_SUMMARY, "[Navi] Navigating Finish Point:(%f %f)\r\n", gGPSSensor.getPosx(), gGPSSensor.getPosy());
				Debug::print(LOG_SUMMARY, "[Navi] But I will try more accuracy...\r\n");
		
				mSubState = NearGoalNavi;
			}
		}
		else
		{
			mSubState = FarGoalNavi;
		}
		break;
	case FarGoalNavi:
		gMotorDrive.drive(100);
		gServo.wrap(0.0);

		Debug::print(LOG_SUMMARY, "[Navi] [FarGoalNavi]\r\n");
		navigationFarMode();
		mSubState = CheckGoal;
		break;
	case NearGoalNavi:
		Debug::print(LOG_SUMMARY, "[Navi] NearGoal %d / %d\r\n", mNearNaviCount++, NAVIGATING_NEAR_MODE_LIMIT);
		gNearNavigating.setRunMode(true);
		mSubState = InitialRunWhile;
        mInitialRunWhileFlag = true;
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
		Debug::print(LOG_SUMMARY, "[Navi] Far Mode Goal\r\n");
		Debug::print(LOG_SUMMARY, "[Navi] Navigating Finish Point:(%f %f)\r\n", gGPSSensor.getPosx(), gGPSSensor.getPosy());
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
	Debug::print(LOG_SUMMARY, "[Navi] Finished\r\n");
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
		//Debug::print(LOG_SUMMARY, "NAVIGATING : onEstDistanc = -1\r\n");
		return;
	}

	mDistanceToGoal = VECTOR3::calcDistanceXY(current_pos, mGoalPos);
}
double NavigatingState::onEstMidDistance()
{
    onEstDistance();
    return mDistanceToGoal * NAVIGATING_MIDDLE_DISTANCE_RATE;
}

double NavigatingState::getStartDistance()
{
	VECTOR3 current_pos;
	if (!gGPSSensor.getAvePos(current_pos))
	{
		return -1;
	}
	return VECTOR3::calcDistanceXY(current_pos, mStartPos);
}

void NavigatingState::navigationFarMode()
{
	double roverAngle;
	double goalAngle;

	VECTOR3 currentPos;
	gGPSSensor.get(currentPos, false);

	goalAngle = VECTOR3::calcAngleXY(currentPos, mGoalPos);
	gGPSSensor.getDirectionAngle(roverAngle);

	double mDeltaAngle = 0;
	mDeltaAngle = gNineAxisSensor.normalizeAngle(roverAngle - goalAngle);
	auto max_angle = NAVIGATING_MAX_DELTA_ANGLE;
	mDeltaAngle = std::max(std::min(mDeltaAngle, max_angle), -1 * max_angle);
	//deltaAngle *= -1;

	double currentSpeed = gGPSSensor.getSpeed();
	Debug::print(LOG_SUMMARY, "[Navi] Speed: %1.1f \r\n", currentSpeed);
	Debug::print(LOG_SUMMARY, "[Navi] Distance: %1.1f Middle: %1.1f Start: %1.1f\r\n", mDistanceToGoal, mMidDistanceToGoal, getStartDistance());
	Debug::print(LOG_SUMMARY, "[Navi] Goal Angle: %1.1f Rover Angle: %1.1f Delta Angle: %1.1f(%s)\r\n", goalAngle, roverAngle, mDeltaAngle, mDeltaAngle > 0 ? "Left" : "Right");

	double turn_slope = NAVIGATING_TURN_SLOPE;
	double upper = turn_slope - (-1 * turn_slope);
	double under = -1 * max_angle - max_angle;
	//double inc = mFarModePID.calculate(0, deltaAngle) * (upper / under);
	double inc = -1 * mDeltaAngle * (upper / under);
	mTurnValue = inc;
	gServo.turnp(inc);
	gServo.turn(0);
	Debug::print(LOG_SUMMARY, "[Navi] current: %1.2f target: %1.2f inc: %1.2f\r\n", mDeltaAngle, 0.0, inc);
}

void NavigatingState::nextState()
{
	gLED.clearLED();
	setRunMode(false);
	//gTestingState.setRunMode(true);

	if (!mMissionFlag)
	{
		gTestingState.setRunMode(true);
	}
	else
	{
		//gClosingState.setRunMode(true);
		//gClosingState.SetNavigatingFlag(true);
		gTestingState.setRunMode(true);
		SetMissionFlag(false);
	}

}
void NavigatingState::setGoal(const VECTOR3& pos)
{
	mIsGoalPos = true;
	mGoalPos = pos;
	Debug::print(LOG_SUMMARY, "Set Goal ( %f %f )\r\n", mGoalPos.x, mGoalPos.y);
}

void NavigatingState::setStart()
{
	VECTOR3 current_pos;
	if (gGPSSensor.get(current_pos))
	{
		mStartPos = current_pos; 
		mIsStartPos = true;
		return;
	}
	mIsStartPos = false;
	return;
}

bool NavigatingState::getGoal(VECTOR3 & pos)
{
	if (!mIsGoalPos)return false;
	pos = mGoalPos;
	return true;
}

double NavigatingState::getDeltaAngle()
{
	return mDeltaAngle;
}

double NavigatingState::getTurnValue()
{
	return mTurnValue;
}

void NavigatingState::SetMissionFlag(bool flag)
{
	mMissionFlag = flag;
}
NavigatingState::NavigatingState() :mFarModePID(), enableNearNaviMode(false), mDeltaAngle(0.0)
{
	setName("navigating");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
	SetMissionFlag(false);
}
NavigatingState::~NavigatingState()
{
}
