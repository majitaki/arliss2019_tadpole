#include <iostream>
#include <fstream>
#include <sstream>
#include<picojson.h>

#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <time.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <wiringSerial.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <libgpsmm.h>
#include "gps.h"
#include "../rover_util/utils.h"
#include "gps_constant.h"

#define _USE_MATH_DEFINE

GPSSensor gGPSSensor;

bool GPSSensor::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	mLastGetNewDataTime = time;
	mLastGetRemoveTime = time;
	mErrorFlag = true;

	if (gps_rec.stream(WATCH_ENABLE | WATCH_JSON) == NULL)
	{
		Debug::print(LOG_SUMMARY, "Failed to setup GPS Sensor\r\n");
		return false;
	}
	Debug::print(LOG_SUMMARY, "GPS Sensor is Ready! \n");

	mPos.x = mPos.y = mPos.z = 0;
	mIsNewData = false;
	mIsLogger = false;
	mRemoveErrorFlag = true;
	mUseMeanFlag = false;
	mClassMeanFlag = false;
	mSettingSampleNum = GPS_SAMPLES;
	mSettingStuckTh = GPS_STUCK_THRESHOLD;

	if(!readJSON()){
		mSettingSampleNum = GPS_SAMPLES;
		mSettingStuckTh = GPS_STUCK_THRESHOLD;
	}

	return true;
}

bool GPSSensor::readJSON()
{
	// JSONデータの読み込み。
    std::ifstream ifs("../setting/gps.json", std::ios::in);
    if (ifs.fail()) {
		Debug::print(LOG_SUMMARY, "Failed to read GPS JSON\r\n");
        return false;
    }
    const std::string json((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    ifs.close();

    // JSONデータを解析する。
    picojson::value v;
    const std::string err = picojson::parse(v, json);
    if (err.empty() == false) {
        std::cerr << err << std::endl;
        return false;
    }

	picojson::object& obj = v.get<picojson::object>();
	mSettingStuckTh = obj["gps_stuck_threshold"].get<double>();
	mSettingSampleNum  = (int)obj["gps_sample_num"].get<double>();
	return true;
}

void GPSSensor::onClean()
{
}
void GPSSensor::onUpdate(const struct timespec& time)
{
	double dt = Time::dt(time, mLastUpdateTime);
	if (dt < GPS_UPDATE_INTERVAL_TIME)return;

	mLastUpdateTime = time;
	mIsNewData = false;

	//remove old sample
	double dt_get_new = Time::dt(time, mLastGetNewDataTime);
	if (dt_get_new > GPS_REMOVE_UPDATE_INTERVAL_TIME && !mLastPos.empty())
	{
		Debug::print(LOG_SUMMARY, "remove old\r\n");
		mLastPos.pop_back();//remove most old
	}

	if ((newdata = gps_rec.read()) != NULL)
	{
		mPos.x = newdata->fix.latitude;
		mPos.y = newdata->fix.longitude;
		mPos.z = newdata->fix.altitude;
		mSatelites = newdata->satellites_visible;
		mGpsSpeed = newdata->fix.speed;
		mGpsCourse = newdata->fix.track;
		if (mSatelites > 0)	mGpsTime = newdata->fix.time;

		//samples input 
		if (finite(mPos.x) && finite(mPos.y) && finite(mPos.z))
		{
			mLastPos.push_front(mPos);
			mIsNewData = true;
			mLastGetNewDataTime = time;

			if (mLastPos.size() > mSettingSampleNum)
			{
				mLastPos.pop_back();
			}
		}
	}

	double dt_remove_error = Time::dt(time, mLastGetRemoveTime);
	if ((dt_remove_error > GPS_REMOVE_ERROR_INTERVAL_TIME && !mLastPos.empty()) || mErrorFlag)
	{
		//Debug::print(LOG_SUMMARY, "error check \r\n");
		//remove error
		//removeAveNewErrorSample();//remove new error
		removeErrorSample();//remove sample error
		mLastGetRemoveTime = time;
	}

	if (mIsLogger) showState();
	return;
}
bool GPSSensor::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;


	switch (args.size())
	{
	case 1:
		Debug::print(LOG_PRINT,
			"\r\n\
gps view   : gps view start/stop\r\n\
gps rem_error   : remove error true/false\r\n\n\
");
		showState();
		return true;
	case 2:
		if (args[1].compare("view") == 0)
		{
			mIsLogger = !mIsLogger;
		}
		else if (args[1].compare("rem_error") == 0)
		{
			mRemoveErrorFlag = !mRemoveErrorFlag;
		}
		return true;
	default:
		return false;
	}
}
bool GPSSensor::get(VECTOR3& pos, bool disableNewFlag)
{
	VECTOR3 new_pos = mLastPos.front();
	if (mSatelites >= 4 && !(new_pos.x == 0 && new_pos.y == 0 && new_pos.z == 0))//3D fix
	{
		if (!disableNewFlag)mIsNewData = false;
		pos = new_pos;
		return true;
	}
	return false;//Invalid Position
}

double GPSSensor::getPosx() const
{
	return mPos.x;
}
double GPSSensor::getPosy() const
{
	return mPos.y;
}
double GPSSensor::getPosz() const
{
	return mPos.z;
}


bool GPSSensor::isNewPos() const
{
	return mIsNewData;
}
int GPSSensor::getTime() const
{
	return mGpsTime;
}
float GPSSensor::getCourse() const
{
	return mGpsCourse;
	//return NineAxisSensor::normalize(mGpsCourse);
}
float GPSSensor::getSpeed() const
{
	return mGpsSpeed;
}
bool GPSSensor::removeErrorSample()
{
	VECTOR3 ave_pos;
	if (!mRemoveErrorFlag) return false;
	if (!getAvePos(ave_pos)) return false;
	if (mLastPos.size() <= mSettingSampleNum / 2) return false;

	//mLastPos.unique();

	auto it = mLastPos.rbegin();
	mErrorFlag = false;
	int count = 0;
	while (it != mLastPos.rend())
	{
		if (VECTOR3::calcDistanceXY(ave_pos, *it) > GPS_ERROR_AVE_IT_THRESHOLD)
		{
			Debug::print(LOG_SUMMARY, "remove error sample \r\n");
			mLastPos.erase(--it.base());
			mErrorFlag = true;
			//return true; //it is specification
		}

		if (count++ > 2) return true;
		it++;
	}
	return false;
}

bool GPSSensor::removeAveNewErrorSample()//if remove return true
{
	VECTOR3 ave_pos;

	if (!getAvePos(ave_pos)) return false;

	double distance = VECTOR3::calcDistanceXY(mLastPos.front(), ave_pos);
	if (distance > GPS_ERROR_AVE_NEW_THRESHOLD)
	{
		Debug::print(LOG_SUMMARY, "remove error new date \r\n");
		mLastPos.erase(mLastPos.end());
		return true;
	}
	return false;
}

void GPSSensor::clearSample() 
{
	mLastPos.clear();
	if (mLastPos.empty()) Debug::print(LOG_SUMMARY, "Cleared All Sample of GPS\r\n");
}

bool GPSSensor::getAvePos(VECTOR3& pos)
{
	if (mLastPos.size() <= GPS_MINIMUM_SAMPLES_FOR_AVERAGE) return false;

	std::list<VECTOR3>::iterator it = mLastPos.begin();
	VECTOR3 average;
	while (it != mLastPos.end())
	{
		average += *it;
		++it;
	}
	average /= mLastPos.size();
	pos = average;

	return true;
}
bool GPSSensor::getDirectionAngle(double & angle)
{
	if (mUseMeanFlag) {
		VECTOR3 currentPos = mLastPos.front();
		VECTOR3 avePos;
		getAvePos(avePos);
		angle = VECTOR3::calcAngleXY(avePos, currentPos);//���[�o�[�̕���
		//VECTOR3 averagePos;
		//std::list<VECTOR3>::const_iterator it = mLastPos.begin();
		//while (it != mLastPos.end())
		//{
		//averagePos += *it;
		//++it;
		//}
		//averagePos -= mLastPos.back();
		//averagePos /= mLastPos.size() - 1;
	}
	else if (mClassMeanFlag) {
		VECTOR3 frontAvePos;
		VECTOR3 backAvePos;
		std::list<VECTOR3>::const_iterator it = mLastPos.begin();
		int count = 0;
		while (it != mLastPos.end())
		{
			if (count < int(mLastPos.size()/2)) {
				frontAvePos += *it;
			}
			else {
				backAvePos += *it;
			}
			++it;
			count++;
		}
		frontAvePos /= int(mLastPos.size() / 2);
		backAvePos /= mLastPos.size() - int(mLastPos.size() / 2);
		angle = VECTOR3::calcAngleXY(backAvePos, frontAvePos);//���[�o�[�̕���
	}
	else {
		VECTOR3 currentPos = mLastPos.front();
		VECTOR3 prePos = mLastPos.back();
		angle = VECTOR3::calcAngleXY(prePos, currentPos);//���[�o�[�̕���
	}
	return true;
}
void GPSSensor::clearSamples()
{
	mLastPos.clear();
}
bool GPSSensor::isStuckGPS()
{
	VECTOR3 ave_pos;
	if (!getAvePos(ave_pos)) return false;
	if(mSatelites == 0) return false;

	double distance = VECTOR3::calcDistanceXY(mLastPos.front(), ave_pos);
	return distance < mSettingStuckTh ? true : false;
}


bool GPSSensor::isStuckGPS(double &distance)
{
	VECTOR3 ave_pos;
	if (!getAvePos(ave_pos)) return false;
	if(mSatelites == 0) return false;

	distance = VECTOR3::calcDistanceXY(mLastPos.front(), ave_pos);
	Debug::print(LOG_SUMMARY, "isStuck: %f \r\n",distance);
	return distance < mSettingStuckTh ? true : false;
}

void GPSSensor::showState()
{
	if (mSatelites < 4)
	{
		Debug::print(LOG_SUMMARY, "Unknown Position\r\nSatelites: %d\r\n", mSatelites);
	}

	VECTOR3 ave_pos;
	VECTOR3 new_pos;
	VECTOR3 old_pos;

	getAvePos(ave_pos);
	new_pos.x = mLastPos.front().x;
	new_pos.y = mLastPos.front().y;
	new_pos.z = mLastPos.front().z;

	old_pos.x = mLastPos.back().x;
	old_pos.y = mLastPos.back().y;
	old_pos.z = mLastPos.back().z;

	double distance;
	double angle;
	if (!getDirectionAngle(angle)) angle = -1;

	//angle = VECTOR3::calcAngleXY(a, b);

	Debug::print(LOG_SUMMARY, "Satelites: %d \r\n", mSatelites);
	Debug::print(LOG_SUMMARY, "New Position: %f %f %f \r\n", new_pos.x, new_pos.y, new_pos.z);
	Debug::print(LOG_SUMMARY, "Ave Position: %f %f %f \r\n", ave_pos.x, ave_pos.y, ave_pos.z);
	Debug::print(LOG_SUMMARY, "Distance between New and Ave: %f \r\n", VECTOR3::calcDistanceXY(new_pos, ave_pos));
	Debug::print(LOG_SUMMARY, "Distance between New and Old: %f \r\n", VECTOR3::calcDistanceXY(new_pos, old_pos));
	Debug::print(LOG_SUMMARY, "Samples: %d / %d \r\n", mLastPos.size(), mSettingSampleNum);
	Debug::print(LOG_SUMMARY, "Time: %d \r\n", mGpsTime);
	Debug::print(LOG_SUMMARY, "Course: %f \r\n", mGpsCourse);
	Debug::print(LOG_SUMMARY, "Speed: %f \r\n", mGpsSpeed);
	Debug::print(LOG_SUMMARY, "Angle: %f \r\n", angle);
	Debug::print(LOG_SUMMARY, "Stuck: %s \r\n", isStuckGPS(distance) ? "true" : "false");
	Debug::print(LOG_SUMMARY, "Stuck distance: %f threshold: %f\r\n", distance, mSettingStuckTh);
	Debug::print(LOG_SUMMARY, "Remove Error function: %s \r\n", mRemoveErrorFlag ? "true" : "false");
}

GPSSensor::GPSSensor() : mFileHandle(-1), mPos(), mSatelites(0), mIsNewData(false), gps_rec("localhost", DEFAULT_GPSD_PORT)
{
	setName("gps");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_SENSOR);
}
GPSSensor::~GPSSensor()
{
}
