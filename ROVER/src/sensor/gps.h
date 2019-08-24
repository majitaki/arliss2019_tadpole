#pragma once
#include "../rover_util/task.h"
#include "../rover_util/utils.h"
#include <pthread.h>
#include <list>
#include <libgpsmm.h>

class GPSSensor : public TaskBase
{
private:
	struct timespec mLastUpdateTime;
	struct timespec mLastGetNewDataTime;
	struct timespec mLastGetRemoveTime;
	int mFileHandle;//winringPi i2
	VECTOR3 mPos;
	VECTOR3 mAveragePos;
	int mSatelites;
	int mGpsTime;
	float mGpsSpeed;
	float mGpsCourse;
	bool mIsNewData;
	bool mIsLogger;
	bool mErrorFlag;
	bool mUseMeanFlag;
	bool mClassMeanFlag;
	gpsmm gps_rec;
	struct gps_data_t *newdata;
	std::list<VECTOR3> mLastPos;
	void showState();
	bool readJSON();
	bool mRemoveErrorFlag;
	int mSettingSampleNum;
	double mSettingStuckTh;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);

public:
	bool get(VECTOR3& pos, bool disableNewFlag = false);
	bool isNewPos() const;
	int getTime() const;
	double getPosx() const;
	double getPosy() const;
	double getPosz() const;
	float getCourse() const;
	float getSpeed() const;
	void clearSample();
	bool removeErrorSample();
	bool removeAveNewErrorSample();
	bool getAvePos(VECTOR3& pos);
	bool getDirectionAngle(double & angle);
	void clearSamples();
	bool isStuckGPS();
	bool isStuckGPS(double& distance);

	GPSSensor();
	~GPSSensor();
};

extern GPSSensor gGPSSensor;