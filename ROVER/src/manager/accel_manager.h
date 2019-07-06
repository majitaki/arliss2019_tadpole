#pragma once
#include <pthread.h>
#include <list>
#include <ctime>
#include <iostream>
#include <math.h>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"

const static float GyroMeasError = M_PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
const static float GyroMeasDrift = M_PI * (0.0f / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
const static float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
const static float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

class AccelManager : public TaskBase
{
private:
	double mQuaternion[4];
	double mErrorOfIntegral[3];
	double mRoll, mPitch, mYaw;
	struct timespec mLastUpdateTime;
	double mOffsetMagnetX, mOffsetMagnetY, mOffsetMagnetZ;
	double mScaleMagnetX, mScaleMagnetY, mScaleMagnetZ;
	bool isSensorView;
	bool isTryNineaxis;
	std::list<VECTOR3> mAccelSamples;

protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual  void onUpdate(const timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);

public:
	//加速度
	VECTOR3 getVectorAccel() const;
	double getAx() const;
	double getAy() const;
	double getAz() const;
	//ジャイロ
	VECTOR3 getVectorGyro() const;
	double getGx() const;
	double getGy() const;
	double getGz() const;
	//地磁気
	void calibrationMagnet(int samples);
	VECTOR3 getVectorMagnet() const;
	double getMx() const;
	double getMy() const;
	double getMz() const;
	double getRawMagnetDirection() const;
	//速度
	VECTOR3 getVectorSpeed() const;
	double getSx() const;
	double getSy() const;
	double getSz() const;
	//ローリング
	VECTOR3 getVectorRolling() const;
	float getRoll() const;
	float getPitch() const;
	float getYaw() const;
	//print
	void printInfo();
	void printCalibInfo();
	//is turn
	bool isTurnSide();
	bool isTurnBack();

	//引数のベクトルを(-180〜+180)の範囲に修正
	void normalize(VECTOR3& pos);
	double normalize(double pos);
	//フィルタ
	void UpdateMadgwickQuaternion(double dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
	void UpdateMahonyQuaternion(double dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

	AccelManager();
	~AccelManager();
};

extern AccelManager gAccelManager;