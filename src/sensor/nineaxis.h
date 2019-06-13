#pragma once
#include <pthread.h>
#include <list>
#include <ctime>
#include <iostream>
#include <deque>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"
#include  "./nineaxis_address.h"

//
//const static int ACCEL_SAMPLES = 10;
//const static int GYRO_SAMPLES = 10;
//const static int MAGNET_SAMPLES = 10;
//const static double NINEAXIS_UPDATE_INTERVAL_TIME = 0.01;

class NineAxisSensor : public TaskBase
{
private:
	int mFileHandleNineaxis, mFileHandleCompass;
	VECTOR3 mAccel, mAccelAve;
	VECTOR3 mGyro, mGyroAve;
	VECTOR3 mMagnet, mMagnetAve;
	std::deque<VECTOR3> mAccelDeque;
	std::deque<VECTOR3> mGyroDeque;
	std::deque<VECTOR3> mMagnetDeque;
	struct timespec mLastSampleTime;
	float _gres, _ares, _mres;
	float _magXcoef, _magYcoef, _magZcoef;
	struct timespec mLastUpdateTime;
	void writeI2c(int fd, int register_addr, unsigned int data);
	void readI2c(int fd, int register_addr, int num, unsigned int * buffer);
	void setAccelSamples(int accel_samples);
	void setGyroSamples(int gyro_samples);
	void setMagnetSamples(int magnet_samples);
	bool searchDevice();
	void configMPU9250(unsigned int gfs = MPU9250_GFS_250, unsigned int afs = MPU9250_AFS_8G);
	void configAK8963(unsigned int mode = AK8963_MODE_C8HZ, unsigned int mfs = AK8963_BIT_16);
	bool checkDataReady();
	void readAccelXYZ(float *accel);
	void readGyroXYZ(float *gyro);
	void readMagnetXYZ(float *magnet);
	void readTemperature(float * temperature);
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	VECTOR3 getAccel() const;
	VECTOR3 getAccelAve() const;
	VECTOR3 getGyro() const;
	VECTOR3 getGyroAve() const;
	VECTOR3 getMagnet() const;
	VECTOR3 getMagnetAve() const;
	std::deque<VECTOR3> getAccelSamples();
	std::deque<VECTOR3> getGyroSamples();
	std::deque<VECTOR3> getMagnetSamples();
	bool isAlive();

	NineAxisSensor();
	~NineAxisSensor();
};

extern NineAxisSensor gNineAxisSensor;



