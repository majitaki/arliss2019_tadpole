#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <time.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
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
#include <deque>
#include "./nineaxis.h"
#include "../rover_util/utils.h"
#include  "./nineaxis_address.h"
#include  "./nineaxis_constant.h"


using namespace std;

NineAxisSensor gNineAxisSensor;

bool NineAxisSensor::onInit(const struct timespec& time)
{
	if (!isAlive())
	{
		Debug::print(LOG_SUMMARY, "Failed to Begin NineAxis Sensor\r\n");
		setRunMode(false);
		return false;
	}

	Debug::print(LOG_SUMMARY, "NineAxis Sensor is Ready!\r\n");

	mLastUpdateTime = time;
	return true;
}

void NineAxisSensor::onClean()
{
	wiringPiI2CWriteReg8(mFileHandleCompass, 0x20, 0x00);
	wiringPiI2CWriteReg8(mFileHandleNineaxis, 0x16, 0x00);

	close(mFileHandleCompass);
	close(mFileHandleNineaxis);
}

void NineAxisSensor::setAccelSamples(int accel_samples = ACCEL_SAMPLES)
{
	float accel[3];
	readAccelXYZ(accel);
	VECTOR3 accel_vector_element;
	accel_vector_element.x = accel[0];
	accel_vector_element.y = accel[1];
	accel_vector_element.z = accel[2];

	if (mAccelDeque.size() < accel_samples)
	{
		//cout << "size is low " << mAccelDeque.size() << endl;
		mAccelDeque.push_front(accel_vector_element);
	}
	else
	{
		//cout << "size is enough" << endl;
		mAccelDeque.pop_back();
		mAccelDeque.push_front(accel_vector_element);
	}

	double sum[3];
	for (auto i = mAccelDeque.begin(); i != mAccelDeque.end(); i++)
	{
		sum[0] += (*i).x;
		sum[1] += (*i).y;
		sum[2] += (*i).z;
	}

	mAccel = mAccelDeque.front();
	mAccelAve.x = sum[0] / mAccelDeque.size();
	mAccelAve.y = sum[1] / mAccelDeque.size();
	mAccelAve.z = sum[2] / mAccelDeque.size();

	return;
}

void NineAxisSensor::setGyroSamples(int gyro_samples = GYRO_SAMPLES)
{
	float gyro[3];
	readGyroXYZ(gyro);
	VECTOR3 gyro_vector_element;
	gyro_vector_element.x = gyro[0];
	gyro_vector_element.y = gyro[1];
	gyro_vector_element.z = gyro[2];

	if (mGyroDeque.size() < gyro_samples)
	{
		//cout << "size is low " << mAccelDeque.size() << endl;
		mGyroDeque.push_front(gyro_vector_element);
	}
	else
	{
		//cout << "size is enough" << endl;
		mGyroDeque.pop_back();
		mGyroDeque.push_front(gyro_vector_element);
	}

	double sum[3];
	for (auto i = mGyroDeque.begin(); i != mGyroDeque.end(); i++)
	{
		sum[0] += (*i).x;
		sum[1] += (*i).y;
		sum[2] += (*i).z;
	}

	mGyro = mGyroDeque.front();
	mGyroAve.x = sum[0] / mGyroDeque.size();
	mGyroAve.y = sum[1] / mGyroDeque.size();
	mGyroAve.z = sum[2] / mGyroDeque.size();

	return;
}

void NineAxisSensor::setMagnetSamples(int magnet_samples = MAGNET_SAMPLES)
{
	float magnet[3];
	readMagnetXYZ(magnet);
	VECTOR3 magnet_vector_element;
	magnet_vector_element.x = magnet[0];
	magnet_vector_element.y = magnet[1];
	magnet_vector_element.z = magnet[2];

	if (mMagnetDeque.size() <= magnet_samples)
	{
		//cout << "size is low " << mAccelDeque.size() << endl;
		mMagnetDeque.push_front(magnet_vector_element);
	}
	else
	{
		//cout << "size is enough" << endl;
		mMagnetDeque.pop_back();
		mMagnetDeque.push_front(magnet_vector_element);
	}

	double sum[3];
	for (auto i = mMagnetDeque.begin(); i != mMagnetDeque.end(); i++)
	{
		sum[0] += (*i).x;
		sum[1] += (*i).y;
		sum[2] += (*i).z;
	}

	mMagnet = mMagnetDeque.front();
	if (mMagnet.x == mGyro.x && mMagnet.y == mGyro.y && mMagnet.z == mGyro.z)
	{
		mMagnetDeque.pop_front();
		mMagnet = mMagnetDeque.front();
	}
	mMagnetAve.x = sum[0] / mMagnetDeque.size();
	mMagnetAve.y = sum[1] / mMagnetDeque.size();
	mMagnetAve.z = sum[2] / mMagnetDeque.size();

	return;
}

void NineAxisSensor::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) < NINEAXIS_UPDATE_INTERVAL_TIME)
	{
		return;
	}
	mLastUpdateTime = time;

	setAccelSamples();
	setGyroSamples();
	setMagnetSamples();
}

bool NineAxisSensor::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	switch (args.size())
	{
	case 1:
		Debug::print(LOG_PRINT, "Accel   %f %f %f\r\n", getAccel().x, getAccel().y, getAccel().z);
		Debug::print(LOG_PRINT, "Gyro    %f %f %f\r\n", getGyro().x, getGyro().y, getGyro().z);
		Debug::print(LOG_PRINT, "Magnet  %f %f %f\r\n", getMagnet().x, getMagnet().y, getMagnet().z);
		return true;
	default:
		return true;
	}
}
VECTOR3 NineAxisSensor::getAccel() const
{
	return mAccel;
}
VECTOR3 NineAxisSensor::getAccelAve() const
{
	return  mAccelAve;
}
VECTOR3 NineAxisSensor::getGyro() const
{
	return mGyro;
}
VECTOR3 NineAxisSensor::getGyroAve() const
{
	return mGyroAve;
}
VECTOR3 NineAxisSensor::getMagnet() const
{
	return mMagnet;
}
VECTOR3 NineAxisSensor::getMagnetAve() const
{
	return mMagnetAve;
}
std::deque<VECTOR3> NineAxisSensor::getAccelSamples()
{
	setAccelSamples();
	return mAccelDeque;
}
std::deque<VECTOR3> NineAxisSensor::getGyroSamples()
{
	setGyroSamples();
	return mGyroDeque;
}
std::deque<VECTOR3> NineAxisSensor::getMagnetSamples()
{
	setMagnetSamples();
	return mMagnetDeque;
}
void NineAxisSensor::writeI2c(int fd, int register_addr, unsigned int data)
{
	wiringPiI2CWriteReg8(fd, register_addr, data);
	delay(100);
}
void NineAxisSensor::readI2c(int fd, int register_addr, int num, unsigned int * buffer)
{
	if (num == 1)
	{
		*buffer = wiringPiI2CReadReg8(fd, register_addr);
	}
	else
	{
		for (int i = 0; i < num; i++)
		{
			buffer[i] = wiringPiI2CReadReg8(fd, register_addr + i);
		}
		return;
	}
}
bool NineAxisSensor::isAlive()
{
	if ((mFileHandleNineaxis = wiringPiI2CSetup(MPU9250_SLAVE_ADDRESS)) == -1)
	{
		Debug::print(LOG_SUMMARY, "Failed to Setup NineAxis Sensor\r\n");
		return false;
	}

	if ((mFileHandleCompass = wiringPiI2CSetup(AK8963_SLAVE_ADDRESS)) == -1)
	{
		Debug::print(LOG_SUMMARY, "Failed to Setup Compass Sensor\r\n");
		return false;
	}

	if (!searchDevice())
	{
		Debug::print(LOG_SUMMARY, "Failed to Search Device\r\n");
		return false;
	}

	configMPU9250();
	configAK8963();
	Debug::print(LOG_PRINT, "\nChecking Nineaxis sensor\r\n");
	Debug::print(LOG_PRINT, "Accel   %f %f %f\r\n", getAccel().x, getAccel().y, getAccel().z);
	Debug::print(LOG_PRINT, "Gyro    %f %f %f\r\n", getGyro().x, getGyro().y, getGyro().z);
	Debug::print(LOG_PRINT, "Magnet  %f %f %f\r\n", getMagnet().x, getMagnet().y, getMagnet().z);
	return true;
}
bool NineAxisSensor::searchDevice()
{
	unsigned int whoami;
	readI2c(mFileHandleNineaxis, MPU9250_WHO_AM_I, 1, &whoami);
	return whoami == 0x73 ? true : false;
}
void NineAxisSensor::configMPU9250(unsigned int gfs, unsigned int afs)
{
	switch (gfs)
	{
	case MPU9250_GFS_250:
		_gres = 250.0 / 32768.0;
		break;
	case MPU9250_GFS_500:
		_gres = 500.0 / 32768.0;
		break;
	case MPU9250_GFS_1000:
		_gres = 1000.0 / 32768.0;
		break;
	case MPU9250_GFS_2000:
		_gres = 2000.0 / 32768.0;
		break;
	}
	switch (afs)
	{
	case MPU9250_AFS_2G:
		_ares = 2.0 / 32768.0;
		break;
	case MPU9250_AFS_4G:
		_ares = 4.0 / 32768.0;
		break;
	case MPU9250_AFS_8G:
		_ares = 8.0 / 32768.0;
		break;
	case MPU9250_AFS_16G:
		_ares = 16.0 / 32768.0;
		break;
	}
	// sleep off
	writeI2c(mFileHandleNineaxis, MPU9250_PWR_MGMT_1, 0x00);
	delay(100);
	// auto select clock source
	writeI2c(mFileHandleNineaxis, MPU9250_PWR_MGMT_1, 0x01);
	delay(200);
	// DLPF_CFG
	writeI2c(mFileHandleNineaxis, MPU9250_CONFIG, 0x03);
	// sample rate divider
	writeI2c(mFileHandleNineaxis, MPU9250_SMPLRT_DIV, 0x04);
	// gyro full scale select
	writeI2c(mFileHandleNineaxis, MPU9250_GYRO_CONFIG, gfs << 3);
	// accel full scale select
	writeI2c(mFileHandleNineaxis, MPU9250_ACCEL_CONFIG, afs << 3);
	// A_DLPFCFG
	writeI2c(mFileHandleNineaxis, MPU9250_ACCEL_CONFIG_2, 0x03);
	// BYPASS_EN
	writeI2c(mFileHandleNineaxis, MPU9250_INT_PIN_CFG, 0x02);
	delay(100);
}
void NineAxisSensor::configAK8963(unsigned int mode, unsigned int mfs)
{
	unsigned int data[3];

	switch (mfs)
	{
	case AK8963_BIT_14:
		_mres = 4912.0 / 8190.0;
		break;
	case AK8963_BIT_16:
		_mres = 4912.0 / 32760.0;
		break;
	}

	// set software reset
	//   writeI2c(AK8963_SLAVE_ADDRESS, AK8963_CNTL2, 0x01);
	//   delay(100);
	// set power down mode
	writeI2c(mFileHandleCompass, AK8963_CNTL1, 0x00);
	delay(10);
	// set read FuseROM mode
	writeI2c(mFileHandleCompass, AK8963_CNTL1, 0x0F);
	// read coef data
	readI2c(mFileHandleCompass, AK8963_ASAX, 3, data);
	delay(10);
	_magXcoef = (float)(data[0] - 128) / 256.0 + 1.0;
	_magYcoef = (float)(data[1] - 128) / 256.0 + 1.0;
	_magZcoef = (float)(data[2] - 128) / 256.0 + 1.0;
	// set power down mode
	writeI2c(mFileHandleCompass, AK8963_CNTL1, 0x00);
	delay(10);
	// set scale&continous mode
	writeI2c(mFileHandleCompass, AK8963_CNTL1, (mfs << 4 | mode));
	delay(10);
}
void NineAxisSensor::readAccelXYZ(float *accel)
{
	unsigned int data[6];
	short axc, ayc, azc;
	readI2c(mFileHandleNineaxis, MPU9250_ACCEL_XOUT_H, 6, data);
	axc = (data[0] << 8) | data[1];
	ayc = (data[2] << 8) | data[3];
	azc = (data[4] << 8) | data[5];
	accel[0] = axc * _ares;
	accel[1] = ayc * _ares;
	accel[2] = azc * _ares;
}
void NineAxisSensor::readGyroXYZ(float *gyro)
{
	unsigned int data[6];
	short gxc, gyc, gzc;

	readI2c(mFileHandleNineaxis, MPU9250_GYRO_XOUT_H, 6, data);
	gxc = (data[0] << 8) | data[1];
	gyc = (data[2] << 8) | data[3];
	gzc = (data[4] << 8) | data[5];
	gyro[0] = gxc * _gres;
	gyro[1] = gyc * _gres;
	gyro[2] = gzc * _gres;
}
void NineAxisSensor::readMagnetXYZ(float *magnet)
{
	unsigned int data[7];
	unsigned int drdy;
	short mxc, myc, mzc;
	readI2c(mFileHandleCompass, AK8963_ST1, 1, &drdy);
	if (drdy & 0x01)
	{ // check data ready
		readI2c(mFileHandleCompass, AK8963_HXL, 7, data);
		if (!(data[6] & 0x08))
		{ // check overflow
			mxc = (data[1] << 8) | data[0];
			myc = (data[3] << 8) | data[2];
			mzc = (data[5] << 8) | data[4];
			magnet[0] = mxc * _mres * _magXcoef;
			magnet[1] = myc * _mres * _magYcoef;
			magnet[2] = mzc * _mres * _magZcoef;
		}
	}
}
void NineAxisSensor::readTemperature(float * temperature)
{
	unsigned int data[2];
	short tmc;
	readI2c(mFileHandleNineaxis, MPU9250_TEMP_OUT_H, 2, data);
	tmc = (data[0] << 8) | data[1];
	*temperature = tmc / 333.87 + 21.0;
}

NineAxisSensor::NineAxisSensor() : mFileHandleNineaxis(-1), mFileHandleCompass(-1), mAccel(), mAccelAve(), mMagnet(), mMagnetAve(), mGyro(), mGyroAve(), mLastUpdateTime()
{
	setName("nineaxis");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_SENSOR);
}
NineAxisSensor::~NineAxisSensor()
{
}
