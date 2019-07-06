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
#include <ctime>
#include <iostream>
#include <algorithm>
#include <vector>
#include "../rover_util/utils.h"
#include "../sensor/nineaxis.h"
#include "accel_manager.h"
#include "accel_mangaer_constanth.h"

AccelManager gAccelManager;

bool AccelManager::onInit(const struct timespec& time)
{
	gNineAxisSensor.setRunMode(true);
	mLastUpdateTime = time;
	isTryNineaxis = false;
	mAccelSamples.clear();
	return true;
}
void AccelManager::onClean()
{
}
void AccelManager::onUpdate(const timespec & time)
{
	double dt = Time::dt(time, mLastUpdateTime);
	if (dt < ACCELMANAGER_UPDATE_INTERVAL_TIME)return;
	mLastUpdateTime = time;

	if (!gNineAxisSensor.isActive())
	{
		if (isTryNineaxis)
		{
			setRunMode(false);
			return;
		}
		gNineAxisSensor.setRunMode(true);
		isTryNineaxis = true;
	}

	UpdateMadgwickQuaternion(dt, getAx(), getAy(), getAz(), getGx()*M_PI / 180.0f, getGy()*M_PI / 180.0f, getGz()*M_PI / 180.0f, getMx(), getMy(), getMz());
	//UpdateMahonyQuaternion(dt, getAx(), getAy(), getAz(), getGx()*M_PI / 180.0f, getGy()*M_PI / 180.0f, getGz()*M_PI / 180.0f, getMx(), getMy(), getMz());

	if (!isSensorView) return;
	printInfo();
}
bool AccelManager::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	switch (args.size())
	{
	case 1:
		Debug::print(LOG_PRINT,
			"\r\n\
accel calibmagnet   : calibrate magnet. Do 8 circle!\r\n\
accel view   : sensor view \r\n\n\
");
		printInfo();
		printCalibInfo();
		return true;
	case 2:
		if (args[1].compare("calibmagnet") == 0)
		{
			calibrationMagnet(MAGNET_CALIBRATION_SAMPLES);
			Debug::print(LOG_PRINT, "Magnet OFFSET POS X:%f, Y:%f, Z:%f\r\n", mOffsetMagnetX, mOffsetMagnetY, mOffsetMagnetZ);
			Debug::print(LOG_PRINT, "Magnet OFFSET SCALE X:%f, Y:%f, Z:%f\r\n", mScaleMagnetX, mScaleMagnetY, mScaleMagnetZ);
		}
		else if (args[1].compare("view") == 0)
		{
			isSensorView = !isSensorView;
		}
		return true;
	default:
		return true;
	}
}

void AccelManager::printInfo()
{
	Debug::print(LOG_PRINT, "Accel   %3.3f %3.3f %3.3f\r\n", getAx(), getAy(), getAz());
	Debug::print(LOG_PRINT, "Gyro    %3.3f %3.3f %3.3f\r\n", getGx(), getGy(), getGz());
	Debug::print(LOG_PRINT, "Magnet  %3.3f %3.3f %3.3f\r\n", getMx(), getMy(), getMz());
	Debug::print(LOG_PRINT, "Magnet Angle %3.3f %3.3f %3.3f\r\n", getMx(), getMy(), getMz());
	Debug::print(LOG_PRINT, "Roll, Pitch, Yaw  %3.3f %3.3f %3.3f\r\n", getRoll(), getPitch(), getYaw());
	Debug::print(LOG_PRINT, "Azimuth(deg) %3.3f\r\n", getRawMagnetDirection());
	//Debug::print(LOG_PRINT, "Sx, Sy, Sz  %3.3f %3.3f %3.3f\r\n", getSx(), getSy(), getSz());
	Debug::print(LOG_PRINT, "Turn Side, Turn Back  %s %s \r\n", isTurnSide() ? "true" : "false", isTurnBack() ? "true" : "false");
}

void AccelManager::printCalibInfo()
{
	Debug::print(LOG_PRINT, "Calibration Magnet POS X:%3.3f, Y:%3.3f, Z:%3.3f\r\n", mOffsetMagnetX, mOffsetMagnetY, mOffsetMagnetZ);
	Debug::print(LOG_PRINT, "Calibration Magnet SCALE X:%3.3f, Y:%3.3f, Z:%3.3f\r\n", mScaleMagnetX, mScaleMagnetY, mScaleMagnetZ);
}

bool AccelManager::isTurnSide()
{
	float roll = abs(getRoll());
	return (roll > 90 - TURN_SIDE_THRESHOLD && roll < 90 + TURN_SIDE_THRESHOLD);
}

bool AccelManager::isTurnBack()
{
	float roll = abs(getRoll());

	return (roll > 180 - TURN_BACK_THRESHOLD && roll < 180 + TURN_BACK_THRESHOLD);
}

VECTOR3 AccelManager::getVectorAccel() const
{
	return VECTOR3();
}

double AccelManager::getAx() const
{
	return 	gNineAxisSensor.getAccel().x;
}

double AccelManager::getAy() const
{
	return 	gNineAxisSensor.getAccel().y;
}

double AccelManager::getAz() const
{
	return 	gNineAxisSensor.getAccel().z;
}

VECTOR3 AccelManager::getVectorGyro() const
{
	return VECTOR3();
}

double AccelManager::getGx() const
{
	return 	gNineAxisSensor.getGyro().x;
}

double AccelManager::getGy() const
{
	return 	gNineAxisSensor.getGyro().y;
}

double AccelManager::getGz() const
{
	return 	gNineAxisSensor.getGyro().z;
}

void AccelManager::calibrationMagnet(int calib_magnet_samples)
{
	std::vector<VECTOR3> magnet_samples;

	for (; magnet_samples.size() < calib_magnet_samples;)
	{
		magnet_samples.push_back(gNineAxisSensor.getMagnet());
		Debug::print(LOG_SUMMARY, "Calib Progres %d / %d\r\n", magnet_samples.size(), calib_magnet_samples);
		delay(10);
	}

	std::vector<double> magnet_vector_x;
	std::vector<double> magnet_vector_y;
	std::vector<double> magnet_vector_z;

	for (VECTOR3 sample : magnet_samples)
	{
		magnet_vector_x.push_back(sample.x);
		magnet_vector_y.push_back(sample.y);
		magnet_vector_z.push_back(sample.z);
	}

	auto max_x = max_element(magnet_vector_x.begin(), magnet_vector_x.end());
	auto max_y = max_element(magnet_vector_y.begin(), magnet_vector_y.end());
	auto max_z = max_element(magnet_vector_z.begin(), magnet_vector_z.end());

	auto min_x = min_element(magnet_vector_x.begin(), magnet_vector_x.end());
	auto min_y = min_element(magnet_vector_y.begin(), magnet_vector_y.end());
	auto min_z = min_element(magnet_vector_z.begin(), magnet_vector_z.end());

	mOffsetMagnetX = (*max_x + *min_x) / 2;
	mOffsetMagnetY = (*max_y + *min_y) / 2;
	mOffsetMagnetZ = (*max_z + *min_z) / 2;

	double avg_delta = (mOffsetMagnetX + mOffsetMagnetY + mOffsetMagnetZ) / 3;

	mScaleMagnetX = avg_delta / mOffsetMagnetX;
	mScaleMagnetY = avg_delta / mOffsetMagnetY;
	mScaleMagnetZ = avg_delta / mOffsetMagnetZ;

}

VECTOR3 AccelManager::getVectorMagnet() const
{
	return gNineAxisSensor.getMagnet();
}

double AccelManager::getMx() const
{
	return 	(gNineAxisSensor.getMagnet().x - mOffsetMagnetX) * mScaleMagnetX;
}

double AccelManager::getMy() const
{
	return 	(gNineAxisSensor.getMagnet().y - mOffsetMagnetY) * mScaleMagnetY;
}

double AccelManager::getMz() const
{
	return 	(gNineAxisSensor.getMagnet().z - mOffsetMagnetZ) * mScaleMagnetZ;
}

double AccelManager::getRawMagnetDirection() const
{
	double azimuth;
	double mag_x = getMx();
	double mag_y = getMy();
	double mag_z = getMz();
	azimuth = -(fmod(atan2(mag_y, mag_x) * (180.0 / M_PI) + 270.0 - 7, 360.0) - 360.0);
	return azimuth;
}

VECTOR3 AccelManager::getVectorSpeed() const
{

	return VECTOR3();

}

double AccelManager::getSx() const
{
	/*std::deque<VECTOR3> accel_samples = gNineAxisSensor.getAccelSamples();
	std::deque<VECTOR3>::iterator it = accel_samples.begin();
	double speed;
	double curr_accel = 0.0;
	double pre_accel = 0.0;
	while (it != accel_samples.end())
	{
		curr_accel = (*it).x;
		speed += (curr_accel + pre_accel) / 2 * NINEAXIS_UPDATE_INTERVAL_TIME;
		pre_accel = curr_accel;
		++it;
	}
	return speed;*/
	return 0.0;
}

double AccelManager::getSy() const
{
	/*std::deque<VECTOR3> accel_samples = gNineAxisSensor.getAccelSamples();
	std::deque<VECTOR3>::iterator it = accel_samples.begin();
	double speed;
	double curr_accel = 0.0;
	double pre_accel = 0.0;
	while (it != accel_samples.end())
	{
		curr_accel = (*it).y;
		speed += (curr_accel + pre_accel) / 2 * NINEAXIS_UPDATE_INTERVAL_TIME;
		pre_accel = curr_accel;
		++it;
	}
	return speed;*/
	return 0.0;
}

double AccelManager::getSz() const
{
	/*std::deque<VECTOR3> accel_samples = gNineAxisSensor.getAccelSamples();
	std::deque<VECTOR3>::iterator it = accel_samples.begin();
	double speed;
	double curr_accel = 0.0;
	double pre_accel = 0.0;
	while (it != accel_samples.end())
	{
		curr_accel = (*it).z;
		speed += (curr_accel + pre_accel) / 2 * NINEAXIS_UPDATE_INTERVAL_TIME;
		pre_accel = curr_accel;
		++it;
	}
	return speed;*/
	return 0.0;
}

VECTOR3 AccelManager::getVectorRolling() const
{
	return VECTOR3();
}

float AccelManager::getRoll() const
{
	double roll = atan2(2.0f * (mQuaternion[0] * mQuaternion[1] + mQuaternion[2] * mQuaternion[3]), mQuaternion[0] * mQuaternion[0] - mQuaternion[1] * mQuaternion[1] - mQuaternion[2] * mQuaternion[2] + mQuaternion[3] * mQuaternion[3]);

	roll *= 180.0f / M_PI;
	return roll;
}

float AccelManager::getPitch() const
{
	double pitch = -asin(2.0f * (mQuaternion[1] * mQuaternion[3] - mQuaternion[0] * mQuaternion[2]));
	pitch *= 180.0f / M_PI;
	return  pitch;
}

float AccelManager::getYaw() const
{
	double yaw = atan2(2.0f * (mQuaternion[1] * mQuaternion[2] + mQuaternion[0] * mQuaternion[3]), mQuaternion[0] * mQuaternion[0] + mQuaternion[1] * mQuaternion[1] - mQuaternion[2] * mQuaternion[2] - mQuaternion[3] * mQuaternion[3]);
	yaw *= 180.0f / M_PI;
	yaw -= -DECLINATION_FOR_YAW;

	return yaw;
}

double AccelManager::normalize(double pos)
{
	while (pos >= 180 || pos < -180)pos += (pos > 0) ? -360 : 360;
	return pos;
}

void AccelManager::UpdateMadgwickQuaternion(double dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{

	float q1 = mQuaternion[0], q2 = mQuaternion[1], q3 = mQuaternion[2], q4 = mQuaternion[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrtf(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f / norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * dt;
	q2 += qDot2 * dt;
	q3 += qDot3 * dt;
	q4 += qDot4 * dt;
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f / norm;
	mQuaternion[0] = q1 * norm;
	mQuaternion[1] = q2 * norm;
	mQuaternion[2] = q3 * norm;
	mQuaternion[3] = q4 * norm;

}

void AccelManager::UpdateMahonyQuaternion(double dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = mQuaternion[0], q2 = mQuaternion[1], q3 = mQuaternion[2], q4 = mQuaternion[3];   // short name local variable for readability
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrtf((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f)
	{
		mErrorOfIntegral[0] += ex;      // accumulate integral error
		mErrorOfIntegral[1] += ey;
		mErrorOfIntegral[2] += ez;
	}
	else
	{
		mErrorOfIntegral[0] = 0.0f;     // prevent integral wind up
		mErrorOfIntegral[1] = 0.0f;
		mErrorOfIntegral[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + Kp * ex + Ki * mErrorOfIntegral[0];
	gy = gy + Kp * ey + Ki * mErrorOfIntegral[1];
	gz = gz + Kp * ez + Ki * mErrorOfIntegral[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * dt);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * dt);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * dt);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * dt);

	// Normalise quaternion
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	mQuaternion[0] = q1 * norm;
	mQuaternion[1] = q2 * norm;
	mQuaternion[2] = q3 * norm;
	mQuaternion[3] = q4 * norm;
}

void AccelManager::normalize(VECTOR3& pos)
{
	pos.x = normalize(pos.x);
	pos.y = normalize(pos.y);
	pos.z = normalize(pos.z);
}

AccelManager::AccelManager() :mOffsetMagnetX(MAGNET_POS_OFFSET_X), mOffsetMagnetY(MAGNET_POS_OFFSET_Y), mOffsetMagnetZ(MAGNET_POS_OFFSET_Z), mScaleMagnetX(MAGNET_SCALE_OFFSET_X), mScaleMagnetY(MAGNET_SCALE_OFFSET_Y), mScaleMagnetZ(MAGNET_SCALE_OFFSET_Z)
{
	setName("accel");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_SENSOR);
	mQuaternion[0] = 1;
	mQuaternion[1] = 0;
	mQuaternion[2] = 0;
	mQuaternion[3] = 0;
	mErrorOfIntegral[0] = 0;
	mErrorOfIntegral[1] = 0;
	mErrorOfIntegral[2] = 0;
	isSensorView = false;
}
AccelManager::~AccelManager()
{
}