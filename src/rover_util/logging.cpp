﻿#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <stdarg.h>
#include <sys/stat.h>
#include <time.h>
#include <RTIMULib.h>

#include "logging.h"
#include "../rover_util/utils.h"
#include "../rover_util/serial_command.h"
#include "../sensor/gps.h"
#include "../sensor/light.h"
#include "../sensor/nineaxis.h"
#include "../sensor/pressure.h"
#include "../actuator/motor.h"
//#include "../manager/accel_manager.h"
#include "../actuator/servo.h"

SensorLogging gSensorLoggingState;
MovementLogging gMovementLoggingState;
UnitedLogging gUnitedLoggingState;



bool UnitedLogging::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Sensor Log: Enabled\r\n");

	gGPSSensor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gNineAxisSensor.setRunMode(true);
	mLastUpdateTime = time;

	write(mFilenameUnitedLog, "time, event, latitude,longitude,height,ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temperature,altitude,humidity\r\n");
	return true;
}
void UnitedLogging::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) < UNITEDLOGGING_UPDATE_INTERVAL_TIME)return;

	mLastUpdateTime = time;
	std::string str_timestamp = Time::getTimeStamp(true);
	auto char_timestamp = str_timestamp.c_str();

	VECTOR3 vec;
	gGPSSensor.get(vec);
	double nodate = 0;

	//gps
	double lati;
	double longi;
	double height;
	//accel
	double ax;
	double ay;
	double az;
	//gyro
	double gx;
	double gy;
	double gz;
	//magnet
	double mx;
	double my;
	double mz;
	//pressure
	double pressure;
	double temp;
	double alt;
	double humidity;

	if (gGPSSensor.isActive()) {
		lati = vec.x;
		longi = vec.y;
		height = vec.z;
	}
	else {
		lati = longi = height = nodate;
	}

	if (gPressureSensor.isActive()) {
		pressure = gPressureSensor.getPressure();
		temp = gPressureSensor.getTemperature();
		alt = gPressureSensor.getAltitude();
		humidity = gPressureSensor.getHumidity();
	}
	else {
		pressure = temp = alt = humidity = nodate;
	}


	if (gNineAxisSensor.isActive()) {
		ax = gNineAxisSensor.getAccel().x();
		ay = gNineAxisSensor.getAccel().y();
		az = gNineAxisSensor.getAccel().z();

		mx = gNineAxisSensor.getMagnet().x();
		my = gNineAxisSensor.getMagnet().y();
		mz = gNineAxisSensor.getMagnet().z();

		gx = gNineAxisSensor.getGyro().x();
		gy = gNineAxisSensor.getGyro().y();
		gz = gNineAxisSensor.getGyro().z();
	}
	else {
		ax = ay = az = mx = my = mz = gx = gy = gz = nodate;
	}

	write(mFilenameUnitedLog, "%s,%s,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f \r\n", char_timestamp, mEventMessage.c_str(), lati, longi, height, ax, ay, az, gx, gy, gz, mx, my, mz, pressure, temp, alt, humidity);
	mEventMessage = "";
}

void UnitedLogging::write(const std::string& filename, const char* fmt, ...)
{
	std::ofstream of(filename.c_str(), std::ios::out | std::ios::app);

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);

	of << buf;
}
void UnitedLogging::writeEventMessage(std::string str)
{
}
UnitedLogging::UnitedLogging() : mLastUpdateTime(), mEventMessage("")
{
	setName("united_logging");
	setPriority(UINT_MAX, TASK_INTERVAL_SEQUENCE);

	//const int dir_err = mkdir(LOG_FOLDER, S_IRWXO | S_IROTH | S_IWOTH);

	std::string str_timestamp = Time::getTimeStamp();
	std::string str_log_dir = std::string(LOG_FOLDER);

	Filename(str_log_dir + "/" + str_timestamp + "/" + str_timestamp + "_" + "log_united", ".csv").get(mFilenameUnitedLog);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenameUnitedLog.c_str());
}
UnitedLogging::~UnitedLogging()
{
}



bool SensorLogging::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Sensor Log: Enabled\r\n");

	gGPSSensor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gNineAxisSensor.setRunMode(true);
	mLastUpdateTime = time;

	write(mFilenameGPS, "time,latitude,longitude,height\r\n");
	write(mFilenameAccel, "time,ax,ay,az\r\n");
	write(mFilenameGyro, "time,gx,gy,gz\r\n");
	write(mFilenameMagnet, "time,mx,my,mz\r\n");
	write(mFilenamePressure, "time,pressure,temperature,altitude,humidity\r\n");

	return true;
}
void SensorLogging::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) >= SENSORLOGGING_UPDATE_INTERVAL_TIME)
	{
		mLastUpdateTime = time;
		std::string str_timestamp = Time::getTimeStamp(true);
		auto char_timestamp = str_timestamp.c_str();

		VECTOR3 vec;
		gGPSSensor.get(vec);
		if (gGPSSensor.isActive())write(mFilenameGPS, "%s,%f,%f,%f\r\n", char_timestamp, vec.x, vec.y, vec.z);
		else write(mFilenameGPS, "unavailable\r\n");

		if (gPressureSensor.isActive())write(mFilenamePressure, "%s,%f,%f,%f,%f\r\n", char_timestamp, gPressureSensor.getPressure(), gPressureSensor.getTemperature(), gPressureSensor.getAltitude(), gPressureSensor.getHumidity());
		else write(mFilenamePressure, "unavailable\r\n");

		if (gNineAxisSensor.isActive())write(mFilenameAccel, "%s,%f,%f,%f\r\n", char_timestamp, gNineAxisSensor.getAccel().x(), gNineAxisSensor.getAccel().y(), gNineAxisSensor.getAccel().z());
		else write(mFilenameAccel, "unavailable\r\n");

		if (gNineAxisSensor.isActive())write(mFilenameGyro, "%s,%f,%f,%f\r\n", char_timestamp, gNineAxisSensor.getGyro().x(), gNineAxisSensor.getGyro().y(), gNineAxisSensor.getGyro().z());
		else write(mFilenameGyro, "unavailable\r\n");

		if (gNineAxisSensor.isActive())write(mFilenameMagnet, "%s,%f,%f,%f\r\n", char_timestamp, gNineAxisSensor.getMagnet().x(), gNineAxisSensor.getMagnet().y(), gNineAxisSensor.getMagnet().z());
		else write(mFilenameMagnet, "unavailable\r\n");
	}
}

void SensorLogging::write(const std::string& filename, const char* fmt, ...)
{
	std::ofstream of(filename.c_str(), std::ios::out | std::ios::app);

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);

	of << buf;
}
SensorLogging::SensorLogging() : mLastUpdateTime()
{
	setName("sensor_logging");
	setPriority(UINT_MAX, TASK_INTERVAL_SEQUENCE);

	//const int dir_err = mkdir(LOG_FOLDER, S_IRWXO | S_IROTH | S_IWOTH);

	std::string str_timestamp = Time::getTimeStamp();
	std::string str_log_dir = std::string(LOG_FOLDER);

	Filename(str_log_dir + "/" + str_timestamp + "/" + str_timestamp + "_" + "log_gps", ".csv").get(mFilenameGPS);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenameGPS.c_str());
	Filename(str_log_dir + "/" + str_timestamp + "/" + str_timestamp + "_" + "log_pressure", ".csv").get(mFilenamePressure);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenamePressure.c_str());
	Filename(str_log_dir + "/" + str_timestamp + "/" + str_timestamp + "_" + "log_accel", ".csv").get(mFilenameAccel);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenameAccel.c_str());
	Filename(str_log_dir + "/" + str_timestamp + "/" + str_timestamp + "_" + "log_magnet", ".csv").get(mFilenameMagnet);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenameMagnet.c_str());
	Filename(str_log_dir + "/" + str_timestamp + "/" + str_timestamp + "_" + "log_gyro", ".csv").get(mFilenameGyro);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenameGyro.c_str());

}
SensorLogging::~SensorLogging()
{
}


//////////////////////////////////////////////
// MovementLogging
//////////////////////////////////////////////

bool MovementLogging::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Movement Log: Enabled\r\n");

	gGPSSensor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	//gNineAxisSensor.setRunMode(true);
	gMotorDrive.setRunMode(true);
	mLastUpdateTime = time;

	write(mFilenameActuator, "time,motor, para, direct\r\n");

	return true;
}
void MovementLogging::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) < MOVEMENTLOGGING_UPDATE_INTERVAL_TIME)return;
	mLastUpdateTime = time;

	std::string str_timestamp = Time::getTimeStamp(true);
	auto char_timestamp = str_timestamp.c_str();

	mPrevPowerR = gMotorDrive.getPowerR();
	//mPrevPowerL = gMotorDrive.getPowerL();
	//mPrevParaServo = gServo.mParaServoPulseWidth;
	//mPrevDirectServo = gServo.mDirectServoPulseWidth;

	if (gMotorDrive.isActive() && gServo.isActive())write(mFilenameActuator, "%s,%d,%d,%d\r\n", char_timestamp, mPrevPowerR, mPrevParaServo, mPrevDirectServo);
	else write(mFilenameActuator, "unavailable\r\n");

}
bool MovementLogging::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	if (!gMovementLoggingState.isActive())
	{
		Debug::print(LOG_PRINT, "Movement Logging is not active\r\n");
		return true;
	}

	switch (args.size())
	{
	case 1:
		Debug::print(LOG_PRINT, "movementlogging stop            : stop MovementLogging\r\n\
														movementlogging buzzer          : switch buzzer\r\n\
																					movementlogging print           : switch pulse print\r\n\
																												movementlogging comment [string]: comment into log\r\n");
		return true;
	case 2:
		if (args[1].compare("stop") == 0)
		{
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			gMotorDrive.drive(0);
			gMovementLoggingState.setRunMode(false);
			return true;
		}
		else if (args[1].compare("print") == 0)
		{
			mPrintFlag = !mPrintFlag;
			if (mPrintFlag)
			{
				Debug::print(LOG_PRINT, "Command Executed! Print(ON)\r\n");
			}
			else
			{
				Debug::print(LOG_PRINT, "Command Executed! Print(OFF)\r\n");
			}
			return true;
		}
	case 3:
		if (args[1].compare("comment") == 0)
		{
			std::string str = args[2];
			Debug::print(LOG_PRINT, "Command Executed! comment: %s\r\n", str.c_str());
			write(mFilenameActuator, "comment: %s\r\n", str.c_str());
			return true;
		}
		return true;
	default:
		return true;
	}
}
void MovementLogging::write(const std::string& filename, const char* fmt, ...)
{
	std::ofstream of(filename.c_str(), std::ios::out | std::ios::app);

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);

	of << buf;
	//Debug::print(LOG_SUMMARY, "%s\r\n",buf);
}
MovementLogging::MovementLogging() : mLastUpdateTime(), mPrevPowerL(0), mPrevPowerR(0), mPrevParaServo(0), mPrevDirectServo(0), mPrevDeltaPulseL(0), mPrevDeltaPulseR(0), mPrintFlag(false)
{
	setName("actuator_logging");
	setPriority(UINT_MAX, TASK_INTERVAL_SEQUENCE);

	std::string str_timestamp = Time::getTimeStamp();
	std::string str_log_dir = std::string(LOG_FOLDER);

	Filename(str_log_dir + "/" + str_timestamp + "/" + str_timestamp + "_" + "log_motor_servo", ".txt").get(mFilenameActuator);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenameActuator.c_str());
}
MovementLogging::~MovementLogging()
{
}