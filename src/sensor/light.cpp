//ttb
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
#include "./light.h"
#include "../rover_util/utils.h"
#include "light_constant.h"


LightSensor gLightSensor;
bool LightSensor::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "light sensor is Ready\r\n");
	pinMode(mPin, INPUT);
	return true;
}
void LightSensor::onClean()
{
}
bool LightSensor::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	if (get())Debug::print(LOG_SUMMARY, "light is high\r\n");
	else Debug::print(LOG_SUMMARY, "light is low\r\n");
	return true;

}
bool LightSensor::get() const
{
	return digitalRead(mPin) == 0;
}
LightSensor::LightSensor() : mPin(PIN_LIGHT_SENSOR)
{
	setName("light");
	setPriority(TASK_PRIORITY_SENSOR, UINT_MAX);
}
LightSensor::~LightSensor()
{
}
