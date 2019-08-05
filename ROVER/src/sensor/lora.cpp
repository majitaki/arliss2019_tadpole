#include "./lora.h"
#include "./lora_constant.h"
#include "../rover_util/utils.h"
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <termios.h>

Lora gLora;
bool Lora::onInit(const struct timespec& time)
{
	fd = serialOpen("/dev/ttySOFT0", 9600);
	if(fd < 0){
		Debug::print(LOG_SUMMARY, "lora serial device cannot open\r\n");
		return false;
	}
	struct termios options ;
	delay(100);
	serialPuts(fd,"hello world\r\n");
	mLastUpdateTime = time;
	return true;
}

void Lora::onClean()
{
	Debug::print(LOG_SUMMARY, "Lora is Finished\r\n");
	serialClose(fd);
}

bool Lora::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	switch (args.size())
	{
	case 1:
		return true;
	case 2:
		std::string str = args[1]; 
		send(str);
		return true;
	}
	Debug::print(LOG_PRINT, "Failed Command\r\n");
	return false;
}

void Lora::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) < LORA_UPDATE_INTERVAL_TIME)
	{
		return;
	}
}

void Lora::send(const std::string & str)
{
	//serialPuts(fd, str.c_str() + "\r\n");
	serialPuts(fd, "hello world\r\n");
}

Lora::Lora(): mLastUpdateTime()
{
	setName("lora");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_SENSOR);
}

Lora::~Lora()
{
}
