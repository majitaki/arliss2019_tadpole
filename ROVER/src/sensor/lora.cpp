#include "./lora.h"
#include "./lora_constant.h"
#include "../rover_util/utils.h"
#include "../sensor/gps.h"
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
	std::string str_insmod = "sudo insmod ../setting/soft_uart.ko"; 
	std::string str_tx = " gpio_tx=";
	std::string str_txpin = std::to_string(LORA_TX_PIN);
	std::string str_rx = " gpio_rx=";
	std::string str_rxpin = std::to_string(LORA_RX_PIN);
	std::string str_command = str_insmod + str_tx + str_txpin  + str_rx + str_rxpin;
	system(str_command.c_str());
    pinMode(LORA_SLEEP_PIN, OUTPUT);
	delay(10);

	fd = serialOpen("/dev/ttySOFT0", 9600);
	if(fd < 0){
		Debug::print(LOG_SUMMARY, "lora serial device cannot open\r\n");
		this->onClean();
		return false;
	}

	for(int i = 0; i < 5; i++){
		send("1");
		send("start");
	}
	mLastUpdateTime = time;
	return true;
}

void Lora::onClean()
{
	Debug::print(LOG_SUMMARY, "Lora is Finished\r\n");
	serialClose(fd);
	sleep(true);
	//system("sh ../setting/rmmod.sh");
	std::string str_rmmod = "sudo rmmod ../setting/soft_uart.ko"; 
	system(str_rmmod.c_str());
}

bool Lora::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	switch (args.size())
	{
	case 1:
		Debug::print(LOG_PRINT,
			"\r\n\
			lora send [data]: lora send data\r\n\
			lora gps        : lora send gps on/off \r\n\
		");
		return true;
	case 2:
		if (args[1].compare("gps") == 0)
		{
			enableGPSsend(!enableGPSsendFlag);
			return true;
		}
	case 3:
		if (args[1].compare("send") == 0)
		{
			std::string str = args[2]; 
			send(str);
			return true;
		}
	}
	Debug::print(LOG_PRINT, "Failed Command\r\n");
	return false;
}

void Lora::onUpdate(const struct timespec& time)
{
	double dt = Time::dt(time, mLastUpdateTime);
	if (dt < LORA_UPDATE_INTERVAL_TIME)return;
	mLastUpdateTime = time;

	if(!gGPSSensor.isActive() && enableGPSsendFlag){
		Debug::print(LOG_SUMMARY, "lora can't send gps because gps task is not working\r\n");
		return;
	} 

	if(enableGPSsendFlag){
		VECTOR3 currentPos;
		gGPSSensor.get(currentPos, false);
		std::string str_gps = "lati: " + std::to_string(currentPos.x) + " long: " + std::to_string(currentPos.y);
		send(str_gps);
	}

}

void Lora::send(const std::string & str)
{
	sleep(false);
	delay(100);
	std::string str_send = str + "\r\n";
	serialPuts(fd, str_send.c_str());
	//serialPuts(fd, str.c_str() + "\r\n");
	//serialPuts(fd, "hello world\r\n");
}

void Lora::sleep(bool enableSleepMode){
	if(enableSleepMode){
		digitalWrite(LORA_SLEEP_PIN, 0);
		delay(100);
		digitalWrite(LORA_SLEEP_PIN, 1);
	}else{
		digitalWrite(LORA_SLEEP_PIN, 1);
		delay(100);
		digitalWrite(LORA_SLEEP_PIN, 0);
	}
}

void Lora::enableGPSsend(bool flag){
	if(flag){
		if(!gGPSSensor.isActive()) gGPSSensor.setRunMode(true);
		enableGPSsendFlag = true;
	}else{
		enableGPSsendFlag = false;
	}
}

Lora::Lora(): mLastUpdateTime(), enableGPSsendFlag(false)
{
	setName("lora");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_SENSOR);
}

Lora::~Lora()
{
}
