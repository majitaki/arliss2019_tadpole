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
	//system("sh ../setting/insmod.sh");
	std::string str_insmod = "sudo insmod ../setting/soft_uart.ko"; 
	std::string str_tx = " gpio_tx=";
	std::string str_txpin = std::to_string(LORA_TX_PIN);
	std::string str_rx = " gpio_rx=";
	std::string str_rxpin = std::to_string(LORA_RX_PIN);
	std::string str_command = str_insmod + str_tx + str_txpin  + str_rx + str_rxpin;
	system(str_command.c_str());
	//system(std::string("sudo ../setting/insmod soft_uart.ko") + std::string(" gpio_tx=") + std::to_string(LORA_TX_PIN) + std::string(" gpio_rx=") + std::to_string(LORA_RX_PIN));
	//system("");
    pinMode(LORA_SLEEP_PIN, OUTPUT);
	delay(10);

	fd = serialOpen("/dev/ttySOFT0", 9600);
	if(fd < 0){
		Debug::print(LOG_SUMMARY, "lora serial device cannot open\r\n");
		return false;
	}

	//sleep(false);
	//serialPuts(fd,"1\r\n");
	//serialPuts(fd,"start\r\n");
	send("1");
	send("start");
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

Lora::Lora(): mLastUpdateTime()
{
	setName("lora");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_SENSOR);
}

Lora::~Lora()
{
}
