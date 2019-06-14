#include <time.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <termios.h>

#include "../rover_util/utils.h"
#include "servo.h"


Servo gServo;
bool Servo::onInit(const struct timespec& time)
{
    if(wiringPiSetupGpio() == -1) {
		Debug::print(LOG_SUMMARY, "cannot setup wiringpi\r\n");
        return false;
    }
    pinMode(ENIN_ID, OUTPUT);

	/* シリアルポートオープン */
	fd = serialOpen("/dev/ttyAMA0",115200);
	if(fd < 0){
		Debug::print(LOG_SUMMARY, "serial device cannot open\r\n");
	}
	struct termios options ;

	tcgetattr (fd, &options) ;   // Read current options
	options.c_cflag &= ~CSIZE ;  // Mask out size
	options.c_cflag |= CS8 ;
	options.c_cflag |= PARENB ;  // Enable Parity - even by default
	tcsetattr (fd,TCSANOW, &options) ;   // Set new options
	delay(1000);

	return true;
}
void Servo::onClean()
{
	free();
}
void Servo::onUpdate(const timespec & time)
{
	double dt = Time::dt(time, mLastUpdateTime);
	if (dt < SERVO_UPDATE_INTERVAL_TIME)return;
	mLastUpdateTime = time;

}
bool Servo::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	//Debug::print(LOG_PRINT, "current para:%d direct:%d\r\n", mParaServoPulseWidth, mDirectServoPulseWidth);

	switch (args.size())
	{
	case 1:
		Debug::print(LOG_PRINT,
			"\r\n\
servo wrap (value [-1,1])               : \r\n\
servo turn (value [-1,1])               : \r\n\
servo (id, raw_value[3500-11500])  		: \r\n\
servo (name, raw_value[3500-11500])		: \r\n\
servo free                  			: \r\n\
servo free (id)                			: \r\n\
servo free (name)              			: \r\n\
servo getid (name)             			: \r\n\
");
		return true;
	case 2:
		if (args[1].compare("free") == 0)
		{
			free();
			Debug::print(LOG_PRINT, "Servo Free\r\n");
			return true;
		}
		break;
	case 3:
		if (args[1].compare("free") == 0)
		{
			int id = -1;
			int raw_value = -1;
			std::string str_second_input = args[2]; 
			if(String::check_int(str_second_input)) {
				id = atoi(str_second_input.c_str());
				free(id);
				Debug::print(LOG_PRINT, "Servo Free %d\r\n", id);
			}else{
				free(str_second_input);
				Debug::print(LOG_PRINT, "Servo Free %d\r\n", id);
			}
			return true;
		}
		else if (args[1].compare("getid") == 0)
		{
			std::string servo_name = args[2];
			int servo_id = getServoID(servo_name); 
			Debug::print(LOG_PRINT, "Servo ID is %d\r\n", servo_id);
			return true;
		}
		else if (args[1].compare("wrap") == 0)
		{
			double value = 0.0;
			value = atoi(args[2].c_str());
			if(value <-1 || value >1) break;
			wrap(value);	
			Debug::print(LOG_PRINT, "Servo Wrap %f\r\n", value);
			return true;
		}
		else if (args[1].compare("turn") == 0)
		{
			double value = 0.0;
			value = atoi(args[2].c_str());
			if(value <-1 || value >1) break;
			turn(value);	
			Debug::print(LOG_PRINT, "Servo Turn %f\r\n", value);
			return true;
		}
		else
		{
			int id = -1;
			int raw_value = -1;
			std::string str_second_input = args[1]; 

			if(String::check_int(str_second_input)) {
				id = atoi(str_second_input.c_str());
				raw_value = atoi(args[2].c_str());
				move(id, raw_value);
				Debug::print(LOG_PRINT, "Servo Move %d %d\r\n", id, raw_value);
			}else{
				raw_value = atoi(args[2].c_str());
				move(str_second_input, raw_value);
				Debug::print(LOG_PRINT, "Servo Move %d %d\r\n", id, raw_value);
			}
			return true;
		}
		}
	Debug::print(LOG_PRINT, "Failed Command\r\n");
	return false;
}

void Servo::wrap(int value){
	if(value < -1 || value > 1){
		Debug::print(LOG_PRINT, "Please value is from -1 to 1.\r\n");
		return;
	}

	move(NECK_ID, NECK_INNER + ((NECK_INNER - NECK_OUTER) / (-1 - 1)) * (value + 1));
	move(DIRECT_ID, DIRECT_CENTER);
	move(STABI_ID, STABI_INNER + ((STABI_INNER - STABI_OUTER) / (-1 - 1)) * (value + 1));
}
void Servo::turn(int value){

}
void Servo::move(int id, int raw_value){
	digitalWrite(ENIN_ID, 0);
	digitalWrite(ENIN_ID, 1);
	serialPutchar(fd,0x80 | (id & 0x1f));
	serialPutchar(fd,(raw_value >> 7) & 0x7f);
	serialPutchar(fd, raw_value & 0x7f);
}
void Servo::move(std::string servo_name, int raw_value){
	int servo_id = getServoID(servo_name);
	move(servo_id, raw_value);	
}

void Servo::free(int id){
	digitalWrite(ENIN_ID, 0);
	digitalWrite(ENIN_ID, 1);
    serialPutchar(fd, (0x80 | id));
    serialPutchar(fd, 0x00);
    serialPutchar(fd, 0x00);
    serialPutchar(fd, 0x00);
}
void Servo::free(std::string servo_name){
	int servo_id = getServoID(servo_name);
	free(servo_id);
}
void Servo::free(){
	free(NECK_ID);
	free(DIRECT_ID);
	free(WAIST_ID);
	free(STABI_ID);
}
int Servo::getServoID(std::string name){
	if(name == "neck"){
		return NECK_ID;
	}else if(name == "direct"){
		return DIRECT_ID;
	}else if(name == "waist"){
		return WAIST_ID;
	}else if(name == "stabi"){
		return STABI_ID;
	}else{
		return -1;
	}
}
Servo::Servo()
{
	setName("servo");
	//setPriority(TASK_PRIORITY_ACTUATOR, UINT_MAX);
	setPriority(TASK_PRIORITY_ACTUATOR, TASK_INTERVAL_MOTOR);
}
Servo::~Servo()
{
}
