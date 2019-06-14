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
        return false;
    }
    pinMode(ENIN_ID, OUTPUT);

	/* シリアルポートオープン */
	fd = serialOpen("/dev/ttyAMA0",115200);
	if(fd < 0){
		printf("can not open serialport");
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
servo move (id, raw_value[3500-11500])  : \r\n\
servo move (name, raw_value[3500-11500]): \r\n\
servo free                  			: \r\n\
servo free (id)                			: \r\n\
servo free (name)              			: \r\n\
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
				raw_value = atoi(args[3].c_str());
				free(str_second_input);
				Debug::print(LOG_PRINT, "Servo Free %d\r\n", id);
			}
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
		break;
	case 4:
		if (args[1].compare("move") == 0)
			{
				int id = -1;
				int raw_value = -1;
				std::string str_second_input = args[2]; 

				if(String::check_int(str_second_input)) {
					id = atoi(str_second_input.c_str());
					raw_value = atoi(args[3].c_str());
					move(id, raw_value);
					Debug::print(LOG_PRINT, "Servo Move %d %d\r\n", id, raw_value);
				}else{
					raw_value = atoi(args[3].c_str());
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

}
void Servo::turn(int value){

}
void Servo::move(int id, int raw_value){

}
void Servo::move(std::string servo_name, int raw_value){

}
void Servo::free(int id){

}
void Servo::free(std::string servo_name){

}
void Servo::free(){

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
