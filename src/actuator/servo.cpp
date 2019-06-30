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

	Debug::print(LOG_SUMMARY, "Servo is Ready!\r\n");
	return true;
}
void Servo::onClean()
{
	free();
	serialClose(fd);
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
<<<<<<< HEAD
servo wrap (range[-1.0,1.0])                : -1 is outer, 1 is inner, 0 is run style \r\n\
servo turn (range[-1.0,1.0])                : -1 is left, 1 is right \r\n\
servo (id, raw_value[3500,11500])       : \r\n\
=======
servo wrap (value [-1,1])               : -1 is outer, 1 is inner, 0 is run style \r\n\
servo turn (value [-1,1])               : -1 is left, 1 is right \r\n\
servo (id, raw_value[3500-11500])  		: \r\n\
>>>>>>> 1e5699d738261a433e39924bd17966a082bd4e04
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
			double range = 0.0;
			range = atof(args[2].c_str());
			if(range <-1 || range >1) break;
			wrap(range);	
			Debug::print(LOG_PRINT, "Servo Wrap %f\r\n", range);
			return true;
		}
		else if (args[1].compare("turn") == 0)
		{
			double range = 0.0;
			range = atof(args[2].c_str());
			if(range <-1 || range >1) break;
			turn(range);	
			Debug::print(LOG_PRINT, "Servo Turn %f\r\n", range);
			return true;
		}
		else
		{
			int id = -1;
			int raw_value = -1;
			std::string str_second_input = args[1];//id or name 

			if(String::check_int(str_second_input)) {
				id = atoi(str_second_input.c_str());
				raw_value = atoi(args[2].c_str());
				move(id, raw_value);
				Debug::print(LOG_PRINT, "Servo Move %d %d\r\n", id, raw_value);
			}else{
				raw_value = atoi(args[2].c_str());
				move(str_second_input, raw_value);
				Debug::print(LOG_PRINT, "Servo Move %s %d\r\n", str_second_input, raw_value);
			}
			return true;
		}
		}
	Debug::print(LOG_PRINT, "Failed Command\r\n");
	return false;
}

void Servo::wrap(double range){
	if(range < -1 || range > 1){
		Debug::print(LOG_PRINT, "Please range is from -1 to 1.\r\n");
		return;
	}

<<<<<<< HEAD
	if(range <= 0){
		move(NECK_ID, translateToRawValue(range, -1, NECK_CENTER, NECK_OUTER));
		move(DIRECT_ID, DIRECT_CENTER);
		move(WAIST_ID, translateToRawValue(range, -1, WAIST_CENTER, WAIST_OUTER));
		move(STABI_ID, translateToRawValue(range, -1, STABI_CENTER, STABI_OUTER));
		return;
	}
	
	if(range > 0){
		move(NECK_ID, translateToRawValue(range, 1, NECK_CENTER, NECK_INNER));
		move(DIRECT_ID, DIRECT_CENTER);
		move(WAIST_ID, translateToRawValue(range, 1, WAIST_CENTER, WAIST_INNER));
		move(STABI_ID, translateToRawValue(range, 1, STABI_CENTER, STABI_INNER));
=======
	if(value <= 0){
		move(NECK_ID, NECK_CENTER + (int)(((NECK_CENTER - NECK_INNER) / (0.0 + 1.0)) * value));
		move(DIRECT_ID, DIRECT_CENTER);
		move(WAIST_ID, WAIST_CENTER +(int)(((WAIST_CENTER - WAIST_INNER) / (0.0 + 1.0)) * value));
		move(STABI_ID, STABI_CENTER +(int)(((STABI_CENTER - STABI_INNER) / (0.0 + 1.0)) * value));
		return;
	}
	
	if(value > 0){
		move(NECK_ID, NECK_CENTER + (int)(((NECK_CENTER - NECK_OUTER) / (0.0 - 1.0)) * value));
		move(DIRECT_ID, DIRECT_CENTER);
		move(WAIST_ID, WAIST_CENTER + (int)(((WAIST_CENTER - WAIST_OUTER) / (0.0 - 1.0)) * value));
		move(STABI_ID, STABI_CENTER + (int)(((STABI_CENTER - STABI_OUTER) / (0.0 - 1.0)) * value));
>>>>>>> 1e5699d738261a433e39924bd17966a082bd4e04
		return;
	}	
}
void Servo::turn(double range){
	if(range < -1 || range > 1){
		Debug::print(LOG_PRINT, "Please range is from -1 to 1.\r\n");
		return;
	}

<<<<<<< HEAD
	if(range <= 0){
		move(DIRECT_ID, translateToRawValue(range, -1, DIRECT_CENTER, DIRECT_LEFT));
	}

	if(range > 0){
		move(DIRECT_ID, translateToRawValue(range, 1, DIRECT_CENTER, DIRECT_RIGHT));
	}
}

double Servo::translateToRange(int raw_value, int end_value, int center_value, double end_range){
	return ((0.0 - end_range) / (double)(center_value - end_value)) * (double)(raw_value - center_value);
=======
	if(value <= 0){
		move(DIRECT_ID, DIRECT_CENTER + (int)(((DIRECT_CENTER - DIRECT_LEFT) / (0.0 + 1.0)) * value));
	}

	if(value > 0){
		move(DIRECT_ID, DIRECT_CENTER + (int)(((DIRECT_CENTER - DIRECT_RIGHT) / (0.0 - 1.0)) * value));
	}

	move(DIRECT_ID, DIRECT_RIGHT + (int)(((DIRECT_RIGHT - DIRECT_LEFT) / (-1.0 - 1.0)) * (value + 1)));
>>>>>>> 1e5699d738261a433e39924bd17966a082bd4e04
}

int Servo::translateToRawValue(int range, int end_value, int center_value, double end_range){
	return (int)(center_value + ((center_value - end_value) / (0.0 - end_range)) * range);
}

void Servo::move(int id, int raw_value){
	digitalWrite(ENIN_ID, 0);
	digitalWrite(ENIN_ID, 1);
	serialPutchar(fd,0x80 | (id & 0x1f));
	serialPutchar(fd,(raw_value >> 7) & 0x7f);
	serialPutchar(fd, raw_value & 0x7f);
	serialFlush(fd);
	delay(100);
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
	serialFlush(fd);
	delay(10);
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
Servo::Servo(): mLastUpdateTime(), neck_range(), direct_range(), waist_range(), stabi_range()
{
	setName("servo");
	//setPriority(TASK_PRIORITY_ACTUATOR, UINT_MAX);
	setPriority(TASK_PRIORITY_ACTUATOR, TASK_INTERVAL_MOTOR);
}
Servo::~Servo()
{
}

