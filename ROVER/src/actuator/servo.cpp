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
    // if(wiringPiSetupGpio() == -1) {
	// 	Debug::print(LOG_SUMMARY, "cannot setup wiringpi\r\n");
    //     return false;
    // }
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

	//move(NECK_ID, mServoRawData.neck);
	//move(DIRECT_ID, mServoRawData.direct);
	//move(WAIST_ID, mServoRawData.waist);
	//move(STABI_ID, mServoRawData.stabi);

	mLastUpdateTime = time;
}
bool Servo::onCommand(const std::vector<std::string>& args)
{
	if (args[0].compare(getName()) != 0) return true;

	//Debug::print(LOG_PRINT, "current para:%d direct:%d\r\n", mParaServoPulseWidth, mDirectServoPulseWidth);

	switch (args.size())
	{
	case 1:
		showRangeData();
		Debug::print(LOG_PRINT,
			"\r\n\
servo wrap (range[-1.0,1.0])            : -1 is outer, 1 is inner, 0 is run style \r\n\
servo turn (range[-1.0,1.0])            : -1 is left, 1 is right \r\n\
servo (id, raw_value[3500,11500])       : \r\n\
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
	wrap(NECK_NAME, range);
	move(DIRECT_ID, DIRECT_CENTER);
	wrap(WAIST_NAME, range);
	wrap(STABI_NAME, range);
}

void Servo::wrapWithoutDirect(double range){
	wrap(NECK_NAME, range);
	wrap(WAIST_NAME, range);
	wrap(STABI_NAME, range);
}

void Servo::wrap(std::string servo_name, double range){
	 if(range < -1 || range > 1){
	 	Debug::print(LOG_PRINT, "Please range is from -1 to 1.\r\n");
	 	return;
	 }

	int servo_id = getServoID(servo_name);
	int outer_value = getServoOuterValue(servo_name);
	int inner_value = getServoInnerValue(servo_name);
	int center_value = getServoCenterValue(servo_name);

	if(range <= 0){
	 	move(servo_id, translateToRawValue(range, outer_value, center_value, -1));
	 	return;
	}
	
	if(range > 0){
	 	move(servo_id, translateToRawValue(range, inner_value, center_value, 1));
	 	return;
	}	
}

void Servo::turn(double range){
	if(range < -1 || range > 1){
		Debug::print(LOG_PRINT, "Please range is from -1 to 1.\r\n");
		return;
	}

	if(range <= 0){
		move(DIRECT_ID, translateToRawValue(range, DIRECT_LEFT, DIRECT_CENTER, -1));
	}

	if(range > 0){
		move(DIRECT_ID, translateToRawValue(range, DIRECT_RIGHT, DIRECT_CENTER, 1));
	}
}

double Servo::translateToRange(int raw_value, int end_value, int center_value, double end_range){
	return ((0.0 - end_range) / (double)(center_value - end_value)) * (double)(raw_value - center_value);
}

int Servo::translateToRawValue(double range, int end_value, int center_value, double end_range){
	return (int)(center_value + ((center_value - end_value) / (0.0 - end_range)) * range);
}

void Servo::registRangeData(int id, int raw_value){
	if(id == NECK_ID){
		mServoRawData.neck = raw_value;
	}else if(id == DIRECT_ID){
		mServoRawData.direct = raw_value;
	}else if(id == WAIST_ID){
		mServoRawData.waist = raw_value;
	}else if(id == STABI_ID){
		mServoRawData.stabi = raw_value;
	}
}

void Servo::waitingHoldPara(){
	wrap(1.0);
	turn(-1.0);
}
void Servo::holdPara(){
	wrap(1.0);
	turn(0.0); 
}
void Servo::releasePara(){
	wrap(1.0);
    turn(1.0); 
   //delay(1000);
   //move(WAIST_ID, 9000);
   //delay(1000);
   //move(WAIST_ID, 5000);
}

void Servo::move(int id, int raw_value){
	if(raw_value == 0){
		free(id);
		return;
	}

	registRangeData(id, raw_value);
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
	registRangeData(id, 0);

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
	if(name == NECK_NAME){
		return NECK_ID;
	}else if(name == DIRECT_NAME){
		return DIRECT_ID;
	}else if(name == WAIST_NAME){
		return WAIST_ID;
	}else if(name == STABI_NAME){
		return STABI_ID;
	}
	return -1;
}

int Servo::getServoOuterValue(std::string name){
	if(name == NECK_NAME){
		return NECK_OUTER;
	}else if(name == WAIST_NAME){
		return WAIST_OUTER;
	}else if(name == STABI_NAME){
		return STABI_OUTER;
	}
	return -1;
}

int Servo::getServoInnerValue(std::string name){
	if(name == NECK_NAME){
		return NECK_INNER;
	}else if(name == WAIST_NAME){
		return WAIST_INNER;
	}else if(name == STABI_NAME){
		return STABI_INNER;
	}
	return -1;
}

int Servo::getServoCenterValue(std::string name){
	if(name == NECK_NAME){
		return NECK_CENTER;
	}else if(name == WAIST_NAME){
		return WAIST_CENTER;
	}else if(name == STABI_NAME){
		return STABI_CENTER;
	}
	return -1;
}

std::string Servo::getServoName(int id){
	if(id == NECK_ID){
		return NECK_NAME;
	}else if(id == DIRECT_ID){
		return DIRECT_NAME;
	}else if(id == WAIST_ID){
		return WAIST_NAME;
	}else if(id == STABI_ID){
		return STABI_NAME;
	}
	return "";
}

void Servo::showRangeData(){
	Debug::print(LOG_PRINT, "%s range = %d [%d, %d]\r\n", NECK_NAME.c_str(), mServoRawData.neck, NECK_OUTER, NECK_INNER);
	Debug::print(LOG_PRINT, "%s range = %d [%d, %d]\r\n", DIRECT_NAME.c_str(), mServoRawData.direct, DIRECT_LEFT, DIRECT_RIGHT);
	Debug::print(LOG_PRINT, "%s range = %d [%d, %d]\r\n", WAIST_NAME.c_str(), mServoRawData.waist, WAIST_OUTER, WAIST_INNER);
	Debug::print(LOG_PRINT, "%s range = %d [%d, %d]\r\n", STABI_NAME.c_str(), mServoRawData.stabi, STABI_OUTER, STABI_INNER);
}

Servo::Servo(): mLastUpdateTime(), mServoRawData{0, 0, 0, 0}
{
	setName("servo");
	setPriority(TASK_PRIORITY_ACTUATOR, TASK_INTERVAL_MOTOR);
}
Servo::~Servo()
{
}

